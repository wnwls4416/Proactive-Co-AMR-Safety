import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist, PointStamped, Point
from std_msgs.msg import Int32
from nav2_msgs.action import NavigateToPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer

class PC3FinalNode(Node):
    def __init__(self):
        super().__init__('final_stage3_4_node')
        
        self.bridge = CvBridge()
        self.model = YOLO("/home/rokey/rokey_ws/src/pc3/models/best.pt")
        
        self.ns = 'robot4'
        self.target_frame = 'map'
        self.camera_frame = 'oakd_rgb_camera_optical_frame'

        # Publishers & Action
        self._action_client = ActionClient(self, NavigateToPose, f'/{self.ns}/navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.pub_topic1 = self.create_publisher(Int32, 'topic1', 10) # Class ID
        self.pub_topic2 = self.create_publisher(Point, 'topic2', 10) # Map X, Y
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 1단계: 정렬용 Preview 구독
        self.preview_sub = self.create_subscription(Image, f'/{self.ns}/oakd/rgb/preview/image_raw', self.preview_callback, 10)
        
        # 2단계: 정밀 계산용 고해상도 동기화 (704x704)
        self.rgb_sub = Subscriber(self, CompressedImage, f'/{self.ns}/oakd/rgb/preview/image_raw/compressed')
        self.depth_sub = Subscriber(self, Image, f'/{self.ns}/oakd/stereo/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.2)
        self.ts.registerCallback(self.synchronized_callback)

        self.create_subscription(Int32, '/stage3_start', self.start_callback, 10)
        self.create_subscription(CameraInfo, f'/{self.ns}/oakd/rgb/preview/camera_info', self.info_callback, 10)

        # 제어 상태 플래그
        self.is_active = False
        self.alignment_done = False # 정렬 완료 여부
        self.execution_completed = False # 단 한 번의 실행 보장
        self.camera_info = None
        self.preview_center_x = 160 # 320px의 절반
        self.align_threshold = 10 

    def info_callback(self, msg):
        self.camera_info = msg

    def start_callback(self, msg):
        if msg.data == 1:
            self.get_logger().info("🔍 탐색 단계 진입: 시계방향 회전하며 객체 탐색 중...")
            self.is_active = True

    def preview_callback(self, msg):
        """1단계: 정렬 완료 전까지만 Preview YOLO 연산 수행"""
        if not self.is_active or self.alignment_done or self.execution_completed:
            return

        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_img, conf=0.5, verbose=False)
        
        twist = Twist()
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            center_x = (box[0] + box[2]) / 2
            error = self.preview_center_x - center_x
            
            if abs(error) < self.align_threshold:
                self.get_logger().info("🎯 정렬 완료! 고해상도 정밀 계산으로 전환합니다.")
                self.stop_robot()
                self.alignment_done = True # 이후 preview_callback은 리턴됨 (연산 중단)
            else:
                twist.angular.z = 0.005 * error
                self.cmd_vel_pub.publish(twist)
        else:
            twist.angular.z = -0.4 # 시계방향 회전
            self.cmd_vel_pub.publish(twist)

    def synchronized_callback(self, rgb_msg, depth_msg):
        """2단계: 정렬 직후 단 한 번만 실행되는 고해상도 Map Transform 로직"""
        if not self.alignment_done or self.execution_completed:
            return

        try:
            # 1. 고해상도 이미지 해제 및 탐지
            rgb_frame = self.bridge.compressed_imgmsg_to_cv2(rgb_msg)
            results = self.model(rgb_frame, conf=0.6, verbose=False)

            if len(results[0].boxes) > 0:
                self.get_logger().info("📸 고해상도 정밀 측정 실시...")
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                cls_id = int(results[0].boxes[0].cls[0])
                
                # 2. Stereo 이미지 처리 (704x704)
                depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
                x1, y1, x2, y2 = map(int, box)
                roi_depth = depth_frame[y1:y2, x1:x2]
                valid_depths = roi_depth[roi_depth > 0]
                
                if len(valid_depths) > 0:
                    dist_m = np.min(valid_depths) / 1000.0
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # 3. Map Transform 및 토픽 발행
                    self.process_final_move(u, v, dist_m, cls_id)
                    
                    # 4. 종료 플래그 설정 (다시는 실행되지 않음)
                    self.execution_completed = True
                    self.is_active = False
            else:
                self.get_logger().warn("⚠️ 정밀 단계에서 객체를 놓쳤습니다. 재정렬을 시도합니다.")
                self.alignment_done = False # 다시 Preview 정렬로 복구

        except Exception as e:
            self.get_logger().error(f"❌ 정밀 계산 중 오류: {e}")

    def process_final_move(self, u, v, dist, cls_id):
        if self.camera_info is None: return
        
        # 카메라 좌표계 상의 XYZ
        fx, fy, cx, cy = self.camera_info.k[0], self.camera_info.k[4], self.camera_info.k[2], self.camera_info.k[5]
        z = float(dist)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        # 0.7m 앞 지점 계산
        d = math.sqrt(x**2 + y**2 + z**2)
        ratio = max(0.0, (d - 0.7) / d) if d > 0.7 else 0.0
        
        p = PointStamped()
        p.header.frame_id = self.camera_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.point.x, p.point.y, p.point.z = x * ratio, y * ratio, z * ratio

        try:
            # Map 프레임 변환
            t = self.tf_buffer.lookup_transform(self.target_frame, self.camera_frame, rclpy.time.Time())
            p_map = tf2_geometry_msgs.do_transform_point(p, t)
            
            # [토픽 발행] 단 한 번만 수행됨
            self.pub_topic1.publish(Int32(data=cls_id))
            self.pub_topic2.publish(Point(x=p_map.point.x, y=p_map.point.y, z=0.0))
            
            self.get_logger().info(f"✅ 발행 완료: Class={cls_id}, Map=({p_map.point.x:.2f}, {p_map.point.y:.2f})")
            
            # 이동 명령 전송
            self.send_goal(p_map.point.x, p_map.point.y)
        except Exception as e:
            self.get_logger().error(f"TF 변환 실패: {e}")

    def send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.target_frame
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self._action_client.send_goal_async(goal)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main():
    rclpy.init()
    node = PC3FinalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
