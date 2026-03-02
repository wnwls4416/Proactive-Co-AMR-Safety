import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import Twist, PointStamped, Point
from irobot_create_msgs.action import Dock
from std_msgs.msg import Int32, String 
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
import time
from message_filters import Subscriber, ApproximateTimeSynchronizer
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class PC3FinalNode(Node):
    def __init__(self):
        super().__init__('final_stage3_4_node')
        
        self.bridge = CvBridge()
        self.model = YOLO("/home/rokey/rokey_ws/src/pc3/models/best.pt")
        
        self.ns = 'robot5'
        # 팁: 네임스페이스 환경에서는 프레임 이름 확인이 필요할 수 있습니다.
        self.target_frame = 'map' 
        self.camera_frame = 'oakd_rgb_camera_optical_frame'

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._action_client = ActionClient(self, NavigateToPose, f'/{self.ns}/navigate_to_pose')
        self._dock_client = ActionClient(self, Dock, f'/{self.ns}/dock')
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.point_pub = self.create_publisher(Point, '/signal_amr1_map', self.qos)
        self.obj_id_pub = self.create_publisher(Int32, '/signal_amr1_class', self.qos)
        self.create_subscription(Int32, '/finish_signal_2', self.finish_callback, self.qos)
        
        # [핵심 수정] TF 버퍼와 리스너를 올바르게 연결
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.preview_sub = self.create_subscription(Image, f'/{self.ns}/oakd/rgb/preview/image_raw', self.preview_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, f'/{self.ns}/oakd/stereo/image_raw', self.depth_callback, qos_profile=sensor_qos)
        self.create_subscription(CameraInfo, f'/{self.ns}/oakd/rgb/preview/camera_info', self.info_callback, qos_profile=sensor_qos)
        self.create_subscription(Int32, '/stage3_start', self.start_callback, 10)

        self.is_active = False
        self.alignment_done = False
        self.found_it = False
        self.camera_info = None
        self.preview_center_x = 160
        self.align_threshold = 15
        self.is_returning_home = False
        
        self.latest_detection = None
        self.depth_samples = []
        self.measurement_timer = None

    def info_callback(self, msg):
        self.camera_info = msg

    def start_callback(self, msg):
        if msg.data == 1 and not self.is_active and not self.found_it:
            self.get_logger().info("1단계: 객체 탐색 및 중앙 정렬 시작")
            self.is_active = True

    def finish_callback(self, msg):
        if msg.data == 1 and not self.is_returning_home:
            self.get_logger().info("종료 신호 수신. 복귀 시작")
            self.is_returning_home = True
            self.go_home()

    def go_home(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.target_frame
        goal_msg.pose.pose.position.x = -0.3
        goal_msg.pose.pose.position.y = -0.3
        goal_msg.pose.pose.orientation.w = 1.0
        self._action_client.send_goal_async(goal_msg).add_done_callback(self.home_response_callback)

    def home_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            goal_handle.get_result_async().add_done_callback(self.home_result_callback)

    def home_result_callback(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("복귀 완료. 도킹 시작")
            self.dock_robot()

    def dock_robot(self):
        self._dock_client.wait_for_server()
        self._dock_client.send_goal_async(Dock.Goal())

    def preview_callback(self, msg):
        if not self.is_active or self.found_it:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, conf=0.5, verbose=False)
        
        if self.alignment_done:
            if len(results[0].boxes) > 0:
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                self.latest_detection = {
                    'box': [int(box[0]), int(box[1]), int(box[2]), int(box[3])],
                    'class': int(results[0].boxes[0].cls.item()),
                    'center': ((box[0] + box[2]) // 2, (box[1] + box[3]) // 2)
                }
            return
        
        twist = Twist()
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            error = self.preview_center_x - (box[0] + box[2]) / 2
            
            if abs(error) < self.align_threshold:
                self.get_logger().info("정렬 완료! 측정 모드 진입")
                self.cmd_vel_pub.publish(Twist())
                self.alignment_done = True
                self.measurement_timer = self.create_timer(1.0, self.process_depth_measurement)
            else:
                twist.angular.z = 0.005 * error
                self.cmd_vel_pub.publish(twist)
        else:
            twist.angular.z = -0.4
            self.cmd_vel_pub.publish(twist)

    def depth_callback(self, msg):
        if not self.is_active or self.found_it or not self.alignment_done:
            return
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if self.latest_detection:
                box = self.latest_detection['box']
                scale_x, scale_y = depth_frame.shape[1] / 320.0, depth_frame.shape[0] / 320.0
                roi = depth_frame[int(box[1]*scale_y):int(box[3]*scale_y), int(box[0]*scale_x):int(box[2]*scale_x)]
                valid = roi[(roi > 0) & (roi < 10000)]
                if len(valid) > 10:
                    self.depth_samples.append(np.median(valid))
        except:
            pass

    def process_depth_measurement(self):
        if self.measurement_timer:
            self.measurement_timer.cancel()
            self.measurement_timer = None
        
        if not self.depth_samples:
            self.alignment_done = False
            return

        dist_m = np.mean(self.depth_samples) / 1000.0
        u, v = self.latest_detection['center']
        self.obj_id_pub.publish(Int32(data=self.latest_detection['class']))
        self.calculate_and_move(u, v, dist_m)
        self.found_it = True
        self.is_active = False

    def calculate_and_move(self, u, v, dist):
        if self.camera_info is None: return
        fx, fy, cx, cy = self.camera_info.k[0], self.camera_info.k[4], self.camera_info.k[2], self.camera_info.k[5]
        z = float(dist)
        x, y = (u - cx) * z / fx, (v - cy) * z / fy
        
        try:
            # rclpy.time.Time(seconds=0)는 가장 최신 데이터를 의미합니다.
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.camera_frame, rclpy.time.Time(seconds=0), timeout=rclpy.duration.Duration(seconds=1.0))
            
            p_obj = PointStamped()
            p_obj.header.frame_id, p_obj.point.x, p_obj.point.y, p_obj.point.z = self.camera_frame, x, y, z
            map_obj = tf2_geometry_msgs.do_transform_point(p_obj, transform)
            self.point_pub.publish(Point(x=map_obj.point.x, y=map_obj.point.y))

            d = math.sqrt(x**2 + y**2 + z**2)
            ratio = max(0.0, (d - 0.7) / d)
            p_nav = PointStamped()
            p_nav.header.frame_id, p_nav.point.x, p_nav.point.y, p_nav.point.z = self.camera_frame, x*ratio, y*ratio, z*ratio
            map_nav = tf2_geometry_msgs.do_transform_point(p_nav, transform)
            # self.point_pub.publish(Point(x=map_nav.point.x, y=map_nav.point.y))
            
            self.send_goal(map_nav.point.x, map_nav.point.y)
        except Exception as e:
            self.get_logger().error(f"TF 오류: {e}")

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.target_frame
        goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y, goal_msg.pose.pose.orientation.w = x+0.2, y+0.2, 1.0
        self._action_client.send_goal_async(goal_msg)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

def main():
    rclpy.init()
    rclpy.spin(PC3FinalNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()