import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
import time
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped 
from nav2_msgs.action import NavigateToPose 
from std_msgs.msg import Int32, String 

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class Stage56Node(Node):
    def __init__(self):
        super().__init__('pc3_stage5_6_node')
        
        self.group = ReentrantCallbackGroup()
        self.navigator = TurtleBot4Navigator()
        
        # QoS 설정 수정 완료 (ReliabilityPolicy -> DurabilityPolicy)
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.topic2_coords = None
        self.is_running = False 

        self.sub_topic1 = self.create_subscription(
            Int32, '/signal_amr2_class', self.topic1_callback, self.qos, callback_group=self.group) # 변경
        self.sub_topic2 = self.create_subscription(
            Point, '/signal_amr2_map', self.topic2_callback, self.qos, callback_group=self.group) # 변경
        self.finish_pub = self.create_publisher(Int32, '/finish_signal_1', self.qos) # 추가
        
        self.get_logger().info('Stage 5 & 6 Node: Ready and waiting for Topic 1...')

    def topic2_callback(self, msg):
        self.topic2_coords = [msg.x, msg.y]

    def topic1_callback(self, msg):
        obj_id = msg.data
        if obj_id != 0 and not self.is_running:
            self.is_running = True 
            self.get_logger().info(f'Object {obj_id} detected! Sequence starting...')
            self.execute_sequence(obj_id)

    def execute_sequence(self, obj_id):
        try:
            initial_pose = self.navigator.getPoseStamped([-0.3,-0.3], TurtleBot4Directions.EAST)
            initial_coord = [-0.3, -0.3] # 추가(도킹전 복귀좌표)
            self.navigator.setInitialPose(initial_pose)
            self.navigator.waitUntilNav2Active()

            if self.navigator.getDockedStatus():
                self.get_logger().info('Executing Undock...')
                self.navigator.undock()
                time.sleep(1.0) 

            first_goal = [-1.2, -3.3] if obj_id in [1, 3] else [-1.8, -3.0]
            
            self.get_logger().info(f'Moving to Goal 1: {first_goal}')
            goal1_pose = self.navigator.getPoseStamped(first_goal, TurtleBot4Directions.NORTH)
            self.navigator.goToPose(goal1_pose)

            while not self.navigator.isTaskComplete():
                time.sleep(0.5)

            self.get_logger().info('Goal 1 reached. Waiting 3s...')
            time.sleep(3)

            if self.topic2_coords:
                self.get_logger().info(f'Moving to Goal 2 (Topic 2): {self.topic2_coords}')
                goal2_pose = self.navigator.getPoseStamped(self.topic2_coords, TurtleBot4Directions.NORTH)
                self.navigator.goToPose(goal2_pose)
                
                while not self.navigator.isTaskComplete():
                    time.sleep(0.5)
                self.get_logger().info('All missions completed.')
                
                #[추가] 5초 대기 후 완료 신호 발행
                self.get_logger().info('Waiting 5 seconds before finish signal...')
                time.sleep(5.0)
                
                finish_msg = Int32()
                finish_msg.data = 1
                self.finish_pub.publish(finish_msg)
                
                self.get_logger().info('Finish signal sent to /finish_signal_1.')

                # 6. 첫 좌표(경유지)로 다시 이동 ##추가
                self.get_logger().info(f'Returning to Goal 1: {first_goal}')
                return_goal1_pose = self.navigator.getPoseStamped(first_goal, TurtleBot4Directions.SOUTH)
                self.navigator.goToPose(return_goal1_pose)
                while not self.navigator.isTaskComplete():
                    time.sleep(0.5)

                # 7. 초기 위치로 이동 ##추가
                self.get_logger().info(f'Returning to Initial Position: {initial_coord}')
                home_pose = self.navigator.getPoseStamped(initial_coord, TurtleBot4Directions.WEST)
                self.navigator.goToPose(home_pose)
                while not self.navigator.isTaskComplete():
                    time.sleep(0.5)

                # 8. 도킹 수행 ##추가
                self.get_logger().info('Starting Docking sequence...')
                self.navigator.dock()
                while not self.navigator.isTaskComplete():
                    time.sleep(0.5)
                self.get_logger().info('Docking completed. Mission truly finished.')
            else:
                self.get_logger().warn('Topic 2 data missing. Mission ending.')

        except Exception as e:
            self.get_logger().error(f'Error in sequence: {e}')
        
        self.is_running = False 

def main(args=None):
    rclpy.init(args=args)
    node = Stage56Node()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 이미 종료된 경우 중복 shutdown 방지를 위해 try-except 처리 권장
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()