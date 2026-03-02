import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int32, String 
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from irobot_create_msgs.action import Undock 
from irobot_create_msgs.msg import DockStatus

class PC3Stage12Node(Node):
    def __init__(self):
        super().__init__('pc3_stage1_2_node')

        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, 
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )
        
        self.ns = 'robot4' 
        self._nav_client = ActionClient(self, NavigateToPose, f'/{self.ns}/navigate_to_pose')
        self._undock_client = ActionClient(self, Undock, f'/{self.ns}/undock')

        # 섹션 좌표 설정 (11번 섹션 안전 좌표로 수정)
        self.section_goals = {i: (0.0, 0.0) for i in range(1, 17)}
        self.section_goals[1] = (-0.5, -0.7) 
        self.section_goals[2] = (0.0, 0.0) 
        self.section_goals[3] = (0.0, 0.0) 
        self.section_goals[4] = (0.0, 0.0) 
        self.section_goals[5] = (0.0, 0.0) 
        self.section_goals[6] = (-0.2, -0.7) 
        self.section_goals[7] = (-0.3, -0.9) 
        self.section_goals[8] = (0.0, 0.0)
        self.section_goals[9] = (-1.0, -1.5)
        self.section_goals[10] = (-1.0, -1.5)
        self.section_goals[11] = (-1.01, -4.98)
        self.section_goals[12] = (0.0, 0.0)
        self.section_goals[13] = (0.0, 0.0)
        self.section_goals[14] = (-1.0, -1.5)
        self.section_goals[15] = (-1.0, -1.5)
        self.section_goals[16] = (0.0,0.0)

        self.current_class = 0
        self.current_section = 0
        self.last_completed_section = 0 # 추가: 중복 실행 방지용
        self.is_moving = False
        self.is_docked = False

        self.create_subscription(Int32, '/signal_web2_class', self.class_callback, self.qos)
        self.create_subscription(Int32, '/signal_web2_section', self.section_callback, self.qos)
        self.create_subscription(DockStatus, f'/{self.ns}/dock_status', self.dock_status_callback, self.qos)
        
        self.pub_stage3_start = self.create_publisher(Int32, '/stage3_start', 10)
        self.get_logger().info(f"PC3 Stage 1&2 Node Started. Namespace: {self.ns}")

    def dock_status_callback(self, msg):
        self.is_docked = msg.is_docked

    def class_callback(self, msg):
        self.current_class = msg.data
        if self.current_class == 0:
            self.last_completed_section = 0 # 명령이 해제되면 초기화
        self.check_and_move()

    def section_callback(self, msg):
        self.current_section = msg.data
        self.check_and_move()

    def check_and_move(self):
        # 수정: 이미 완료한 섹션이 아닐 때만 주행 시작
        if self.current_class == 1 and not self.is_moving and self.current_section != self.last_completed_section:
            if 1 <= self.current_section <= 16:
                self.is_moving = True
                if self.is_docked:
                    self.send_undock_goal()
                else:
                    goal_x, goal_y = self.section_goals[self.current_section]
                    self.send_nav_goal(goal_x, goal_y)

    def send_undock_goal(self):
        self._undock_client.wait_for_server()
        self._undock_client.send_goal_async(Undock.Goal()).add_done_callback(self.undock_response_callback)

    def undock_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_moving = False
            return
        goal_handle.get_result_async().add_done_callback(self.undock_result_callback)

    def undock_result_callback(self, future):
        goal_x, goal_y = self.section_goals[self.current_section]
        self.send_nav_goal(goal_x, goal_y)

    def send_nav_goal(self, x, y):
        self._nav_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x, goal_msg.pose.pose.position.y = x, y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"섹션 {self.current_section} ({x}, {y})으로 이동 중...")
        self._nav_client.send_goal_async(goal_msg).add_done_callback(self.nav_response_callback)

    def nav_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_moving = False
            return
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("섹션 도착 완료! 3단계 신호를 보냅니다.")
            self.last_completed_section = self.current_section # 🚀 도착 완료 기록
            self.pub_stage3_start.publish(Int32(data=1))
        else:
            self.get_logger().warn(f"이동 실패 (Status: {status})")
        self.is_moving = False

def main():
    rclpy.init()
    rclpy.spin(PC3Stage12Node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
