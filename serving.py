import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# 1) 테이블별 pose 저장
TABLE_POSES = {
    1: {"x": -0.044681931640118154, "y": 0.056523243210510464,
        "qz": 0.6505781366772044, "qw": 0.759439324816418},
    2: {"x": 0.8822777411351439, "y": 0.4586906938018086,
        "qz": 0.5926069981668277, "qw": 0.8054917415614523},
    3: {"x": -0.03639668308655514, "y": 1.8047287803442704,
        "qz": 0.7263792139036679, "qw": 0.6872941419863037},
    4: {"x": 1.0718175936562062, "y": 1.9990433252684605,
        "qz": 0.6711968473501887, "qw": 0.7412791593638441},
    5: {"x": 0.06291290008173789, "y": 3.523418241758191,
        "qz": 0.6931269268622413, "qw": 0.7208155542567773},
    6: {"x": 1.2428293768289087, "y": 3.8321198986516336,
        "qz": 0.6851778622334155, "qw": 0.7283757938765173},
}

# 시작 위치 (home pose)
HOME_POSE = {
    "x": 0.06063078549529795,
    "y": 0.10859636429333965,
    "qz": 0.6530552507915445,
    "qw": 0.7573102662803357,
}


class TableNavigator(Node):
    def __init__(self):
        super().__init__('table_navigator')

        # nav2 액션 클라이언트 (navigate_to_pose)
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # table_id 구독 (예: 1,2,3... / 0 = 복귀)
        self.sub = self.create_subscription(
            Int32,
            'table_id',          # 토픽 이름
            self.table_callback, # 콜백 함수
            10
        )

        self.get_logger().info('TableNavigator node started.')

    def table_callback(self, msg: Int32):
        table_id = msg.data
        self.get_logger().info(f"Received table_id: {table_id}")

        # nav2 액션 서버 준비됐는지 확인
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('navigate_to_pose action server not available!')
            return

        # 0이면 "exit" 개념 → 시작 위치로 복귀
        if table_id == 0:
            pose_info = HOME_POSE
            self.get_logger().info("Returning to HOME pose.")
        else:
            # 존재하는 테이블 번호인지 확인
            if table_id not in TABLE_POSES:
                self.get_logger().warn(f"No pose for table_id {table_id}")
                return
            pose_info = TABLE_POSES[table_id]
            self.get_logger().info(f"Navigating to table {table_id}.")

        # 공통 goal 전송 로직
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = pose_info["x"]
        goal_msg.pose.pose.position.y = pose_info["y"]
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = pose_info["qz"]
        goal_msg.pose.pose.orientation.w = pose_info["qw"]

        self.get_logger().info(
            f"Sending goal: x={pose_info['x']:.3f}, y={pose_info['y']:.3f}"
        )

        send_goal_future = self.client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by nav2.')
            return

        self.get_logger().info('Goal accepted by nav2.')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info('Navigation finished (result received).')
        # 성공/실패 status 보고 싶으면 result.status 로 로그 찍을 수 있음

def main(args=None):
    rclpy.init(args=args)
    node = TableNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
