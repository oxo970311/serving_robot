import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class TableNumber(Node):
    def __init__(self):
        super().__init__('table_number')
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Int32, '/table_id', 10)
        self.get_logger().info('TableNumber node started.')

    def publish_number(self, number: int):
        msg = Int32()
        msg.data = number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published table number: {number}')


def main(args=None):
    rclpy.init(args=args)
    node = TableNumber()

    try:
        while rclpy.ok():
            user_input = input('Enter table number (1-6, q to quit): ').strip()

            if user_input.lower() == 'q':
                print('Quit.')
                break

            if not user_input.isdigit():
                print('Number only.')
                continue

            number = int(user_input)
            if number < 1 or number > 6:
                print('Use 1~6 only.')
                continue

            node.publish_number(number)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
