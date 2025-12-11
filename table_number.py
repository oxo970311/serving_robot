import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from time import time
import sys
import select


class TableNumber(Node):
    def __init__(self):
        super().__init__('table_number')
        self.publisher_ = self.create_publisher(Int32, '/table_id', 10)
        self.get_logger().info('TableNumber node started.')

        self.last_table = None
        self.at_home = True
        self.last_cmd_time = None

    def publish_number(self, number: int):
        msg = Int32()
        msg.data = number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published table number: {number}')

    def handle_input(self, table: int):

        if (self.last_table == table) and (not self.at_home):
            self.get_logger().info(
                f'Same table {table} pressed again -> publish 0 (home)'
            )
            self.publish_number(0)
            self.at_home = True
            self.last_table = None
            self.last_cmd_time = None
            
        else:
            self.get_logger().info(
                f'Go to table {table} -> publish {table}'
            )
            self.publish_number(table)
            self.last_table = table
            self.at_home = False
            self.last_cmd_time = time()


def main(args=None):
    rclpy.init(args=args)
    node = TableNumber()

    try:
        while rclpy.ok():
            print('Enter table number (1-6, q to quit): ', end='', flush=True)
            rlist, _, _ = select.select([sys.stdin], [], [], 1.0)

            if rlist:
                user_input = sys.stdin.readline().strip()

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

                node.handle_input(number)

            else:
                if (not node.at_home) and node.last_cmd_time is not None:
                    elapsed = time() - node.last_cmd_time
                    if elapsed >= 10.0:
                        print("10초 동안 입력 없음 → 자동 홈 복귀")
                        node.publish_number(0)
                        node.at_home = True
                        node.last_table = None
                        node.last_cmd_time = None

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
