import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestNode2(Node):
    def __init__(self):
        super().__init__('test_node2')
        self.sub = self.create_subscription(String,'test_topic_sub', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main():
    rclpy.init()
    node = TestNode2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()