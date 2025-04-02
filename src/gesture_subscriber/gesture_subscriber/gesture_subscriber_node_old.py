import rclpy
from rclpy.node import Node
from gesture_msgs.msg import HandCommand  # Updated message import

class GestureSubscriber(Node):
    def __init__(self):
        super().__init__('gesture_subscriber')
        self.subscription = self.create_subscription(
            HandCommand,
            'hand_command',
            self.listener_callback,
            10
        )
        self.get_logger().info('HandCommand subscriber node started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received- hand: {msg.hand}, {msg.gesture}, '
                               f'flex_ext={msg.flex_ext}, '
                               f'pron_supi={msg.pron_supi}, grip_state={msg.grip_state}')
        #deviation={msg.deviation}, 

def main(args=None):
    rclpy.init(args=args)
    node = GestureSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

