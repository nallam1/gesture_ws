#For manual testing while sEMG based gesture recognition is being completed
import rclpy
from rclpy.node import Node
from gesture_msgs.msg import HandCommand

class ManualGesturePublisher(Node):
    def __init__(self):
        super().__init__('manual_gesture_publisher')
        self.publisher_ = self.create_publisher(HandCommand, 'hand_command', 10)

        # Timer to repeatedly ask for user input
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Manual gesture publisher node started.')

    def timer_callback(self):
        try:
            hand = input("Enter hand ('left' or 'right'): ")
            flex_ext = float(input("Enter flex/extension (-1.0 to 1.0): "))
            pron_supi = float(input("Enter pronation/supination (-1.0 to 1.0): "))
            grip_state = float(input("Enter grip state (0.0 open to 1.0 closed): "))

            msg = HandCommand()
            msg.hand = hand
            msg.flex_ext = flex_ext
            msg.pron_supi = pron_supi
            msg.grip_state = grip_state

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: hand={hand}, flex_ext={flex_ext}, pron_supi={pron_supi}, grip_state={grip_state}')

        except Exception as e:
            self.get_logger().error(f'Error in input: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ManualGesturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

