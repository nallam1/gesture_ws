# This is a better test including automation of multiple moves with looping, delays, and gripper control which is
# independent of manual_gesture_publisher.py (node for testing specific single tasks through direct manual input)
# and independent of the main sEMG based publisher node: gesture_publisher_node.py

import rclpy
from rclpy.node import Node
from gesture_msgs.msg import HandCommand
import time
import argparse

class GestureCommandPublisher(Node):
    def __init__(self, args):
        super().__init__('manual_gesture_publisher')
        self.publisher_ = self.create_publisher(HandCommand, 'hand_command', 10)

        self.args = args
        self.rate = args.delay
        self.repeats = args.repeats

        self.get_logger().info(f'Gesture command publisher started. Repeats: {self.repeats}, Delay: {self.rate} sec')

    def publish_commands(self):
        count = 0
        try:
            while self.repeats == -1 or count < self.repeats:
                msg = HandCommand()
                msg.hand = "right"  # or 'left' depending on test
                msg.gesture = "custom_sequence"

                # Example: flexion-extension + pronation-supination cycling
                msg.flex_ext = 30.0 if (count % 2 == 0) else -30.0
                msg.pron_supi = 45.0 if (count % 2 == 0) else -45.0

                # Gripper control (fully open / close based on count)
                msg.grip_state = 1.0 if (self.args.gripper and count % 2 == 0) else 0.0

                self.publisher_.publish(msg)
                self.get_logger().info(
                    f'Published command [{count}]: flex_ext={msg.flex_ext}, pron_supi={msg.pron_supi}, grip_state={msg.grip_state}'
                )

                count += 1
                time.sleep(self.rate)

        except KeyboardInterrupt:
            self.get_logger().info('Gesture command publishing stopped by user.')

        self.get_logger().info('Exiting publisher node.')
        self.destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Manual Gesture Command Publisher for Teleoperation')
    parser.add_argument('--repeats', type=int, default=-1, help='Number of repetitions (-1 for infinite)')
    parser.add_argument('--delay', type=float, default=1.0, help='Delay between commands in seconds')
    parser.add_argument('--gripper', action='store_true', help='Enable gripper open/close control')

    args = parser.parse_args()

    rclpy.init()
    node = GestureCommandPublisher(args)
    node.publish_commands()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


