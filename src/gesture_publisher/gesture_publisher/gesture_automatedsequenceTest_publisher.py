# Another test node where all gesture parameters hard-coded in a particular sequence to be executed
import rclpy
from rclpy.node import Node
from gesture_msgs.msg import HandCommand
import time

class GestureSequencePublisher(Node):
    def __init__(self):
        super().__init__('gesture_sequence_publisher')
        self.publisher_ = self.create_publisher(HandCommand, 'hand_command', 10)
        self.timer_period = 2.0  # Publish new command every 2 seconds
        self.timer = self.create_timer(self.timer_period, self.publish_next_command)
        self.sequence = self.define_gesture_sequence()
        self.index = 0
        self.get_logger().info("Gesture sequence publisher started!")

    def define_gesture_sequence(self):
        # Define your sequence of HandCommand messages
        sequence = []

        # Open hand
        open_hand = HandCommand()
        open_hand.hand = "right"
        open_hand.gesture = "open"
        open_hand.flex_ext = 0.0
        open_hand.pron_supi = 0.0
        open_hand.grip_state = 0.0
        sequence.append(open_hand)

        # Close hand (fist)
        close_hand = HandCommand()
        close_hand.hand = "right"
        close_hand.gesture = "fist"
        close_hand.flex_ext = 1.0
        close_hand.pron_supi = 0.0
        close_hand.grip_state = 1.0
        sequence.append(close_hand)

        # Flexion
        flex = HandCommand()
        flex.hand = "right"
        flex.gesture = "flex"
        flex.flex_ext = 0.8
        flex.pron_supi = 0.0
        flex.grip_state = 0.5
        sequence.append(flex)

        # Extension
        extend = HandCommand()
        extend.hand = "right"
        extend.gesture = "extend"
        extend.flex_ext = -0.8
        extend.pron_supi = 0.0
        extend.grip_state = 0.0
        sequence.append(extend)

        return sequence

    def publish_next_command(self):
        if self.index >= len(self.sequence):
            self.get_logger().info("Gesture sequence completed.")
            rclpy.shutdown()
            return

        command = self.sequence[self.index]
        self.publisher_.publish(command)
        self.get_logger().info(f'Published gesture: {command.gesture}, flex_ext: {command.flex_ext}, pron_supi: {command.pron_supi}, grip_state: {command.grip_state}')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = GestureSequencePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

