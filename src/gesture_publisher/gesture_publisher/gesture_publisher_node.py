import rclpy
from rclpy.node import Node
from gesture_msgs.msg import HandCommand  # Updated message import

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher_ = self.create_publisher(HandCommand, 'hand_command', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.hand = self.get_user_input()
        self.get_logger().info(f'HandCommand publisher node started. Emulating {self.hand} arm')

    def get_user_input(self):
        while True:
            user_input = input("Are you emulating the left (l) or right (r) arm? ").strip().lower()
            if user_input in ['l', 'r']:
                return 'left' if user_input == 'l' else 'right'
            print("Invalid input. Please enter 'l' or 'r'.")

    def get_ml_output(self):
        # Placeholder for ML output - replace this with actual model inference later
        return {
            'gesture': 'relaxed_hand',
            'flex_ext': 0.0,
            'pron_supi': 0.0,
            'grip_state': 0.5
        }
        #'deviation': 0.0,

    def timer_callback(self):
        ml_output = self.get_ml_output()

        msg = HandCommand()
        msg.hand = self.hand
        msg.gesture = ml_output['gesture']
        msg.flex_ext = ml_output['flex_ext']
        #msg.deviation = ml_output['deviation']
        msg.pron_supi = ml_output['pron_supi']
        msg.grip_state = ml_output['grip_state']

        self.publisher_.publish(msg)
        self.get_logger().info(f'hand: {msg.hand}, {msg.gesture}, '
                               f'flex_ext={msg.flex_ext}, '
                               f'wristrotation={msg.pron_supi}, grip_state={msg.grip_state}')
                               #deviation={msg.deviation}, 

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
