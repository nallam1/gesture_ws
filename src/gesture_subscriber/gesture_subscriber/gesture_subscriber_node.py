import rclpy
from rclpy.node import Node
from gesture_msgs.msg import HandCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64

class GestureTeleop(Node):
    def __init__(self):
        super().__init__('gesture_teleop')

        # Initialize current joint positions
        self.current_joint_positions = {}

        # Subscriber to measured joint states
        self.state_sub = self.create_subscription(
            JointState,
            '/franka/measured_js',
            self.state_callback,
            10
        )

        # Subscriber to gesture commands
        self.gesture_sub = self.create_subscription(
            HandCommand,
            'hand_command',
            self.listener_callback,
            10
        )

        # Publisher for joint positions
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)

        # Publisher for gripper control (assuming Float64 message)
        self.gripper_pub = self.create_publisher(Float64, '/franka_gripper/width', 10)

        self.get_logger().info('Gesture teleoperation node started.')

    def state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received: {msg.hand}, {msg.gesture}, '
            f'flex_ext={msg.flex_ext}, '
            f'pron_supi={msg.pron_supi}, grip_state={msg.grip_state}'
        )

        # Get current joint positions with defaults if not available yet
        current_joint6 = self.current_joint_positions.get("panda_joint6", 0.0)
        current_joint7 = self.current_joint_positions.get("panda_joint7", 0.0)

        # Convert gesture values from degrees to radians
        delta_joint6 = msg.flex_ext * 3.1415 / 180.0
        delta_joint7 = msg.pron_supi * 3.1415 / 180.0

        # Apply blending
        joint6_pos = current_joint6 + delta_joint6
        joint7_pos = current_joint7 + delta_joint7

        # Publish blended joint commands
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ["panda_joint6", "panda_joint7"]
        joint_msg.position = [joint6_pos, joint7_pos]
        self.joint_pub.publish(joint_msg)

        # Map grip_state (0.0 to 1.0) to gripper width in meters (0.0 to 0.08 m)
        gripper_width = msg.grip_state * 0.08
        grip_msg = Float64()
        grip_msg.data = gripper_width
        self.gripper_pub.publish(grip_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GestureTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

