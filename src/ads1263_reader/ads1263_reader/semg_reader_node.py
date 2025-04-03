import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from ads1263_reader.ADS1263 import ADS1263
from ads1263_reader.config import RaspberryPi

class SEMGReaderNode(Node):
    def __init__(self):
        super().__init__('semg_reader_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'semg_data', 10)
        timer_period = 0.001  # 1000 Hz sampling (1 ms)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize ADC
        self.adc = ADS1263.ADS1263()
        if self.adc.ADS1263_init_ADC1('ADS1263_GAIN_1', 'ADS1263_10SPS') == -1:
            self.get_logger().error('Failed to initialize ADS1263')
            exit()

        self.adc.ADS1263_SetMode(1)  # Single-ended
        self.channel_list = [0, 1, 2, 3]  # Channels for 4 sEMG electrodes

        self.get_logger().info('sEMG Reader Node started and ADC initialized.')

    def timer_callback(self):
        # Read values from 4 sEMG channels
        try:
            adc_values = self.adc.ADS1263_GetAll()
            selected_values = [adc_values[i] for i in self.channel_list]
            # Convert to microvolts (assumes default reference voltage of 5.0V)
            converted = [(val * 5.0 / 0x7fffffff) * 1e6 for val in selected_values]

            msg = Float32MultiArray()
            msg.data = converted
            self.publisher_.publish(msg)
            self.get_logger().debug(f'Published sEMG data: {converted}')

        except Exception as e:
            self.get_logger().error(f'ADC read failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SEMGReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.adc.ADS1263_Exit()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
