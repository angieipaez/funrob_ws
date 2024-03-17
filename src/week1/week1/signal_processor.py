
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import numpy as np  # Add this import for NumPy

class ProcessNode(Node):
    def __init__(self):
        super().__init__('process')
        self.subscription_signal = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.subscription_time = self.create_subscription(Float32, '/time', self.time_callback, 10)

        self.publisher_proc_signal = self.create_publisher(Float32, '/proc_signal', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Initialize processed_signal at the class level
        self.processed_signal = Float32()

    def signal_callback(self, msg):
        self.get_logger().info('Processed Signal: {:.2f}'.format(self.processed_signal.data))

    def time_callback(self, msg):
        self.processed_signal.data = 4 * (np.sin(msg.data) * np.cos(np.pi/4) + np.cos(msg.data) * np.sin(np.pi/4))
        self.processed_signal.data += 5
        self.processed_signal.data /= float(2.0)

    def timer_callback(self):
        self.publisher_proc_signal.publish(self.processed_signal)
        self.get_logger().info('Processed Signal: {:.2f}'.format(self.processed_signal.data))

def main(args=None):
    rclpy.init(args=args)
    process_node = ProcessNode()
    rclpy.spin(process_node)
    process_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()