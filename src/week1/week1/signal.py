
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class SignalGeneratorNode(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.publisher_signal = self.create_publisher(Float32, '/signal', 10)
        self.publisher_time = self.create_publisher(Float32, '/time', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.time = 0.0

        self.msg_signal = Float32()
        self.msg_time = Float32()



    def timer_callback(self):
        signal_value = math.sin(self.time)
        self.msg_signal.data = signal_value
        self.publisher_signal.publish(self.msg_signal)

        self.msg_time.data = self.time
        self.publisher_time.publish(self.msg_time)

        self.get_logger().info(f"Signal: {signal_value}, Time: {self.time}")
        self.time += 0.1  # Increment time

def main(args=None):
    rclpy.init(args=args)
    signal_generator_node = SignalGeneratorNode()
    rclpy.spin(signal_generator_node)
    signal_generator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()