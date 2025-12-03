import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')

        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise

        # Publisher
        self.pub = self.create_publisher(String, 'serial/data', 10)

        # Timer (read at 50 Hz)
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    msg = String()
                    msg.data = line
                    self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
