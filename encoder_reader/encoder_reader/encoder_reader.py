import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import json
import os
from ament_index_python.packages import get_package_share_directory

config_package_name = 'shared_config' 
config_file_name = 'glove_settings.json'
share_directory = get_package_share_directory(config_package_name)
config_file_path = os.path.join(share_directory, 'config', config_file_name)

try:
    with open(config_file_path, 'r') as f:
        config = json.load(f)
except FileNotFoundError:
    print(f"Error: config.json not found at {config_file_path}")
    exit()


class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')

        # Parameters
        self.declare_parameter('port', config['encoder_serial']['port'])
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
        self.pub = self.create_publisher(Float32MultiArray, 'serial/data', 10)

        # Timer (read at 50 Hz)
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    msg = Float32MultiArray()
                    #spit on commas and publish as list of floats
                    self.get_logger().info(f"Read line: {line}")
                    if(line[0] == 'b' and line[-2] == 'e' and line.count(',') == 12):
                        msg.data  = [float(x.strip()) for x in line[2:-4].split(',')]
                        self.pub.publish(msg)
                    else:
                        self.get_logger().warn(f"Malformed data: {line}")
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
