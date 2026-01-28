import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct
import json
import os
from ament_index_python.packages import get_package_share_directory
import time


# ---------------- Config loading ----------------

config_package_name = 'shared_config'
config_file_name = 'glove_settings.json'
share_directory = get_package_share_directory(config_package_name)
config_file_path = os.path.join(share_directory, 'config', config_file_name)

try:
    with open(config_file_path, 'r') as f:
        config = json.load(f)
except FileNotFoundError:
    raise RuntimeError(f"Config not found: {config_file_path}")


# ---------------- COBS decode ----------------

def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    length = len(data)

    while i < length:
        code = data[i]
        i += 1
        for _ in range(code - 1):
            if i >= length:
                return bytes(out)
            out.append(data[i])
            i += 1
        if code < 0xFF and i < length:
            out.append(0x00)

    return bytes(out)


# ---------------- ROS node ----------------

class EncoderReaderNode(Node):
    def __init__(self):
        super().__init__('serial_reader')

        # Serial parameters
        self.declare_parameter('port', config['encoder_serial']['port'])
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        # Encoder config
        enc_cfg = config['encoders']
        self.cpr = float(enc_cfg.get('counts_per_rev', 4096))
        self.offsets = enc_cfg.get('offsets', [])

        if len(self.offsets) != 12:
            raise RuntimeError(
                f"Expected 12 encoder offsets, got {len(self.offsets)}"
            )

        self.units = enc_cfg.get('unit', 'deg')  # 'deg' or 'rad'

        # Serial open
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info(f"Opened serial port {port} @ {baud}")
        except Exception as e:
            raise RuntimeError(f"Failed to open serial port: {e}")

        # ROS publisher
        self.pub = self.create_publisher(
            Float32MultiArray,
            'serial/data',
            10
        )

        self.timer = self.create_timer(0.01, self.read_serial)
        self.rx_buf = bytearray()

        self.deg_per_count = 360.0 / self.cpr
        self.rad_per_count = 2.0 * 3.141592653589793 / self.cpr

        self.last_log_time = 0.0
        self.log_period = 5.0  # seconds


    def read_serial(self):
        try:
            data = self.ser.read(256)
            if not data:
                return

            for b in data:
                if b == 0x00:
                    if self.rx_buf:
                        self.process_frame(bytes(self.rx_buf))
                        self.rx_buf.clear()
                else:
                    self.rx_buf.append(b)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def process_frame(self, frame: bytes):
        decoded = cobs_decode(frame)

        if len(decoded) != 24:
            self.get_logger().warn(
                f"Bad frame length: {len(decoded)} bytes"
            )
            return

        try:
            counts = struct.unpack('<12H', decoded)
        except struct.error:
            self.get_logger().warn("Unpack failed")
            return

        data = []
        for i, c in enumerate(counts):
            if self.units == 'rad':
                angle = c * self.rad_per_count
                angle += self.offsets[i] * (3.141592653589793 / 180.0)
            else:  # degrees
                angle = c * self.deg_per_count + self.offsets[i]

            data.append(angle)

        # Publish
        msg = Float32MultiArray()
        msg.data = data
        self.pub.publish(msg)

        # Rate-limited INFO log
        now = time.monotonic()
        if now - self.last_log_time >= self.log_period:
            self.get_logger().info(
                "Encoder angles: " +
                ", ".join(f"{v:.2f}" for v in data)
            )
            self.last_log_time = now



# ---------------- Main ----------------

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
