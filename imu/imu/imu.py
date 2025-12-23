#!/usr/bin/env python3

import math
import struct
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class SerialIMUNode(Node):
    def __init__(self):
        super().__init__('serial_imu_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('port', '/dev/ttyUSB2')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('rate', 20.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        rate = self.get_parameter('rate').value

        # -----------------------------
        # Serial
        # -----------------------------
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.get_logger().info(f'Opened IMU on {port} @ {baud}')

        # -----------------------------
        # Publisher
        # -----------------------------
        self.pub = self.create_publisher(
            Float32MultiArray,
            '/serial/imu',
            10
        )

        # -----------------------------
        # Timer
        # -----------------------------
        self.timer = self.create_timer(
            1.0 / rate,
            self.read_and_publish
        )

    def read_and_publish(self):
        try:
            latest = None

            # Drain the serial buffer
            while self.ser.in_waiting >= 11:
                # Sync to header
                if self.ser.read(1) != b'\x55':
                    continue

                pid = self.ser.read(1)
                if not pid:
                    break

                if pid[0] != 0x53:
                    self.ser.read(9)
                    continue

                body = self.ser.read(9)
                if len(body) != 9:
                    break

                data = body[:8]
                checksum = body[8]

                if ((0x55 + 0x53 + sum(data)) & 0xFF) != checksum:
                    continue

                raw_roll, raw_pitch, raw_yaw, _ = struct.unpack('<hhhh', data)

                imu_roll  = math.radians(raw_roll  * 180.0 / 32768.0)
                imu_pitch = math.radians(raw_pitch * 180.0 / 32768.0)
                imu_yaw   = math.radians(raw_yaw   * 180.0 / 32768.0)

                # Axis remapping: IMU → ROS body frame
                roll  =  imu_pitch
                pitch =  -imu_roll
                yaw   =  imu_yaw

                yaw += math.pi / 2  # Adjust yaw zero point

                # overwrite — we only care about the newest
                latest = [roll, pitch, yaw]

            if latest is not None:
                msg = Float32MultiArray()
                msg.data = latest
                self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SerialIMUNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
