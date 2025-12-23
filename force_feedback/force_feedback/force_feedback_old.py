#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32MultiArray
import math


class MultiFingerEllipticalFFB(Node):

    def __init__(self):
        super().__init__('multi_finger_elliptical_ffb')

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Control rate ---
        self.control_rate_hz = 200.0
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.timer_callback
        )

        # --- Frames ---
        self.base_frame = 'F1_base'

        # --- Motors (4 total) ---
        self.motor_map = {
            'thumb_z': 0,   # logical ID 11
            'thumb_y': 1,   # logical ID 12
            'finger_1': 2,
            'finger_2': 3,
        }

        # --- Finger links ---
        self.fingers = [
            {'link': 'F1_link4', 'motor': self.motor_map['finger_1']},
            {'link': 'F2_link4', 'motor': self.motor_map['finger_2']},
        ]

        # --- Thumb ---
        self.thumb_link = 'T_link7'

        # --- Publisher ---
        self.ffb_publisher = self.create_publisher(
            Float32MultiArray,
            'FFB',
            10
        )

        # --- Parameters ---
        self.trigger_threshold = float(
            self.declare_parameter(
                'trigger_threshold', -0.045
            ).value
        )

        self.free_current = float(
            self.declare_parameter(
                'free_current', .0
            ).value
        )

        # Ellipse parameters (meters)
        self.ellipse_a = float(
            self.declare_parameter('ellipse_a', 0.020).value
        )  # Y radius

        self.ellipse_b = float(
            self.declare_parameter('ellipse_b', 0.020).value
        )  # Z radius

        self.stiffness = float(
            self.declare_parameter('stiffness', 6000.0).value
        )  # mA per unit penetration

        self.max_current = float(
            self.declare_parameter('max_current', 200.0).value
        )

        self.get_logger().info(
            f'Multi-finger elliptical FFB started '
            f'({self.control_rate_hz:.0f} Hz)'
        )

    def timer_callback(self):
        cmd = [0.0, 0.0, 0.0, 0.0]

        # ---------- Fingers (simple plane) ----------
        for finger in self.fingers:
            try:
                tf: TransformStamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    finger['link'],
                    rclpy.time.Time()
                )

                z = tf.transform.translation.z
                cmd[finger['motor']] = (
                    self.max_current if z < self.trigger_threshold
                    else self.free_current
                )

            except Exception as e:
                self.get_logger().warn(
                    f'TF failed for {finger["link"]}: {e}'
                )

        # ---------- Thumb (elliptical field in Y–Z) ----------
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.thumb_link,
                rclpy.time.Time()
            )

            y = tf.transform.translation.y
            z = tf.transform.translation.z

            # Ellipse distance
            d = math.sqrt(
                (y / self.ellipse_a) ** 2 +
                (z / self.ellipse_b) ** 2
            )

            if d > 1.0:
                penetration = d - 1.0
                force = self.stiffness * penetration

                # Directional scaling
                fy = force * (y / (self.ellipse_a ** 2))
                fz = force * (z / (self.ellipse_b ** 2))

                # Clamp
                fy = max(min(fy, self.max_current), -self.max_current)
                fz = max(min(fz, self.max_current), -self.max_current)

                cmd[self.motor_map['thumb_y']] = abs(fy)
                cmd[self.motor_map['thumb_z']] = abs(fz)
            else:
                cmd[self.motor_map['thumb_y']] = self.free_current
                cmd[self.motor_map['thumb_z']] = self.free_current

        except Exception as e:
            self.get_logger().warn(
                f'TF failed for thumb ({self.thumb_link}): {e}'
            )

        # ---------- Publish ----------
        msg = Float32MultiArray()
        msg.data = cmd
        self.ffb_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiFingerEllipticalFFB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
