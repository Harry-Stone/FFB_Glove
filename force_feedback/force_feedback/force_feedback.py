#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32MultiArray


class DualFingerCartesianFFB(Node):

    def __init__(self):
        super().__init__('dual_finger_cartesian_ffb')

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

        self.fingers = [
            {
                'name': 'F1',
                'link': 'F1_link4',
                'motor_index': 2,
                'current_on': 200.0,
            },
            {
                'name': 'F2',
                'link': 'F2_link4',
                'motor_index': 3,
                'current_on': 200.0,
            }
        ]

        # --- Publisher ---
        self.ffb_publisher = self.create_publisher(
            Float32MultiArray,
            'FFB',
            10
        )

        # --- Parameters ---
        self.trigger_axis = self.declare_parameter(
            'trigger_axis', 'z'
        ).value

        self.trigger_threshold = float(
            self.declare_parameter(
                'trigger_threshold', -0.045
            ).value
        )

        self.free_current = float(
            self.declare_parameter(
                'free_current', 20.0
            ).value
        )

        self.get_logger().info(
            f'Dual Finger Cartesian FFB started '
            f'({self.control_rate_hz:.0f} Hz)'
        )

    def finger_triggered(self, translation) -> bool:
        axis = self.trigger_axis.lower()

        if axis == 'x':
            return translation.x < self.trigger_threshold
        elif axis == 'y':
            return translation.y < self.trigger_threshold
        elif axis == 'z':
            return translation.z < self.trigger_threshold
        elif axis == 'any':
            return (
                translation.x < self.trigger_threshold or
                translation.y < self.trigger_threshold or
                translation.z < self.trigger_threshold
            )
        else:
            self.get_logger().warn(
                f'Unknown trigger_axis "{self.trigger_axis}"'
            )
            return False

    def timer_callback(self):
        # --- Initialize command (4 motors assumed) ---
        cmd = [0.0, 0.0, 0.0, 0.0]

        for finger in self.fingers:
            try:
                transform: TransformStamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    finger['link'],
                    rclpy.time.Time()
                )

                translation = transform.transform.translation
                triggered = self.finger_triggered(translation)

                current = (
                    finger['current_on'] if triggered else self.free_current
                )

                idx = finger['motor_index']
                if 0 <= idx < len(cmd):
                    cmd[idx] = float(current)

            except Exception as e:
                self.get_logger().warn(
                    f'TF lookup failed for {finger["link"]}: {e}'
                )

        # --- Publish combined command ---
        msg = Float32MultiArray()
        msg.data = cmd
        self.ffb_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DualFingerCartesianFFB()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
