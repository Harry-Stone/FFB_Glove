#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32MultiArray


class LinkCartesianPosition(Node):

    def __init__(self):
        super().__init__('link_cartesian_position')

        # --- TF Buffer & Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- High-rate Timer (200 Hz) ---
        self.control_rate_hz = 200.0
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.timer_callback
        )

        # --- Frames ---
        self.base_frame = 'F1_base'
        self.target_frame = 'F1_link4'

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

        self.motor_index = int(
            self.declare_parameter(
                'motor_index', 2
            ).value
        )

        self.current_on_limit = float(
            self.declare_parameter(
                'current_on_limit', 200.0
            ).value
        )

        self.get_logger().info(
            f'Link Cartesian Position node started '
            f'({self.control_rate_hz:.0f} Hz)'
        )

    def timer_callback(self):
        try:
            # --- Lookup TF transform ---
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time()
            )

            translation = transform.transform.translation

            # --- Trigger logic ---
            axis = self.trigger_axis.lower()

            if axis == 'x':
                triggered = translation.x < self.trigger_threshold
            elif axis == 'y':
                triggered = translation.y < self.trigger_threshold
            elif axis == 'z':
                triggered = translation.z < self.trigger_threshold
            elif axis == 'any':
                triggered = (
                    translation.x < self.trigger_threshold or
                    translation.y < self.trigger_threshold or
                    translation.z < self.trigger_threshold
                )
            else:
                self.get_logger().warn(
                    f'Unknown trigger_axis "{self.trigger_axis}"'
                )
                return

            # --- Continuous current command ---
            current_cmd = (
                self.current_on_limit if triggered else 20.0
            )

            # --- Build and publish message ---
            if 0 <= self.motor_index < 4:
                cmd = [0.0, 0.0, 0.0, 0.0]
                cmd[self.motor_index] = float(current_cmd)

                msg = Float32MultiArray()
                msg.data = cmd

                self.ffb_publisher.publish(msg)

            else:
                self.get_logger().warn(
                    f'Invalid motor_index: {self.motor_index}'
                )

        except Exception as e:
            # Throttle TF warnings to avoid console spam
            self.get_logger().warn(
                f'TF lookup failed: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LinkCartesianPosition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
