#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32MultiArray, Int8

class MultiFingerEllipticalFFB(Node):

    def __init__(self):
        super().__init__('multi_finger_elliptical_ffb')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Control rate ---
        self.control_rate_hz = 200.0
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.timer_callback
        )

        # --- Publisher ---
        self.ffb_publisher = self.create_publisher(
            Float32MultiArray,
            'FFB',
            10
        )

        # Enable / Disable
        self.enabled = False

        self.create_subscription(
            Int8,
            'multi_finger_elliptical_ffb/enabled',
            self.enable_callback,
            10
        )

        # End-effector frames (order == actuator order)
        self.endpoint_frames = [
            'tl3',   # thumb
            'f1l3',   # finger1
            'f2l3'    # finger2
        ]

        self.get_logger().info('FFB node STARTED (paused)')

    def enable_callback(self, msg: Int8):
        new_state = msg.data == 1
        if new_state != self.enabled:
            self.enabled = new_state
            self.get_logger().info(
                f"FFB node {'ENABLED' if self.enabled else 'PAUSED'}"
            )

    def timer_callback(self):
        # ---------- PAUSE GATE ----------
        if not self.enabled:
            zero = Float32MultiArray()
            zero.data = [0.0] * len(self.endpoint_frames)
            self.ffb_publisher.publish(zero)
            return

        # ---------- Get Transforms ----------
        transforms = []
        try:
            for frame in self.endpoint_frames:
                tf: TransformStamped = self.tf_buffer.lookup_transform(
                    'base',
                    frame,
                    rclpy.time.Time()
                )
                transforms.append(tf)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return
        
        cmd = []

        # ---------- Compute FFB Commands ----------
        for tf in transforms:
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            if z < 0.0:
                cmd.append(100)
            else:
                cmd.append(0)
        
        cmd[0] *= -2.0  # Thumb scaling
        cmd[1] *= 2.0  # Finger1 scaling
        cmd[2] *= -2.0  # Finger2 scaling            

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