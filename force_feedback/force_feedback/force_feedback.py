#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32MultiArray

def distance3D(x1, y1, z1, x2, y2, z2):
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2) ** 0.5

k = 1.5
contact_current = 60.0
max_current = 200.0
dist_threshold = 0.04

xo = 0.07
yo = 0.0
zo = -0.08

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

        self.get_logger().info('Bare-bones FFB node started')

    def timer_callback(self):
        cmd = [0.0, 0.0, 0.0, 0.0]

        # ---------- Fingers: lookup transforms and map to motors ----------
        for finger in self.fingers:
            try:
                tf: TransformStamped = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    finger['link'],
                    rclpy.time.Time()
                )
                #F1_link4 0.12610624242093774 0.0 0.09608075736578983 0.0 -0.03352964962289691
                #F1_link4 0.11204354082015804 0.0 0.07826913089277233 0.0 -0.04473709312181075
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z
                dist = distance3D(xo, yo, zo, x, 0 , z) #this is to test finger 2v3 fix after confiming
                if dist < dist_threshold:
                    cmd[finger['motor']] = -1*(contact_current+float(k/dist))
                else:
                    cmd[finger['motor']] = 0.0
            #    if (finger['link'] == 'F1_link4'):
            #        print(finger['link'], dist, cmd[finger['motor']], x, y, z)
            except Exception as e:
                self.get_logger().warn(f'TF failed for {finger["link"]}: {e}')

                # ---------- Thumb: virtual spherical FFB ----------
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.thumb_link,
                rclpy.time.Time()
            )

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            # Displacement from virtual sphere center
            dx = x - xo
            dy = y - yo
            dz = z - zo

            dist = (dx*dx + dy*dy + dz*dz) ** 0.5

            if dist < dist_threshold and dist > 1e-6:
                # Penetration depth (how far inside the sphere)
                penetration = dist_threshold - dist

                # Radial spring magnitude
                effort = k * penetration / dist_threshold

                # Project radial effort onto Y and Z axes
                fy = effort * (dy / dist)
                fz = effort * (dz / dist)

                # Apply motor polarity (Y is flipped)
                cmd[self.motor_map['thumb_y']] = -fy
                cmd[self.motor_map['thumb_z']] =  fz
            else:
                cmd[self.motor_map['thumb_y']] = 0.0
                cmd[self.motor_map['thumb_z']] = 0.0

            # Clamp torques
            for i in range(len(cmd)):
                if cmd[i] > max_current:
                    cmd[i] = max_current
                elif cmd[i] < -max_current:
                    cmd[i] = -max_current

        except Exception as e:
            self.get_logger().warn(f'TF failed for thumb ({self.thumb_link}): {e}')


        # ---------- Publish raw lookup values to FFB topic ----------
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
