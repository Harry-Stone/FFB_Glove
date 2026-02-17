#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Float32MultiArray, Int8

class Sim_FFB(Node):

    def __init__(self):
        super().__init__('sim_ffb_node')

        self.declare_parameter('start_enabled', True)
        self.enabled = self.get_parameter('start_enabled').get_parameter_value().bool_value

        self.last_contact_time = [0.0] * 3

        # --- Control rate ---
        self.control_rate_hz = 200.0
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.timer_callback
        )

        # --- Publishers ---
        self.ffb_publisher = self.create_publisher(
            Float32MultiArray,
            'FFB',
            10
        )
        self.haply_forces_publisher = self.create_publisher(
            Float32MultiArray,
            'haply_forces',
            10
        )

        # Enable / Disable
        # `self.enabled` is set from the `start_enabled` parameter above.
        # Do not override it here so the node can start enabled when requested.
        self.create_subscription(
            Int8,
            'ffb/enabled',
            self.enable_callback,
            10
        )

        # Define the long Gazebo topics
        self.contact_topics = {
            'Index': '/world/shapes/model/glove/link/f1l3/sensor/f1l3_contact_sensor/contact',
            'Middle': '/world/shapes/model/glove/link/f2l3/sensor/f2l3_contact_sensor/contact',
            'Thumb': '/world/shapes/model/glove/link/tl3/sensor/tl3_contact_sensor/contact'
        }

        self.subscriptions_list = []

        # Contact state per endpoint index (thumb, f1, f2)
        self.contact_state = [False] * 3
        # Targets and current outputs for ramping
        self.ffb_targets = [0.0] * 3
        self.ffb_currents = [0.0] * 3
        self.max_current = 150.0  # mA to ramp to on contact
        self.ramp_time = 0.3  # seconds to ramp up/down
        # Watchdog for contact messages: if no message for a finger within
        # `contact_timeout`, treat as released. Times are stored per finger.
        # Make watchdog timeout at least as long as the ramp time so
        # intermittent contact messages don't prevent the ramp from completing.
        self.contact_timeout = max(0.5, self.ramp_time)  # seconds
        self.last_contact_time = [0.0] * 3
        
        # Contact-based force feedback to Haply
        self.haply_force_z = 0.0  # Current ramped Z force
        self.haply_force_target_z = 0.0  # Target Z force based on contacts
        self.haply_force_ramp_time = 0.3  # seconds to ramp
        self.haply_force_contributions = {  # Force per finger (1.0 per digit = 3.0 total)
            0: 1.0,   # thumb
            1: 2.0,   # index
            2: 2.0,   # middle
        }

        # Create a subscriber for each finger. Map published finger names to
        # indices matching the endpoint order used for commands (thumb, f1, f2).
        name_to_index = {
            'Thumb': 0,
            'Index': 1,
            'Middle': 2
        }

        # Create subscriptions and bind callbacks with the finger name
        for finger_name, topic in self.contact_topics.items():
            sub = self.create_subscription(
                Contacts,
                topic,
                lambda msg, name=finger_name: self.contact_callback(msg, name_to_index.get(name, 0)),
                10
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f'Subscribed to {finger_name} contacts on {topic}')

        # End-effector frames (order == actuator order)
        self.endpoint_frames = [
            'tl3',   # thumb
            'f1l3',   # finger1
            'f2l3'    # finger2
        ]

        # Precompute dt for ramp step calculation
        self._dt = 1.0 / self.control_rate_hz
        self._ramp_step = (self.max_current * self._dt) / max(self.ramp_time, 1e-6)

        status = 'ENABLED' if self.enabled else 'PAUSED'
        self.get_logger().info(f'FFB node STARTED ({status})')

    def enable_callback(self, msg: Int8):
            """Allows toggling while the node is running"""
            # 1 = Enable, 0 = Disable
            new_state = (msg.data == 1)
            if new_state != self.enabled:
                self.enabled = new_state
                status = "ENABLED" if self.enabled else "PAUSED"
                self.get_logger().info(f"FFB logic is now {status}")

    def contact_callback(self, msg: Contacts, index: int):
        # Record that we received a contact message for this finger
        self.last_contact_time[index] = time.monotonic()

        was = self.contact_state[index]
        now = len(msg.contacts) > 0
        if now and not was:
            self.get_logger().info(f'COLLISION DETECTED: finger idx={index}')
        if not now and was:
            self.get_logger().info(f'CONTACT RELEASED: finger idx={index}')
        self.contact_state[index] = now
        # Set target for ramping: full current on contact, zero on release
        self.ffb_targets[index] = self.max_current if now else 0.0

    def timer_callback(self):
        # Watchdog: clear targets if no contact messages received recently
        now_tm = time.monotonic()
        for i in range(len(self.last_contact_time)):
            if now_tm - self.last_contact_time[i] > self.contact_timeout:
                if self.contact_state[i]:
                    self.get_logger().debug(f'CONTACT TIMEOUT: clearing contact for finger idx={i}')
                self.contact_state[i] = False
                self.ffb_targets[i] = 0.0

        # Compute Haply force target based on current contact states
        # This includes both callback-updated and watchdog-cleared states
        self.haply_force_target_z = sum(
            self.haply_force_contributions[i] for i in range(3) if self.contact_state[i]
        )

        # Pause gate: publish zeros when disabled
        if not self.enabled:
            zero = Float32MultiArray()
            zero.data = [0.0] * len(self.endpoint_frames)
            self.ffb_publisher.publish(zero)
            return

        # Compute FFB Commands (with ramping)
        # Update currents towards targets linearly over ramp_time using precomputed step
        for i in range(len(self.ffb_currents)):
            target = self.ffb_targets[i]
            cur = self.ffb_currents[i]
            if cur < target:
                cur = min(target, cur + self._ramp_step)
            elif cur > target:
                cur = max(target, cur - self._ramp_step)
            self.ffb_currents[i] = cur

        # Prepare command array from current ramped values
        cmd = [self.ffb_currents[i] for i in range(len(self.ffb_currents))]

        # Clamp commands to a safe range and apply any required sign scaling
        for i in range(len(cmd)):
            cmd[i] = max(-100.0, min(200.0, cmd[i]))

        cmd[0] *= -1.0  # Thumb scaling (direction)
        cmd[1] *= 1.0   # Finger1 scaling
        cmd[2] *= -1.0  # Finger2 scaling

        # Publish FFB currents
        msg = Float32MultiArray()
        msg.data = cmd
        self.ffb_publisher.publish(msg)

        # Ramp Haply force Z-axis based on contact
        max_force_z = max(self.haply_force_contributions.values())
        ramp_step_haply = (max_force_z * self._dt) / max(self.haply_force_ramp_time, 1e-6)
        if self.haply_force_z < self.haply_force_target_z:
            self.haply_force_z = min(self.haply_force_target_z, self.haply_force_z + ramp_step_haply)
        elif self.haply_force_z > self.haply_force_target_z:
            self.haply_force_z = max(self.haply_force_target_z, self.haply_force_z - ramp_step_haply)

        # Publish Haply force (x=0, y=0, z=ramped contact force)
        haply_msg = Float32MultiArray()
        haply_msg.data = [0.0, 0.0, self.haply_force_z]
        self.haply_forces_publisher.publish(haply_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Sim_FFB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down')
        # Publish zero currents to ensure motors are released
        try:
            zero = Float32MultiArray()
            zero.data = [0.0] * len(node.endpoint_frames)
            node.ffb_publisher.publish(zero)
            # Give ROS a moment to process the publish
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()