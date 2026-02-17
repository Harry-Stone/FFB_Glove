#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import Int8


class Vibration_Contactor(Node):

    def __init__(self):
        super().__init__('vibration_contactor_node')

        self.declare_parameter('start_enabled', True)
        self.enabled = self.get_parameter('start_enabled').get_parameter_value().bool_value

        self.last_contact_time = [0.0] * 3

        # --- Control rate ---
        self.control_rate_hz = 200.0
        self.timer = self.create_timer(
            1.0 / self.control_rate_hz,
            self.timer_callback
        )

        # Enable / Disable
        # `self.enabled` is set from the `start_enabled` parameter above.
        # Do not override it here so the node can start enabled when requested.
        self.create_subscription(
            Int8,
            'vibration_contactor/enabled',
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
        # Watchdog for contact messages: if no message for a finger within
        # `contact_timeout`, treat as released. Times are stored per finger.
        self.contact_timeout = 0.5  # seconds
        self.last_contact_time = [0.0] * 3
        
        # Create a subscriber for each finger. Map published finger names to
        # indices matching the endpoint order used (thumb, f1, f2).
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

        status = 'ENABLED' if self.enabled else 'PAUSED'
        self.get_logger().info(f'Vibration Contactor node STARTED ({status})')

        # --- Audio setup ---
        self.audio_file = '/home/kit-haptics/glove/src/vibration_contactor/wood.mp3'
        self.audio_pid = None
        self.audio_started = False
        self.channel_state = [False, False]  # [Index (left), Middle (right)]
        self._start_audio_loop()

    def _start_audio_loop(self):
        import subprocess
        # Start aplay or ffplay in loop mode if not already running
        if not self.audio_started:
            try:
                # ffplay: -nodisp -autoexit -loop 0
                self.audio_pid = subprocess.Popen([
                    'ffplay', '-nodisp', '-autoexit', '-loop', '0', self.audio_file
                ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.audio_started = True
                self.get_logger().info('Started audio loop for wood.mp3')
            except Exception as e:
                self.get_logger().error(f'Failed to start audio loop: {e}')

    def _set_channel_volume(self, left_on, right_on):
        import subprocess
        # Use amixer to set left/right channel volume
        left_vol = '100%' if left_on else '0%'
        right_vol = '100%' if right_on else '0%'
        try:
            subprocess.run([
                'amixer', 'set', 'Master', left_vol, 'unmute'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run([
                'amixer', 'set', 'Master', right_vol, 'unmute'
            ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().error(f'Failed to set channel volume: {e}')

    def destroy_node(self):
        import subprocess
        # Stop audio loop
        if self.audio_pid:
            try:
                self.audio_pid.terminate()
                self.audio_pid.wait(timeout=2)
                self.get_logger().info('Stopped audio loop')
            except Exception as e:
                self.get_logger().error(f'Failed to stop audio loop: {e}')
        super().destroy_node()
    def enable_callback(self, msg: Int8):
            """Allows toggling while the node is running"""
            # 1 = Enable, 0 = Disable
            new_state = (msg.data == 1)
            if new_state != self.enabled:
                self.enabled = new_state
                status = "ENABLED" if self.enabled else "PAUSED"
                self.get_logger().info(f"Vibration Contactor logic is now {status}")

    def contact_callback(self, msg: Contacts, index: int):
        # Record that we received a contact message for this finger
        self.last_contact_time[index] = time.monotonic()

        was = self.contact_state[index]
        now = len(msg.contacts) > 0
        if now and not was:
            self.get_logger().info(f'COLLISION DETECTED: finger idx={index} at {self.last_contact_time[index]:.4f}')
        if not now and was:
            self.get_logger().info(f'CONTACT RELEASED: finger idx={index} at {self.last_contact_time[index]:.4f}')
        self.contact_state[index] = now

    def timer_callback(self):
        # Pause gate: skip processing when disabled
        if not self.enabled:
            # Mute both channels if disabled
            self._set_channel_volume(False, False)
            return

        # Watchdog: clear contacts if no contact messages received recently
        now_tm = time.monotonic()
        for i in range(len(self.last_contact_time)):
            if now_tm - self.last_contact_time[i] > self.contact_timeout:
                if self.contact_state[i]:
                    self.get_logger().info(f'CONTACT TIMEOUT: clearing contact for finger idx={i} at {now_tm:.4f}')
                self.contact_state[i] = False

        # Log current contact state for each finger
        for i in range(len(self.contact_state)):
            state_str = "IN CONTACT" if self.contact_state[i] else "RELEASED"
            self.get_logger().debug(f'Finger {i}: {state_str} | Last msg: {now_tm - self.last_contact_time[i]:.4f}s ago')

        # Index finger (left channel), Middle finger (right channel)
        left_on = self.contact_state[1]
        right_on = self.contact_state[2]
        # Only update if state changes
        if [left_on, right_on] != self.channel_state:
            self._set_channel_volume(left_on, right_on)
            self.channel_state = [left_on, right_on]


def main(args=None):
    rclpy.init(args=args)
    node = Vibration_Contactor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
