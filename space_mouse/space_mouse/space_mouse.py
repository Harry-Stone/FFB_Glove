import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pyspacemouse
import numpy as np

class SpaceMouseNode(Node):
    def __init__(self):
        super().__init__('space_mouse')
        self.get_logger().info("Space Mouse Node initialized")

        # Declare sensitivity parameter
        self.declare_parameter('sensitivity', 1.0)
        self.sensitivity = self.get_parameter('sensitivity').value

        # Initialize pose
        self.position = np.zeros(3)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion w, x, y, z
        self.last_time = self.get_clock().now()

        # Publishers
        self.position_publisher = self.create_publisher(
            Float32MultiArray, 'haply_position', 10)
        self.velocity_publisher = self.create_publisher(
            Float32MultiArray, 'haply_velocity', 10)
        self.orientation_publisher = self.create_publisher(
            Float32MultiArray, 'haply_orientation', 10)
        self.buttons_publisher = self.create_publisher(
            Float32MultiArray, 'haply_buttons', 10)

        # Open space mouse device
        try:
            self.device = pyspacemouse.open()
            self.get_logger().info("Space mouse opened successfully")
        except Exception as e:
            self.get_logger().error(f"Could not open space mouse: {e}")
            self.device = None

        # Timer for updates (100 Hz)
        self.update_timer = self.create_timer(0.01, self.update)

    def quat_mult(self, q1, q2):
        """Quaternion multiplication"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return np.array([w, x, y, z])

    def update(self):
        if self.device is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        try:
            state = self.device.read()

            # Remap axes: X and Y swapped, roll/pitch/yaw corrected
            # Position: x = y, y = x, z = z
            # Orientation: roll = pitch, pitch = yaw, yaw = roll  

            # Update position (integrate velocity)
            vel = np.array([-state.y, state.x, state.z]) * self.sensitivity
            self.position += vel * dt

            # Update orientation (integrate angular velocity)
            omega = np.array([-2*state.yaw, 2*state.roll, -state.pitch]) * self.sensitivity
            omega_quat = np.array([0.0, omega[0], omega[1], omega[2]])
            q_dot = 0.5 * self.quat_mult(self.orientation, omega_quat)
            self.orientation += q_dot * dt
            self.orientation /= np.linalg.norm(self.orientation)  # normalize

            # Publish position
            pos_msg = Float32MultiArray()
            pos_msg.data = self.position.tolist()
            self.position_publisher.publish(pos_msg)

            # Publish velocity
            vel_msg = Float32MultiArray()
            vel_msg.data = vel.tolist()
            self.velocity_publisher.publish(vel_msg)

            # Publish orientation
            ori_msg = Float32MultiArray()
            ori_msg.data = self.orientation.tolist()
            self.orientation_publisher.publish(ori_msg)

            # Publish buttons
            btn_msg = Float32MultiArray()
            btn_msg.data = [float(b) for b in state.buttons]
            self.buttons_publisher.publish(btn_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading space mouse: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()