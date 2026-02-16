import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from threading import Lock
from ament_index_python.packages import get_package_share_directory
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw (radians) to quaternion (x, y, z, w)
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qx, qy, qz, qw)



# =========================
# Load configuration
# =========================

config_package_name = 'shared_config'
config_file_name = 'glove_settings.json'
share_directory = get_package_share_directory(config_package_name)
config_file_path = os.path.join(share_directory, 'config', config_file_name)

try:
    with open(config_file_path, 'r') as f:
        dynamixelSettings = json.load(f)
except FileNotFoundError:
    print(f"Error: config.json not found at {config_file_path}")
    exit()


upperLimit = [0, 0, 0]
lowerLimit = [0, 0, 0]
zeroPoint  = [0, 0, 0]

upperLimit[0] = dynamixelSettings["thumb"]["d1"]["upperLimit"]
lowerLimit[0] = dynamixelSettings["thumb"]["d1"]["lowerLimit"]
zeroPoint[0]  = dynamixelSettings["thumb"]["d1"]["zeroPoint"]

upperLimit[1] = dynamixelSettings["finger1"]["d1"]["upperLimit"]
lowerLimit[1] = dynamixelSettings["finger1"]["d1"]["lowerLimit"]
zeroPoint[1]  = dynamixelSettings["finger1"]["d1"]["zeroPoint"]

upperLimit[2] = dynamixelSettings["finger2"]["d1"]["upperLimit"]
lowerLimit[2] = dynamixelSettings["finger2"]["d1"]["lowerLimit"]
zeroPoint[2]  = dynamixelSettings["finger2"]["d1"]["zeroPoint"]


# =========================
# State Manager Node
# =========================

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.lock = Lock()

        # Serial finger bend data
        self.serial_data = [180.0] * 12

        # Dynamixel joint positions
        self.dynamixel_positions = [0.0, 0.0, 0.0]

        # IMU orientation (roll, pitch, yaw) in radians
        self.imu_xyzw = [0.0, 0.0, 0.0, 0.0]

        self.haply_position = [0.0, 0.0, 0.0]

        # =====================
        # Subscriptions
        # =====================

        self.serial_sub = self.create_subscription(
            Float32MultiArray,
            'serial/data',
            self.serial_data_callback,
            10
        )

        self.dynamixel_sub = self.create_subscription(
            Float32MultiArray,
            'dynamixel_pos',
            self.dynamixel_pos_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Float32MultiArray,
            'haply_orientation',
            self.imu_callback,
            10
        )

        self.haply_pos_sub = self.create_subscription(
            Float32MultiArray,
            'haply_position',
            self.haply_position_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("StateManager initialized")

    def serial_data_callback(self, msg):
        self.serial_data = list(msg.data)

    def haply_position_callback(self, msg):
        try:
            raw = list(msg.data)
            if len(raw) != 3:
                return
            with self.lock:
                self.haply_position = [float(v) for v in raw]
        except Exception as e:
            self.get_logger().warn(f"Haply position data error: {e}")

    def dynamixel_pos_callback(self, msg):
        try:
            raw = list(msg.data)
            if len(raw) != 3:
                return
            with self.lock:
                self.dynamixel_positions = [float(v) for v in raw]
        except Exception as e:
            self.get_logger().warn(f"Dynamixel data error: {e}")

    def imu_callback(self, msg):
        if len(msg.data) != 4:
            return

        # x, y, z, w
        self.imu_xyzw = [float(v) for v in msg.data]

    # =====================
    # TF publishing
    # =====================

    def publish_base_tf(self):
        x, y, z, w = self.imu_xyzw
        base_position = self.haply_position

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'base'

        t.transform.translation.x = 10 * base_position[0]
        t.transform.translation.y = 10 * base_position[1]
        t.transform.translation.z = 10 * base_position[2]

        self.get_logger().debug(f"Publishing TF - Position: {base_position}, XYZW: {self.imu_xyzw}")

        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w

        self.tf_broadcaster.sendTransform(t)


    # =====================
    # Joint State Publisher
    # =====================

    def create_urdf_publisher_node(self):
        self.joint_names = [
            'tl0', 'tl1', 'tl2', 'tl3', 'f1l0', 'f1l1', 'f1l2', 'f1l3', 'f2l0', 'f2l1', 'f2l2', 'f2l3'
        ]

        self.base_positions = [
            math.radians(177.0),  
            math.radians(188.0),
            math.radians(174.0),
            math.radians(177.0),
            math.radians(165.0),
            math.radians(202.0),
            math.radians(180.0),
            math.radians(182.0),
            math.radians(133.0),
            math.radians(159.0),
            math.radians(170.0),
            math.radians(182.0)]
        
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_states)


    def publish_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in self.base_positions]

        with self.lock:
            sd = self.serial_data.copy()

        try:
            #Thumb
            msg.position[0] = self.base_positions[0] - math.radians(sd[3])
            msg.position[1] = self.base_positions[1] + math.radians(sd[2])
            msg.position[2] = self.base_positions[2] - math.radians(sd[1])
            msg.position[3] = self.base_positions[3] + math.radians(sd[0])

            #F1
            msg.position[4] = self.base_positions[4] - math.radians(sd[7])
            msg.position[5] = self.base_positions[5] + math.radians(sd[6])
            msg.position[6] = self.base_positions[6] - math.radians(sd[5])
            msg.position[7] = self.base_positions[7] + math.radians(sd[4])

            #F2
            msg.position[8] = self.base_positions[8] - math.radians(sd[11])
            msg.position[9] = self.base_positions[9] - math.radians(sd[10])
            msg.position[10] = self.base_positions[10] - math.radians(sd[9])
            msg.position[11] = self.base_positions[11] + math.radians(sd[8])

        except Exception as e:
            self.get_logger().warn(f"Joint update error: {e}")

        try:
            self.publish_base_tf()
        except Exception as e:
            self.get_logger().warn(f"TF publish error: {e}")

        self.pub.publish(msg)


# =========================
# Main
# =========================

def main(args=None):
    rclpy.init(args=args)
    node = StateManager()
    node.create_urdf_publisher_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
