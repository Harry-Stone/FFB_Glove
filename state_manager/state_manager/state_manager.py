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


upperLimit = [0, 0, 0, 0]
lowerLimit = [0, 0, 0, 0]
zeroPoint  = [0, 0, 0, 0]

upperLimit[0] = dynamixelSettings["thumb"]["d1"]["upperLimit"]
lowerLimit[0] = dynamixelSettings["thumb"]["d1"]["lowerLimit"]
zeroPoint[0]  = dynamixelSettings["thumb"]["d1"]["zeroPoint"]

upperLimit[1] = dynamixelSettings["thumb"]["d2"]["upperLimit"]
lowerLimit[1] = dynamixelSettings["thumb"]["d2"]["lowerLimit"]
zeroPoint[1]  = dynamixelSettings["thumb"]["d2"]["zeroPoint"]

upperLimit[2] = dynamixelSettings["finger1"]["d1"]["upperLimit"]
lowerLimit[2] = dynamixelSettings["finger1"]["d1"]["lowerLimit"]
zeroPoint[2]  = dynamixelSettings["finger1"]["d1"]["zeroPoint"]

upperLimit[3] = dynamixelSettings["finger2"]["d1"]["upperLimit"]
lowerLimit[3] = dynamixelSettings["finger2"]["d1"]["lowerLimit"]
zeroPoint[3]  = dynamixelSettings["finger2"]["d1"]["zeroPoint"]


# =========================
# State Manager Node
# =========================

class StateManager(Node):
    def __init__(self):
        super().__init__('state_manager')

        self.lock = Lock()

        # Serial finger bend data
        self.serial_data = [180.0] * 10

        # Dynamixel joint positions
        self.dynamixel_positions = [0.0, 0.0, 0.0, 0.0]

        # IMU orientation (roll, pitch, yaw) in radians
        self.imu_rpy = [0.0, 0.0, 0.0]

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
            '/serial/imu',
            self.imu_callback,
            10
        )

        # =====================
        # TF Broadcaster
        # =====================

        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("StateManager initialized")


    # =====================
    # Callbacks
    # =====================

    def serial_data_callback(self, msg):
        self.serial_data = list(msg.data)

    def dynamixel_pos_callback(self, msg):
        try:
            raw = list(msg.data)
            if len(raw) != 4:
                return
            with self.lock:
                self.dynamixel_positions = [float(v) for v in raw]
        except Exception as e:
            self.get_logger().warn(f"Dynamixel data error: {e}")

    def imu_callback(self, msg):
        if len(msg.data) != 3:
            return

        # roll, pitch, yaw already in radians
        self.imu_rpy = [float(v) for v in msg.data]
        self.publish_base_tf()


    # =====================
    # TF publishing
    # =====================

    def publish_base_tf(self):
        roll, pitch, yaw = self.imu_rpy

        # Adjust axes here if needed
        # roll  = -roll
        # yaw   = -yaw

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'F1_base'



        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


    # =====================
    # Joint State Publisher
    # =====================

    def create_urdf_publisher_node(self):
        self.joint_names = [
            'F1_joint0', 'F1_joint1', 'F1_joint2', 'F1_joint3', 'F1_joint4',
            'F2_joint0', 'F2_joint1', 'F2_joint2', 'F2_joint3', 'F2_joint4',
            'T_joint0', 'T_joint1', 'T_joint2', 'T_joint3',
            'T_joint4', 'T_joint5', 'T_joint6', 'T_joint7'
        ]

        self.base_positions = [
            0.0, -0.8086398227678077, 1.5708, 1.0967606203126732, 0.05745128772331998,
            -0.15041057879455066, -0.7393525339285175, 1.5708, 1.0274733314733828, 0,
            1.5708, 1.5708, -1.5708, 0.19602586540190026,
            -0.7047088895088723, 1.5708, 1.131404264732318, 0.8
        ]

        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.01, self.publish_joint_states)


    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [float(v) for v in self.base_positions]

        with self.lock:
            dp = self.dynamixel_positions.copy()
            sd = self.serial_data.copy()

        try:
            msg.position[13] = (dp[0] - zeroPoint[0]) * 0.00153398078
            msg.position[14] = (dp[1] - zeroPoint[1]) * 0.00153398078
            msg.position[1]  = (dp[2] - zeroPoint[2]) * 0.00153398078
            msg.position[6]  = (dp[3] - zeroPoint[3]) * 0.00153398078

            # Index
            msg.position[7] = math.radians(180) - math.radians(sd[1])
            msg.position[8] = math.radians(180) + math.radians(sd[2])

            # Pointer
            msg.position[2] = math.radians(180) - math.radians(sd[6])
            msg.position[3] = math.radians(180) + math.radians(sd[7])

            # Thumb
            msg.position[15] = math.radians(180) - math.radians(sd[3])
            msg.position[16] = math.radians(180) + math.radians(sd[8])

            msg.position[17] = math.radians(180)

        except Exception as e:
            self.get_logger().warn(f"Joint update error: {e}")

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
