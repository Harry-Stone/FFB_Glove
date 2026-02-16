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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformBroadcaster
from ros_gz_interfaces.srv import SetEntityPose


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

def euler_from_quaternion(qx, qy, qz, qw):
    """
    Convert quaternion (x, y, z, w) to roll, pitch, yaw (radians)
    """
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (qw * qy - qz * qx)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw



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

        self.declare_parameter('sim_mode', False)
        self.sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value

        self.get_logger().info(f"Simulation mode: {self.sim_mode}")
        
        if self.sim_mode:
            self.get_logger().info("Simulation mode enabled")
            self.cmd_pub = self.create_publisher(
                JointTrajectory,
                '/glove_controller/joint_trajectory',
                10
            )
            self.sim_timer = self.create_timer(
                0.01,
                self.publish_to_simulation
            )

        self.lock = Lock()

        # Serial finger bend data (initialized to 180.0 - encoder midpoint/neutral position)
        self.serial_data = [180.0] * 12

        # Dynamixel joint positions
        self.dynamixel_positions = [0.0, 0.0, 0.0]

        # IMU orientation (x, y, z, w quaternion) - initialized to identity (no rotation)
        self.imu_xyzw = [0.0, 0.0, 0.0, 1.0]

        self.haply_position = [0.0, 0.0, 1.0]

        if self.sim_mode:
            self.set_pose_client = self.create_client(SetEntityPose, '/world/shapes/set_pose')
            # spin waiting, no blocking forever
            self.get_logger().info("Waiting for /world/shapes/set_pose service...")
            while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Still waiting for set_pose...")


        # =====================
        # Subscriptions
        # =====================
        try:
            self.serial_sub = self.create_subscription(
                Float32MultiArray,
                'serial/data',
                self.serial_data_callback,
                10
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create serial subscription: {e}")
            
        try:
            self.dynamixel_sub = self.create_subscription(
                Float32MultiArray,
                'dynamixel_pos',
                self.dynamixel_pos_callback,
                10
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create dynamixel subscription: {e}")

        try:
            self.imu_sub = self.create_subscription(
                Float32MultiArray,
                'haply_orientation',
                self.imu_callback,
                10
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create IMU subscription: {e}")

        try:
            self.haply_pos_sub = self.create_subscription(
                Float32MultiArray,
                'haply_position',
                self.haply_position_callback,
                10
            )
        except Exception as e:
            self.get_logger().error(f"Failed to create Haply position subscription: {e}")

        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Deadband filter - reduces jitter from encoder noise
        self.prev_positions = [0.0] * 12
        self.threshold = 0.005  # Radians (approx 0.3 degrees)
        
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
        with self.lock:
            qx, qy, qz, qw = [float(v) for v in msg.data]
            roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)
            roll, pitch, yaw = roll, pitch, yaw - math.pi / 2
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            self.imu_xyzw = [qx, qy, qz, qw]

    def update_sim_base_pose(self):
        if not self.set_pose_client.service_is_ready():
            return

        req = SetEntityPose.Request()
        # MUST match the name in your URDF or Spawn command
        req.entity.name = "glove" 

        with self.lock:
            req.pose.position.x = -30 * self.haply_position[0] + 2.5
            req.pose.position.y = -30 * self.haply_position[1] - 4
            req.pose.position.z = 30 * self.haply_position[2]  - 2.5
            
            req.pose.orientation.x = self.imu_xyzw[0]
            req.pose.orientation.y = self.imu_xyzw[1]
            req.pose.orientation.z = self.imu_xyzw[2]
            req.pose.orientation.w = self.imu_xyzw[3]

        self.set_pose_client.call_async(req)


    def publish_base_tf(self):
        base_position = self.haply_position

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id  = 'base'

        t.transform.translation.x = 10 * base_position[0]
        t.transform.translation.y = 10 * base_position[1]
        t.transform.translation.z = 10 * base_position[2] + 2

        with self.lock:
            imu_data = self.imu_xyzw.copy()
        
        self.get_logger().debug(f"Publishing TF - Position: {base_position}, XYZW: {imu_data}")

        t.transform.rotation.x = imu_data[0]
        t.transform.rotation.y = imu_data[1]
        t.transform.rotation.z = imu_data[2]
        t.transform.rotation.w = imu_data[3]

        self.tf_broadcaster.sendTransform(t)


    # =====================
    # Joint State Publisher
    # =====================

    def create_urdf_publisher_node(self):
        self.joint_names = [
            'tl0_joint', 'tl1_joint', 'tl2_joint', 'tl3_joint', 'f1l0_joint', 'f1l1_joint', 'f1l2_joint', 'f1l3_joint', 'f2l0_joint', 'f2l1_joint', 'f2l2_joint', 'f2l3_joint'
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

    def get_current_positions(self):
        positions = [0.0] * 12

        with self.lock:
            sd = self.serial_data.copy()

        # Convert encoder values to radians relative to 180° (neutral position)
        # If encoder is 180 (neutral), current_val is 0.0
        # This represents the deviation from the rest pose
        current_val = [math.radians(v - 180.0) for v in sd]

        try:
            # Thumb - base_positions represents the neutral/rest pose
            positions[0] = self.base_positions[0] - current_val[3]
            positions[1] = self.base_positions[1] + current_val[2]
            positions[2] = self.base_positions[2] - current_val[1]
            positions[3] = self.base_positions[3] + current_val[0]

            # F1
            positions[4] = self.base_positions[4] - current_val[7]
            positions[5] = self.base_positions[5] + current_val[6]
            positions[6] = self.base_positions[6] - current_val[5]
            positions[7] = self.base_positions[7] + current_val[4]

            # F2
            positions[8] = self.base_positions[8] - current_val[11]
            positions[9] = self.base_positions[9] - current_val[10]
            positions[10] = self.base_positions[10] - current_val[9]
            positions[11] = self.base_positions[11] + current_val[8]

            # Wrap angles to +/- pi to align with URDF safety limits
            positions = [math.atan2(math.sin(p), math.cos(p)) for p in positions]
            
            # Apply deadband filter to reduce encoder noise jitter
            positions = [self.apply_deadband(positions[i], self.prev_positions[i]) 
                        for i in range(len(positions))]
            
            # Update previous positions for next iteration
            self.prev_positions = positions.copy()

        except Exception as e:
            self.get_logger().warn(f"Joint update error: {e}")

        return positions

    def apply_deadband(self, new_pos, old_pos):
        """Apply deadband filter - only update if change exceeds threshold"""
        if abs(new_pos - old_pos) < self.threshold:
            return old_pos
        return new_pos
    
    def publish_trajectory(self, positions):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50 ms

        traj.points.append(point)

        self.cmd_pub.publish(traj)

    def publish_states(self):
        positions = self.get_current_positions()

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names
        js.position = positions

        self.pub.publish(js)
        self.publish_base_tf()

        if self.sim_mode:
            self.publish_trajectory(positions)



    def publish_to_simulation(self):
        self.update_sim_base_pose()
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.get_current_positions()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50000000  # 50ms

        traj.points.append(point)

        self.cmd_pub.publish(traj)



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
