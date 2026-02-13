#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStateToGazebo(Node):
    def __init__(self):
        super().__init__('jointstate_to_gazebo')

        # Subscribe to ROS2 joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publisher for the ROS 2 side of the bridge
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/model/glove/joint_trajectory',
            10
        )

    def joint_callback(self, msg: JointState):
        traj_msg = JointTrajectory()
        
        # IMPORTANT: Set the header stamp to the current ROS time
        # This helps the bridge understand the message is fresh
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.joint_names = msg.name

        point = JointTrajectoryPoint()
        point.positions = msg.position
        
        # Use a slightly larger window (50ms - 100ms) 
        # to account for network/bridge latency
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 50_000_000 
        
        traj_msg.points.append(point)
        self.publisher.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
