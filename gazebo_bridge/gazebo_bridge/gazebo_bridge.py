#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import ros_gz_py as gz

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

        # Initialize Gazebo publisher
        self.publisher = gz.transport.Node()
        self.pub_joint = self.publisher.advertise('/model/glove/joint_trajectory', 'gz.msgs.JointTrajectory')

        # Small fixed time for points
        self.time_from_start = 0.01

    def joint_callback(self, msg: JointState):
        # Build Gazebo JointTrajectory message
        traj_msg = gz.msgs.JointTrajectory()

        # Copy joint names
        traj_msg.joint_names.extend(msg.name)

        # Single trajectory point
        point = traj_msg.points.add()
        point.positions.extend(msg.position)
        point.time_from_start.sec = int(self.time_from_start)
        point.time_from_start.nsec = int((self.time_from_start % 1) * 1e9)

        # Publish to Gazebo
        self.pub_joint.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
