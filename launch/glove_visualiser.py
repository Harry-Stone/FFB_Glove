from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('glove_description'),
                    'urdf',
                    'hand_description.yaml'
                ])
            ]
        ),

        Node(
            package='state_manager',
            executable='state_manager',
        ),

        Node(
            package='encoder_reader',
            executable='encoder_reader',
        ),

                Node(
            package='haply_connection_manager',
            executable='haply_connection_manager',
        ),

                Node(
            package='imu',
            executable='imu',
        ),

        Node(
            package='rviz2',
            executable='rviz2'
        )

    ])