from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            'cat ',
            PathJoinSubstitution([
                FindPackageShare('glove_description'),
                'urdf',
                'glove.urdf'
            ])
        ]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
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
            package='rviz2',
            executable='rviz2'
        )

    ])