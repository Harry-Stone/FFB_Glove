from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
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
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True}
            ]
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=PathJoinSubstitution([
                FindPackageShare('glove_description'),
                '..'
            ])
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[
                EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
                ':/opt/ros/kilted/lib'
            ]
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'shapes.sdf'],
            output='screen'
        ),

        # Spawn robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'glove',
                '-world', 'shapes',
                '-x', '0', '-y', '0', '-z', '1.5'
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
            ],
            output='screen'
        ),

        Node(
            package='state_manager',
            executable='state_manager',
        ),

        Node(
            package='encoder_reader',
            executable='encoder_reader',
        )

    ])
