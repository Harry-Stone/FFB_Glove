from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    delayed_state_manager = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to load and the robot to spawn
        actions=[
            Node(
                package='state_manager',
                executable='state_manager',
                output='screen',
                parameters=[ 
                    {'sim_mode': True}, 
                    {'use_sim_time': True}
                ]
            )
        ]
    )

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

    # -----------------------
    # Controller spawners
    # -----------------------

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    glove_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['glove_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([

        # -----------------------
        # Environment variables
        # -----------------------

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

        # -----------------------
        # Robot State Publisher
        # -----------------------

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True}
            ],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '/home/kit-haptics/glove/src/glove_description/shapes.sdf'],
            output='screen'
        ),

        # -----------------------
        # Spawn robot
        # -----------------------

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'glove',
                '-x', '0', '-y', '0', '-z', '1.5'
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/shapes/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/shapes/model/glove/link/f1l3/sensor/f1l3_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                '/world/shapes/model/glove/link/f2l3/sensor/f2l3_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                '/world/shapes/model/glove/link/tl3/sensor/tl3_contact_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            ],
            output='screen'
        ),

        # -----------------------
        # Spawn controllers (delay required)
        # -----------------------

        TimerAction(
            period=5.0,
            actions=[
                joint_state_broadcaster_spawner,
                glove_controller_spawner
            ]
        ),

        # -----------------------
        # Your nodes
        # -----------------------

        Node(
            package='encoder_reader',
            executable='encoder_reader',
            output='screen'
        ),

        Node(
            package='haply_connection_manager',
            executable='haply_connection_manager',
            output='screen'
        ),

        Node(
            package='force_feedback',
            executable='force_feedback',
            output='screen'
        ),

        Node(
            package='dynamixel_manager',
            executable='dynamixel_manager',
            output='screen'
        ),

        Node(
            package='vibration_contactor',
            executable='vibration_contactor',
            output='screen'
        ),

        delayed_state_manager
    ])
