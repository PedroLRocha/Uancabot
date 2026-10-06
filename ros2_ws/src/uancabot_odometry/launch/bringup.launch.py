from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            ])
        ),
        launch_arguments={'port': '8765'}.items()
    )

    return LaunchDescription([
        foxglove_bridge,
        Node(
            package='uancabot_odometry',
            executable='motor_bridge',
            name='uancabot_motor_bridge',
            output='screen',
        ),
        Node(
            package='uancabot_odometry',
            executable='path_follower',
            name='uancabot_path_follower',
            output='screen',
        ),
        Node(
            package='uancabot_odometry',
            executable='path_generator',
            name='uancabot_path_generator',
            output='screen',
        ),
        Node(
            package='uancabot_odometry',
            executable='route_ui',
            name='uancabot_route_ui',
            output='screen',
        ),
    ])
