from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    foxglove_bridge_launch = IncludeLaunchDescription(
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
        foxglove_bridge_launch,
        Node(
            package='uancabot_odometry',
            executable='uancabot_node',
            name='uancabot_odometry_node',
            output='screen',
        ),
        Node(
            package='uancabot_odometry',
            executable='path_executor',
            name='uancabot_path_executor',
            output='screen',
        ),
        Node(
            package='uancabot_odometry',
            executable='path_generator',
            name='uancabot_path_generator',
            output='screen',
        ),
    ])
