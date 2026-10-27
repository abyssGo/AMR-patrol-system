"""Launch the Control <-> AMR bridge for one robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot1'),
        DeclareLaunchArgument('route_action', default_value='patrol_route'),
        Node(
            package='patrol_amr_bridge',
            executable='amr_bridge_node',
            name=['patrol_amr_bridge_', robot_id],
            output='screen',
            parameters=[{
                'robot_id': robot_id,
                'route_action': LaunchConfiguration('route_action'),
            }],
        ),
    ])
