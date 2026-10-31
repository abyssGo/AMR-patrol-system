"""
AMR PC 실행(설계도 01 PC1 · PC2): mission_supervisor · local_safety_supervisor · 감지 노드.

Nav2 · AMCL은 turtlebot4_navigation으로 따로 띄운다. Nav2 collision_monitor 출력은
cmd_vel_safety_in이고, local_safety_supervisor가 DriveToken이 유효할 때만 cmd_vel로 넘긴다.

  ros2 launch patrol_amr_safety amr.launch.py robot_id:=robot1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    common = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    # TurtleBot4Navigator는 절대 /tf를 구독하므로 로봇 namespace로 옮긴다.
    tf = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot1',
                              choices=['robot1', 'robot6']),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='patrol_amr_safety', executable='local_safety_supervisor',
             namespace=robot_id, output='screen', parameters=[common]),
        Node(package='patrol_amr_safety', executable='mission_supervisor',
             namespace=robot_id, output='screen', parameters=[common], remappings=tf),
        Node(package='patrol_amr_safety', executable='vision_node_v2',
             namespace=robot_id, output='screen', parameters=[common]),
    ])
