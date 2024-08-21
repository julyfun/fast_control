import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    left_config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit_left.yaml'
    )
    ik_moveit_left_node = Node(
        package='ik',
        executable='ik_moveit_exe',
        name='ik_moveit_left',
        output='screen',
        parameters=[left_config]
    )

    right_config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit_right.yaml'
    )
    ik_moveit_right_node = Node(
        package='ik',
        executable='ik_moveit_exe',
        name='ik_moveit_right',
        output='screen',
        parameters=[right_config]
    )

    return LaunchDescription([
        ik_moveit_left_node,
        ik_moveit_right_node
    ])
