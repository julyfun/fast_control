import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit_real.yaml'
    )
    ik_moveit_node = Node(
        package='ik',
        executable='ik_moveit_exe',
        name='ik_moveit_real',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        ik_moveit_node,
    ])
