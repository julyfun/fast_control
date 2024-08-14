import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    right_config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit_right.yaml'
    )
    ik_joint_broadcaster_right_node = Node(
        package='ik',
        executable='ik_joint_broadcaster_exe',
        name='ik_joint_broadcaster_right',
        output='screen',
        parameters=[right_config]
    )

    left_config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit_left.yaml'
    )
    ik_joint_broadcaster_left_node = Node(
        package='ik',
        executable='ik_joint_broadcaster_exe',
        name='ik_joint_broadcaster_left',
        output='screen',
        parameters=[left_config]
    )

    return LaunchDescription([
        ik_joint_broadcaster_right_node,
        ik_joint_broadcaster_left_node
    ])
