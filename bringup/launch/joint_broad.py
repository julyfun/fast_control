import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit_vmware.yaml'
    )
    ik_joint_broadcaster_node = Node(
        package='ik',
        executable='ik_joint_broadcaster_exe',
        name='ik_joint_broadcaster_vmware',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        ik_joint_broadcaster_node,
    ])
