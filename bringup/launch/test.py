import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ik'), 'config', 'ik_moveit.yaml'
    )

    return LaunchDescription([
        # Node(
        #     package='vr_track_tcp',
        #     executable='vr_track_tcp_node',
        #     name='vr_track_tcp',
        # ),
        # Node(
        #     package='vr_cali',
        #     executable='vr_cali_node',
        #     name='vr_cali',
        # ),
        Node(
            package='ik',
            executable='ik_test_exe',
            name='ik_test',
            output='screen',
            parameters=[config]
        ),
    ])
