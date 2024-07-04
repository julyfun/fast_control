from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vr_track_tcp',
            executable='vr_track_tcp_node',
            name='vr_track_tcp',
        ),
        Node(
            package='vr_cali',
            executable='vr_cali_node',
            name='vr_cali',
        )
    ])
