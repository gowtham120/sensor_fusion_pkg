from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_fusion_pkg',
            executable='fused_data',
            name='fused_data'
        )
    ])
