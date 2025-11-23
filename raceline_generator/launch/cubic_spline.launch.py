from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raceline_generator',
            executable='cubic_spline_node',
            name='cubic_spline_raceline',
            output='screen',
        )
    ])
