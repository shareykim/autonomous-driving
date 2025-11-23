from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raceline_generator',
            executable='bspline',
            name='bspline_raceline',
            output='screen',
        )
    ])
