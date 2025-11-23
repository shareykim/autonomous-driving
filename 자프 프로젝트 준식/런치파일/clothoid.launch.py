from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raceline_generator',
            executable='clothoid_node',
            name='clothoid_raceline',
            output='screen',
        )
    ])
