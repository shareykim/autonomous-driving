from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raceline_generator',
            executable='path_logger_node',
            name='path_logger',
            output='screen',
        )
    ])
