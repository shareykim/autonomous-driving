from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speed_profiler',
            executable='speed_profiler_node',
            name='speed_profiler',
            output='screen',
            parameters=[{
                'a_lat_max': 3.0,
                'v_min': 0.5,
                'v_max': 5.0,
                'lookahead_points': 15,
                'smooth_window': 7,
                'epsilon': 1e-6,
            }]
        )
    ])

