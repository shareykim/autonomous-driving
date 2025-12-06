## ros2 launch raceline_generator map_raceline.launch.py map_yaml:=/home/misys/maps/track1.yaml (맵 위치)


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    # map.yaml 경로를 launch 인자로 받을 수 있게 설정
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/misys/shared_dir/map.yaml",
        description="Full path to map.yaml file"
    )

    map_yaml = LaunchConfiguration("map_yaml")

    return LaunchDescription([

        map_yaml_arg,

        # 1) Map Loader Node
        Node(
            package="raceline_generator",
            executable="map_loader_node",
            name="map_loader_node",
            output="screen",
            parameters=[{"map_yaml": map_yaml}],
        ),

        # 2) Boundary Extractor Node
        Node(
            package="raceline_generator",
            executable="boundary_extractor_node",
            name="boundary_extractor_node",
            output="screen",
        ),

        # 3) Centerline Generator Node
        Node(
            package="raceline_generator",
            executable="centerline_generator_node",
            name="centerline_generator_node",
            output="screen",
        ),

        # 4) B-spline Raceline Node (기존 smoothing 노드)
        Node(
            package="raceline_generator",
            executable="bspline",
            name="bspline_raceline_node",
            output="screen",
            remappings=[
                ("/raw_path", "/centerline_raw")  # 입력을 중앙선으로 변경
            ]
        ),

    ])
