from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression


def generate_launch_description():

    # -----------------------------
    # 1) Launch arguments
    # -----------------------------
    map_yaml_arg = DeclareLaunchArgument(
        "map_yaml",
        default_value="/home/misys/shared_dir/map.yaml",
        description="Full path to map.yaml file"
    )

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="bspline",
        description="Choose raceline generator: bspline / cubic / catmullrom / clothoid"
    )

    map_yaml = LaunchConfiguration("map_yaml")
    mode = LaunchConfiguration("mode")

    # -----------------------------
    # 2) Raceline smoothing nodes
    # -----------------------------
    raceline_nodes = [

        # B-SPLINE
        Node(
            package="raceline_generator",
            executable="bspline_raceline_node",
            name="bspline_raceline_node",
            remappings=[("/raw_path", "/centerline_raw")],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'bspline'"]))
        ),

        # CUBIC SPLINE
        Node(
            package="raceline_generator",
            executable="cubic_spline_node",
            name="cubic_spline_node",
            remappings=[("/raw_path", "/centerline_raw")],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'cubic'"]))
        ),

        # CATMULL-ROM
        Node(
            package="raceline_generator",
            executable="catmullrom_raceline_node",
            name="catmullrom_raceline_node",
            remappings=[("/raw_path", "/centerline_raw")],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'catmullrom'"]))
        ),

        # CLOTHOID
        Node(
            package="raceline_generator",
            executable="clothoid_raceline_node",
            name="clothoid_raceline_node",
            remappings=[("/raw_path", "/centerline_raw")],
            condition=IfCondition(PythonExpression(["'", mode, "' == 'clothoid'"]))
        ),
    ]

    # -----------------------------
    # 3) LaunchDescription
    # -----------------------------
    return LaunchDescription([

        map_yaml_arg,
        mode_arg,

        # Map Loader Node
        Node(
            package="raceline_generator",
            executable="map_loader_node",
            name="map_loader_node",
            parameters=[{"map_yaml": map_yaml}],
            output="screen",
        ),

        # Boundary Extractor
        Node(
            package="raceline_generator",
            executable="boundary_extractor_node",
            name="boundary_extractor_node",
            output="screen",
        ),

        # Centerline Generator
        Node(
            package="raceline_generator",
            executable="centerline_generator_node",
            name="centerline_generator_node",
            output="screen",
        ),

        # 🔥 IMPORTANT: insert smoothing nodes here
        *raceline_nodes
    ])
