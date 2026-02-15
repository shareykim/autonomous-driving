cmake_minimum_required(VERSION 3.5)
project(raceline_generator)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# -------------------------------------
# Dependencies
# -------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# -------------------------------------
# Executables
# -------------------------------------

# Path logger
add_executable(path_logger_node src/path_logger_node.cpp)
ament_target_dependencies(path_logger_node
  rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Cubic spline
add_executable(cubic_spline_raceline_node src/cubic_spline_raceline_node.cpp)
ament_target_dependencies(cubic_spline_raceline_node
  rclcpp nav_msgs geometry_msgs)

# Catmullrom
add_executable(catmullrom_raceline_node src/catmullrom_raceline_node.cpp)
ament_target_dependencies(catmullrom_raceline_node
  rclcpp nav_msgs geometry_msgs)

# Clothoid
add_executable(clothoid_raceline_node src/clothoid_raceline_node.cpp)
ament_target_dependencies(clothoid_raceline_node
  rclcpp nav_msgs geometry_msgs)

# BSpline
add_executable(bspline_raceline_node src/bspline_raceline_node.cpp)
ament_target_dependencies(bspline_raceline_node
  rclcpp nav_msgs geometry_msgs)

# ------------------------------
# NEW: Map loader
# ------------------------------
add_executable(map_loader_node src/map_loader_node.cpp)
ament_target_dependencies(map_loader_node
  rclcpp std_msgs)

target_link_libraries(map_loader_node
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

# ------------------------------
# NEW: Boundary extractor
# ------------------------------
add_executable(boundary_extractor_node src/boundary_extractor_node.cpp)
ament_target_dependencies(boundary_extractor_node
  rclcpp std_msgs nav_msgs)

target_link_libraries(boundary_extractor_node
  ${OpenCV_LIBRARIES}
)

# ------------------------------
# NEW: Centerline generator
# ------------------------------
add_executable(centerline_generator_node src/centerline_generator_node.cpp)
ament_target_dependencies(centerline_generator_node
  rclcpp nav_msgs geometry_msgs)

# -------------------------------------
# Install executables
# -------------------------------------
install(TARGETS
  path_logger_node
  cubic_spline_raceline_node
  catmullrom_raceline_node
  clothoid_raceline_node
  bspline_raceline_node
  map_loader_node
  boundary_extractor_node
  centerline_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
