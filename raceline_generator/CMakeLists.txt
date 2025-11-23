cmake_minimum_required(VERSION 3.5)
project(raceline_generator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

###########################
#  Executable: path_logger
###########################
add_executable(path_logger_node src/path_logger_node.cpp)
ament_target_dependencies(path_logger_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###########################
#  Executable: cubic spline
###########################
add_executable(cubic_spline_raceline_node src/cubic_spline_raceline_node.cpp)
ament_target_dependencies(cubic_spline_raceline_node
  rclcpp
  nav_msgs
  geometry_msgs
)

###########################
#  Executable: catmullrom
###########################
add_executable(catmullrom_raceline_node src/catmullrom_raceline_node.cpp)
ament_target_dependencies(catmullrom_raceline_node
  rclcpp
  nav_msgs
  geometry_msgs
)

###########################
#  Executable: clothoid
###########################
add_executable(clothoid_raceline_node src/clothoid_raceline_node.cpp)
ament_target_dependencies(clothoid_raceline_node
  rclcpp
  nav_msgs
  geometry_msgs
)

###########################
#  Executable: bspline
###########################
add_executable(bspline_raceline_node src/bspline_raceline_node.cpp)
ament_target_dependencies(bspline_raceline_node
  rclcpp
  nav_msgs
  geometry_msgs
)

###########################
#  Install executables
###########################
install(
  TARGETS
    path_logger_node
    cubic_spline_raceline_node
    catmullrom_raceline_node
    clothoid_raceline_node
    bspline_raceline_node
  DESTINATION lib/${PROJECT_NAME}
)

###########################
#  Install launch files
###########################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

