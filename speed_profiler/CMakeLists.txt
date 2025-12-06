cmake_minimum_required(VERSION 3.5)
project(speed_profiler)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Executable
add_executable(speed_profiler_node src/speed_profiler_node.cpp)

ament_target_dependencies(speed_profiler_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

# Install binary  → install/lib/speed_profiler_node 에 깔림
install(TARGETS speed_profiler_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/  → 나중에 launch 파일 쓸 수 있게
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

