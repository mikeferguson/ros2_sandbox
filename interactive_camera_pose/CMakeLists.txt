cmake_minimum_required(VERSION 3.5)
project(interactive_camera_pose)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(interactive_camera_pose src/interactive_camera_pose.cpp)
ament_target_dependencies(interactive_camera_pose
  geometry_msgs
  interactive_markers
  rclcpp
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(
  TARGETS
    interactive_camera_pose
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
