cmake_minimum_required(VERSION 3.5)
project(bt_beer_me)

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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robot_impedance_controllers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(src)

set(dependencies
  behaviortree_cpp_v3
  nav_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  robot_impedance_controllers
  tf2
  tf2_geometry_msgs
)

add_executable(bt_beer_me 
  src/bt_beer_me.cpp
)
ament_target_dependencies(bt_beer_me
  ${dependencies}
)
install (
  TARGETS bt_beer_me
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY behavior_trees 
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
