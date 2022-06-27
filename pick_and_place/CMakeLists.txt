cmake_minimum_required(VERSION 3.5)
project(pick_and_place)

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

find_package(ament_cmake REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(simple_grasping REQUIRED)

add_executable(${PROJECT_NAME}
  src/pick_and_place.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  grasping_msgs
  moveit_core
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  grasping_msgs
  moveit_core
  moveit_task_constructor_core
  rclcpp
  rclcpp_action
  simple_grasping
)
ament_package()
