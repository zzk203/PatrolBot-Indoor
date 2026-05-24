cmake_minimum_required(VERSION 3.8)
project(nav2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(nav2_demo_node src/nav2_demo_node.cpp)
target_include_directories(nav2_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_demo_node
  rclcpp geometry_msgs nav_msgs nav2_msgs rclcpp_action)

install(TARGETS nav2_demo_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/odom_republisher.py
  scripts/pointcloud_to_scan.py
  scripts/static_map_publisher.py
  scripts/gazebo_diff_drive.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch behavior_trees worlds models maps rviz
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
