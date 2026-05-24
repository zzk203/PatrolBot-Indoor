cmake_minimum_required(VERSION 3.8)
project(bt_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(patrol_nodes_lib src/patrol_nodes.cpp)
target_include_directories(patrol_nodes_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(patrol_nodes_lib rclcpp std_msgs behaviortree_cpp)

add_executable(bt_basic_node src/bt_basic_node.cpp)
target_include_directories(bt_basic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_basic_node rclcpp std_msgs behaviortree_cpp)

add_executable(bt_patrol_node src/bt_patrol_node.cpp)
target_include_directories(bt_patrol_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bt_patrol_node patrol_nodes_lib)
ament_target_dependencies(bt_patrol_node rclcpp std_msgs behaviortree_cpp)

install(TARGETS patrol_nodes_lib bt_basic_node bt_patrol_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp
    APPEND_LIBRARY_DIRS "${behaviortree_cpp_LIBRARY_DIRS}")
  target_include_directories(test_bt_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(test_bt_nodes rclcpp std_msgs behaviortree_cpp)
endif()

ament_package()
