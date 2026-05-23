cmake_minimum_required(VERSION 3.8)
project(topic_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Demo.msg"
)

add_executable(publisher_node src/publisher_node.cpp)
target_include_directories(publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(publisher_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(publisher_node rclcpp)

add_executable(subscriber_node src/subscriber_node.cpp)
target_include_directories(subscriber_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(subscriber_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(subscriber_node rclcpp)

install(TARGETS publisher_node subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
