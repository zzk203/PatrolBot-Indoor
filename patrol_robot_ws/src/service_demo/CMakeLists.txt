cmake_minimum_required(VERSION 3.8)
project(service_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
)

add_executable(server_node src/server_node.cpp)
target_include_directories(server_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(server_node rclcpp)

add_executable(client_node src/client_node.cpp)
target_include_directories(client_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
rosidl_target_interfaces(client_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
ament_target_dependencies(client_node rclcpp)

install(TARGETS server_node client_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
