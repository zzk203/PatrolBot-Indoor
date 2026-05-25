cmake_minimum_required(VERSION 3.8)
project(patrol_bot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(patrol_bot_interfaces REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

# 核心库
add_library(patrol_bot_core STATIC
  src/config_loader.cpp
  src/patrol_logger.cpp
  src/battery_model.cpp
  src/nav2_action_client.cpp
  src/camera_buffer.cpp
  src/alarm_manager.cpp
)

target_include_directories(patrol_bot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(patrol_bot_core
  rclcpp rclcpp_action std_srvs std_msgs sensor_msgs
  geometry_msgs nav2_msgs cv_bridge patrol_bot_interfaces
  tf2
)

target_link_libraries(patrol_bot_core
  yaml-cpp
  behaviortree_cpp::behaviortree_cpp
  ${OpenCV_LIBS}
)

# 行为树节点库
add_library(patrol_bt_nodes STATIC
  src/bt_nodes/conditions/is_battery_low.cpp
  src/bt_nodes/conditions/has_alarm.cpp
  src/bt_nodes/conditions/is_alarm_critical.cpp
  src/bt_nodes/actions/navigate_to_waypoint.cpp
  src/bt_nodes/actions/navigate_to_charger.cpp
  src/bt_nodes/actions/wait_at_waypoint.cpp
  src/bt_nodes/actions/capture_image.cpp
  src/bt_nodes/actions/simulate_charging.cpp
  src/bt_nodes/actions/handle_alarm.cpp
  src/bt_nodes/actions/save_patrol_context.cpp
  src/bt_nodes/actions/restore_patrol_context.cpp
  src/bt_nodes/actions/load_routes.cpp
  src/bt_nodes/actions/set_route_context.cpp
)

target_include_directories(patrol_bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(patrol_bt_nodes
  patrol_bot_core
  behaviortree_cpp::behaviortree_cpp
)

# 安装（数据目录）
install(DIRECTORY
  config
  bt_xml
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装（控制脚本，带可执行权限）
install(PROGRAMS
  scripts/start_patrol.sh
  scripts/pause_patrol.sh
  scripts/resume_patrol.sh
  scripts/stop_patrol.sh
  DESTINATION lib/${PROJECT_NAME}
)

# 主可执行文件
add_executable(patrol_bot_node src/patrol_bot_node.cpp)
ament_target_dependencies(patrol_bot_node
  rclcpp std_srvs sensor_msgs patrol_bot_interfaces
)
target_link_libraries(patrol_bot_node
  patrol_bot_core
  patrol_bt_nodes
  behaviortree_cpp::behaviortree_cpp
)

install(TARGETS patrol_bot_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_config_loader test/test_config_loader.cpp)
  target_link_libraries(test_config_loader patrol_bot_core)

  ament_add_gtest(test_patrol_logger test/test_patrol_logger.cpp)
  target_link_libraries(test_patrol_logger patrol_bot_core)

  ament_add_gtest(test_battery_model test/test_battery_model.cpp)
  target_link_libraries(test_battery_model patrol_bot_core)

  ament_add_gtest(test_nav2_action_client test/test_nav2_action_client.cpp)
  target_link_libraries(test_nav2_action_client patrol_bot_core)

  ament_add_gtest(test_camera_buffer test/test_camera_buffer.cpp)
  target_link_libraries(test_camera_buffer patrol_bot_core)

  ament_add_gtest(test_bt_nodes test/test_bt_nodes.cpp)
  target_link_libraries(test_bt_nodes patrol_bt_nodes patrol_bot_core)
endif()

ament_package()
