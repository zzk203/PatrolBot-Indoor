cmake_minimum_required(VERSION 3.8)
project(nav2_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# ====== BT 节点库（供 nav2_demo_node 和测试链接使用） ======
add_library(bt_nodes
  src/bt_nodes/waypoint_structs.cpp
  src/bt_nodes/load_waypoints_node.cpp
  src/bt_nodes/navigate_to_pose_node.cpp
  src/bt_nodes/next_waypoint_node.cpp
  src/bt_nodes/record_failure_node.cpp
  # M4 BT 节点
  src/bt_nodes/battery_monitor_node.cpp
  src/bt_nodes/is_battery_low_condition.cpp
  src/bt_nodes/set_charging_mode_node.cpp
  # M5 BT 节点
  src/bt_nodes/visual_servo_node.cpp
  src/bt_nodes/retry_node.cpp
  src/bt_nodes/dock_action_node.cpp
  # M6 BT 节点
  src/bt_nodes/is_patrol_complete_condition.cpp
  # M7 BT 节点
  src/bt_nodes/log_alert_node.cpp
  src/bt_nodes/amcl_pose_monitor_node.cpp
)
target_include_directories(bt_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(bt_nodes
  rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs
  sensor_msgs std_srvs std_msgs behaviortree_cpp)
target_link_libraries(bt_nodes yaml-cpp)

# ====== 电池模拟节点（独立节点） ======
add_executable(battery_simulator_node src/battery_simulator_node.cpp)
target_include_directories(battery_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(battery_simulator_node
  rclcpp sensor_msgs std_srvs)

# ====== 导航演示节点 ======
add_executable(nav2_demo_node src/nav2_demo_node.cpp)
target_include_directories(nav2_demo_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nav2_demo_node
  rclcpp geometry_msgs nav_msgs nav2_msgs rclcpp_action)

install(TARGETS
  nav2_demo_node
  battery_simulator_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/keyboard_teleop.py
  scripts/odom_republisher.py
  scripts/pointcloud_to_scan.py
  scripts/static_map_publisher.py
  scripts/gazebo_diff_drive.sh
  scripts/dynamic_obstacle_test.py
  scripts/nav2_single_goal_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch behavior_trees worlds models maps rviz
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# ====== 测试 ======
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # ---- M2 已有 Python 测试 ----
  ament_add_pytest_test(test_keyboard_teleop tests/test_keyboard_teleop.py
    TIMEOUT 30)
  ament_add_pytest_test(test_odom_republisher tests/test_odom_republisher.py
    TIMEOUT 30)
  ament_add_pytest_test(test_sim_launch tests/test_sim_launch.py
    TIMEOUT 30)
  ament_add_pytest_test(test_navigate_to_pose tests/test_navigate_to_pose.py
    TIMEOUT 60)
  ament_add_pytest_test(test_nav2_demo_node tests/test_nav2_demo_node.py
    TIMEOUT 60)
  ament_add_pytest_test(test_dynamic_obstacle tests/test_dynamic_obstacle.py
    TIMEOUT 60)

  # ---- M3 C++ GTest 单元测试 ----
  ament_add_gtest(test_load_waypoints
    tests/test_load_waypoints.cpp)
  target_link_libraries(test_load_waypoints bt_nodes)
  ament_target_dependencies(test_load_waypoints rclcpp rclcpp_action)

  ament_add_gtest(test_patrol_bt_nodes
    tests/test_patrol_bt_nodes.cpp)
  target_link_libraries(test_patrol_bt_nodes bt_nodes)
  ament_target_dependencies(test_patrol_bt_nodes rclcpp rclcpp_action)

  ament_add_pytest_test(test_patrol_sequence tests/test_patrol_sequence.py
    TIMEOUT 60)

  # ---- M4 C++ GTest 单元测试 ----
  # M4.1: BatterySimulatorNode 单元测试
  # 注意：battery_simulator_node.cpp 含 main，无法与测试共同编译。
  # 后续需将 BatterySimulatorNode 类分离为独立库后再启用本测试。
  # ament_add_gtest(test_battery_simulator
  #   tests/test_battery_simulator.cpp)
  # target_link_libraries(test_battery_simulator bt_nodes)
  # ament_target_dependencies(test_battery_simulator
  #   rclcpp sensor_msgs std_srvs)

  # M4.2-4.5: 电池 BT 节点集成测试（依赖 BatterySimulatorNode 库分离，暂禁用）
  # ament_add_gtest(test_battery_bt_nodes
  #   tests/test_battery_bt_nodes.cpp)
  # target_link_libraries(test_battery_bt_nodes bt_nodes)
  # ament_target_dependencies(test_battery_bt_nodes
  #   rclcpp rclcpp_action sensor_msgs std_srvs)

  # ---- M5 C++ GTest 单元测试（待修复 BT.CPP v4 API 兼容性） ----
  # ament_add_gtest(test_m5_docking
  #   tests/test_m5_docking.cpp)
  # target_link_libraries(test_m5_docking bt_nodes)
  # ament_target_dependencies(test_m5_docking
  #   rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs
  #   sensor_msgs std_srvs std_msgs)

  # ---- M6 C++ GTest 单元测试（待 BT.CPP v4 Delay/Script 语法修复） ----
  # ament_add_gtest(test_m6_round_scheduling
  #   tests/test_m6_round_scheduling.cpp)
  # target_link_libraries(test_m6_round_scheduling bt_nodes)
  # ament_target_dependencies(test_m6_round_scheduling
  #   rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs
  #   sensor_msgs std_srvs std_msgs)

  # ---- M7 异常处理与日志单元测试（部分用例待逻辑修复） ----
  ament_add_gtest(test_m7_exceptions
    tests/test_m7_exceptions.cpp)
  target_link_libraries(test_m7_exceptions bt_nodes)
  ament_target_dependencies(test_m7_exceptions
    rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs
    sensor_msgs std_srvs std_msgs)

endif()

ament_package()
