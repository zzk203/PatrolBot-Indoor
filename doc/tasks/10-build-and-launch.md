# 10. 构建系统、启动脚本与测试

> 对应需求: 全部（构建验证、部署启动、测试覆盖）
> 参考设计: detailed_design.md §7, §10

## 10.1 patrol_bot_interfaces 构建

- [ ] 编写 `patrol_bot_interfaces/CMakeLists.txt`（rosidl_generate_interfaces）
- [ ] 编写 `patrol_bot_interfaces/package.xml`（format=3, rosidl_interface_packages 成员组）
- [ ] 编译验证消息生成成功

## 10.2 patrol_bot 构建

- [ ] 编写 `patrol_bot/package.xml`（depend: rclcpp, rclcpp_action, std_srvs, nav2_msgs, sensor_msgs, behaviortree_cpp, yaml-cpp, OpenCV, cv_bridge, patrol_bot_interfaces）
- [ ] 编写 `patrol_bot/CMakeLists.txt`：
  - 设置 C++17 标准
  - find_package 所有依赖
  - 构建静态库 `patrol_bot_core`（config_loader, battery_model, alarm_manager, patrol_logger, nav2_action_client, camera_buffer）
  - target_link_libraries: behaviortree_cpp, OpenCV
  - 构建静态库 `patrol_bt_nodes`（所有 bt_nodes/ 下源码）
  - target_link_libraries: patrol_bot_core, behaviortree_cpp
  - 构建可执行文件 `patrol_bot_node`
  - target_link_libraries: patrol_bot_core, patrol_bt_nodes, behaviortree_cpp
  - install: 可执行文件、bt_xml、config、launch 目录

## 10.3 Launch 文件

- [ ] 创建 `launch/` 目录
- [ ] 编写 `launch/patrol_bot.launch.py`（仅启动 patrol_bot_node）
- [ ] 编写 `launch/patrol_simulation.launch.py`（Gazebo + Nav2 + patrol_bot_node 联合启动）
  - 包含 gazebo 世界启动
  - 包含 robot_state_publisher
  - 包含 nav2_bringup
  - 包含 patrol_bot_node

## 10.4 控制脚本

- [ ] 创建 `scripts/` 目录
- [ ] 编写 `scripts/start_patrol.sh`（ros2 service call /patrol/start_patrol）
- [ ] 编写 `scripts/pause_patrol.sh`（ros2 service call /patrol/pause_patrol）
- [ ] 编写 `scripts/resume_patrol.sh`（ros2 service call /patrol/resume_patrol）
- [ ] 编写 `scripts/stop_patrol.sh`（ros2 service call /patrol/stop_patrol）

## 10.5 Nav2 参数

- [ ] 创建 `config/nav2_params.yaml`（Nav2 导航参数：规划器、控制器、行为树等）

## 10.6 整体编译验证

- [ ] `colcon build --packages-select patrol_bot_interfaces patrol_bot` 编译通过
- [ ] 确认 `patrol_bot_node` 可执行文件生成
- [ ] 确认配置文件、XML 文件、launch 文件正确安装

---

## 10.7 单元测试

### 10.7.1 ConfigLoader 测试

- [ ] 创建 `test/test_config_loader.cpp`
- [ ] 测试用例：完整 YAML 正确解析
- [ ] 测试用例：缺少必填字段抛异常
- [ ] 测试用例：缺少可选字段使用默认值
- [ ] 测试用例：类型错误抛异常
- [ ] 测试用例：空路线数组抛异常

### 10.7.2 BatteryModel 测试

- [ ] 创建 `test/test_battery_model.cpp`
- [ ] 测试用例：放电消耗（level 正确下降）
- [ ] 测试用例：充电恢复（level 正确上升）
- [ ] 测试用例：上界钳位（不超过 100.0）
- [ ] 测试用例：下界钳位（不低于 0.0）
- [ ] 测试用例：充放电模式切换
- [ ] 测试用例：BatteryState 消息字段正确性

### 10.7.3 PatrolLogger 测试

- [ ] 创建 `test/test_patrol_logger.cpp`
- [ ] 测试用例：文件创建成功
- [ ] 测试用例：info/warn/error 写入验证
- [ ] 测试用例：时间戳格式正确
- [ ] 测试用例：flush 后内容立即可读

### 10.7.4 AlarmManager 测试

- [ ] 创建 `test/test_alarm_manager.cpp`
- [ ] 测试用例：消息字段完整性
- [ ] 测试用例：warning 不暂停（不修改 patrol_state）
- [ ] 测试用例：critical 发布正确

### 10.7.5 CameraBuffer 测试

- [ ] 创建 `test/test_camera_buffer.cpp`
- [ ] 测试用例：模拟 Image 消息、cv_bridge 转换
- [ ] 测试用例：imwrite 输出文件验证
- [ ] 测试用例：无帧时 save_latest 返回 false

### 10.7.6 Nav2ActionClient 测试

- [ ] 创建 `test/test_nav2_action_client.cpp`
- [ ] 测试用例：mock Action Server 的 send/cancel/check_result 状态机
- [ ] 测试用例：服务器未就绪行为

### 10.7.7 BT Condition 节点测试

- [ ] 创建 `test/test_bt_conditions.cpp`
- [ ] 测试用例：IsBatteryLow 边界值
- [ ] 测试用例：IsAlarmCritical 边界值
- [ ] 测试用例：黑板变量缺失时的行为

### 10.7.8 BT Action 节点测试

- [ ] 创建 `test/test_bt_actions.cpp`
- [ ] 测试用例：StatefulActionNode onStart/onRunning/onHalted 生命周期
- [ ] 测试用例：halt 取消行为
- [ ] 测试用例：超时返回 FAILURE

### 10.7.9 测试构建

- [ ] CMakeLists.txt 中添加测试目标（ament_cmake_gtest）
- [ ] 运行 `colcon test` 通过所有测试

---

## 10.8 集成测试

- [ ] 启动→巡逻→停止：验证状态转移、BT tick 正常、Topic 有数据
- [ ] 低电→充电→恢复：验证 ReactiveFallback 抢占、断点保存/恢复、充电完成切回
- [ ] Critical 异常→暂停：验证 patrol_state 变 PAUSED、/patrol/alarm 发布
- [ ] 导航超时跳过：验证跳过当前点、日志记录、继续下一点
- [ ] pause/resume：验证合法/非法状态拒绝、BT 暂停/恢复
