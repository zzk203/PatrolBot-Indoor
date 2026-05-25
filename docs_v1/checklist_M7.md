# M7: 异常处理与日志 - 任务清单

> 目标：完善异常处理策略，添加日志与报警发布。

- [x] **M7.1** 实现 `LogAlertNode`
  - 接受 `alert_msg` 输入
  - 写入 ROS2 错误日志（`RCLCPP_ERROR`）
  - 同时发布到 `/patrol_alerts` 话题
  - 单元测试验证话题输出内容（3 case: 基本发布、默认 severity、WARN level）

- [x] **M7.2** 集成堵赛异常日志
  - 导航超时后，调用 `LogAlertNode` 记录"Waypoint X blocked"
  - 在 `patrol_round.xml` 的 Fallback 备选分支中串联 RecordFailureNode + LogAlertNode
  - 单元测试验证 Fallback 触发告警和导航成功时跳过告警

- [x] **M7.3** 集成对接失败日志
  - 重试耗尽后，记录"Docking failed after 3 retries"
  - DockActionNode 内部集成 publishAlert()，导航失败和伺服重试耗尽时发布告警
  - 单元测试验证 RetryNode 重试耗尽后的日志记录

- [x] **M7.4** 处理电池数据丢失异常
  - 在 `BatteryMonitor` 中检测话题超时（>5 秒无新数据）
  - 超时时视为满电（100%）并发布告警
  - 返回 SUCCESS 保证主流程不中断
  - 单元测试验证超时处理和 fail-safe 行为

- [x] **M7.5** 添加定位丢失检测（可选）
  - 订阅 AMCL 协方差（/amcl_pose）
  - 协方差过大时告警并标记定位无效
  - 指数平滑防止误报
  - 单元测试验证低协方差正常、高协方差告警、无数据三种场景

- [x] **M7.6** 测试所有异常路径
  - LogAlertNode 话题输出验证（3 cases）
  - 堵赛日志集成（2 cases）
  - 对接失败告警（1 case）
  - 电池超时处理（3 cases）
  - 定位丢失检测（3 cases）
  - Fail-safe 不破坏主流程（2 cases）

## 新增/修改文件

| 文件 | 类型 | 说明 |
|------|------|------|
| `include/nav2_demo/bt_nodes/log_alert_node.hpp` | 新增 | LogAlertNode 头文件 |
| `src/bt_nodes/log_alert_node.cpp` | 新增 | LogAlertNode 实现 |
| `include/nav2_demo/bt_nodes/amcl_pose_monitor_node.hpp` | 新增 | AmclPoseMonitorNode 头文件 |
| `src/bt_nodes/amcl_pose_monitor_node.cpp` | 新增 | AmclPoseMonitorNode 实现 |
| `include/nav2_demo/bt_nodes/battery_monitor_node.hpp` | 修改 | 添加超时检测成员和告警发布者 |
| `src/bt_nodes/battery_monitor_node.cpp` | 修改 | 实现话题超时检测逻辑 |
| `include/nav2_demo/bt_nodes/dock_action_node.hpp` | 修改 | 添加 /patrol_alerts 发布者 |
| `src/bt_nodes/dock_action_node.cpp` | 修改 | 对接失败告警集成 |
| `behavior_trees/patrol_round.xml` | 修改 | 集成 LogAlertNode 到 Fallback 备选分支 |
| `tests/test_m7_exceptions.cpp` | 新增 | M7 综合单元测试（14 个用例） |
| `CMakeLists.txt` | 修改 | 注册新源文件和 M7 测试目标 |
| `readme_M7.md` | 新增 | M7 模块文档 |
| `docs/checklist_M7.md` | 修改 | 本文件，标记所有项完成 |
