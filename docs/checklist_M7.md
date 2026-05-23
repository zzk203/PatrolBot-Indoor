# M7: 异常处理与日志 - 任务清单

> 目标：完善异常处理策略，添加日志与报警发布。

- [ ] **M7.1** 实现 `LogAlertNode`
  - 接受 `alert_msg` 输入
  - 写入 ROS2 错误日志（`RCLCPP_ERROR`）
  - 同时发布到 `/patrol_alerts` 话题
- [ ] **M7.2** 集成堵赛异常日志
  - 导航超时后，调用 `LogAlertNode` 记录“Waypoint X blocked”
- [ ] **M7.3** 集成视觉对接失败日志
  - 重试耗尽后，记录“Docking failed after 3 retries”
- [ ] **M7.4** 处理电池数据丢失异常
  - 在 `BatteryMonitor` 中检测话题超时，视为满电并告警
- [ ] **M7.5** 添加定位丢失检测（可选）
  - 订阅 AMCL 协方差，过大时暂停并告警
- [ ] **M7.6** 测试所有异常路径
  - 人为制造堵塞、遮挡标记、杀死电池节点等，观察日志输出和行为树反应