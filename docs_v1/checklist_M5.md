# M5: 视觉分段对接 - 任务清单

> 目标：实现从预备点到充电桩的精细对接，初期使用理想位姿模拟视觉伺服。

- [x] **M5.1** 实现 `DockActionNode` 结构
  - StatefulActionNode，含 NAVIGATE 和 SERVO 两阶段
  - 先导航到 `dock_prep_pose`（复用 Nav2 navigate_to_pose 动作客户端）
  - 然后执行视觉伺服 P 控制器
  - 伺服阶段内部最多重试 3 次
  - 支持注入位姿源用于测试

- [x] **M5.2** 实现模拟视觉伺服 `VisualServoNode`
  - StatefulActionNode，从订阅的 `/odom` 获取机器人位姿
  - 充电桩位姿从黑板端口参数（dock_pose_x/y/yaw）获取
  - 实现 P 控制器输出 `cmd_vel` 进行对准
  - 成功条件：角度偏差 < 0.05 rad，横向偏差 < 0.02 m
  - 超时 15 秒
  - **可测试性**：通过 `setRobotPoseSource()` 注入自定义位姿源回调
  - 单元测试通过注入位姿验证：已对准、远离中、逐步靠近、超时等场景

- [x] **M5.3** 添加对接重试机制
  - `RetryNode`：DecoratorNode，包装子节点
  - 最多重试 3 次（通过 `max_attempts` 端口配置）
  - 失败时记录 ERROR 级别报警日志
  - 单元测试验证重试次数和最终失败行为

- [x] **M5.4** 模拟充电完成
  - 对接成功后等待 10 秒（`Wait(10.0)`）
  - 充电完成后：`is_low_battery = false`, `battery_level = 100.0`

- [x] **M5.5** 测试完整对接流程
  - 集成测试验证从低电触发到对接成功并满电恢复巡逻
  - 验证：dock_success、充电服务调用次数、电量恢复、断点恢复
