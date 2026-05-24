# M1: 仿真环境与基础移动 - 任务清单

> 目标：搭建室内仿真世界，导入机器人模型，实现键盘控制和基础里程计显示。

- [x] **M1.1** 创建室内环境世界
  - 编写 `patrol_world.sdf`，包含墙壁、房间、静态障碍物
  - 添加地面和光照
- [x] **M1.2** 设计/导入差分机器人模型
  - 底盘为差分驱动，配有激光雷达和虚拟相机（后续视觉用）
  - 在 Gazebo 中正确加载并驱动
- [x] **M1.3** 实现键盘控制移动
  - 使用 `ros_gz_bridge` 发送 `cmd_vel` 消息
  - 验证机器人运动学模型正确
  - ✅ 注册到 CMakeLists.txt
  - ✅ 集成到 launch 文件（可选，enable_keyboard 参数控制）
  - ✅ 单元测试 `tests/test_keyboard_teleop.py`（10 项，含键值映射/边界情况）
- [x] **M1.4** 发布里程计和 TF 变换
  - 配置 `diff_drive` 插件输出里程计话题
  - 发布 `base_link` → `laser` 等 TF 帧
  - ✅ 单元测试 `tests/test_odom_republisher.py`（8 项，含帧名转换/TF 广播）
- [x] **M1.5** 运行 Rviz2 查看机器人状态
  - 订阅激光雷达数据、里程计、TF 并可视化
  - ✅ 验证测试 `tests/test_sim_launch.py`（22 项，覆盖 M1.1-M1.5）
- [x] **最终产出**
  - ✅ M1 模块文档 `readme_M1.md`
  - ✅ 测试框架已集成（ament_cmake_pytest）
