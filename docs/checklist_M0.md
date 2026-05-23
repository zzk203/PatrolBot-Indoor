# M0: 学习准备与环境配置 - 任务清单

> 目标：掌握 ROS2 基础概念、C++ 客户端编程、Gazebo 仿真基础，并完成开发环境搭建。

## 任务列表

- [ ] **M0.1** 学习 ROS2 核心概念（节点/话题/服务/动作）
  - 完成官方教程：Understanding ROS2 nodes, topics, services, actions
  - 实际操作：运行 `talker/listener` 示例
- [ ] **M0.2** 学习 ROS2 C++ 客户端 (rclcpp)
  - 编写一个自定义节点，发布/订阅自定义消息
  - 实现一个 Service 服务器/客户端
  - 实现一个 Action 服务器/客户端
- [ ] **M0.3** 安装 Gazebo Fortress 并运行官方示例
  - 安装 `ignition-gazebo` 和 `ros_gz` 桥接包
  - 运行差分驱动机器人模型，用键盘控制移动
  - 理解 Gazebo 世界、模型、插件基本概念
- [ ] **M0.4** 配置开发工作空间
  - 创建 `patrol_robot_ws` 工作空间
  - 初始化 `src` 目录结构（如设计文档所示）
  - 安装依赖：`ros-humble-navigation2`, `ros-humble-nav2-bringup`, `ros-humble-behaviortree-cpp-v4` 等
- [ ] **M0.5** 学习 BehaviorTree.CPP 核心概念
  - 理解 tick 机制、黑板、常用节点类型 (Sequence/Fallback/Decorator)
  - 运行官方教程示例，用 Groot2 可视化
- [ ] **M0.6** 学习 Nav2 基本使用
  - 使用 Nav2 官方示例跑通 TurtleBot 仿真导航
  - 理解 Global Planner, Local Controller, Costmap, AMCL 作用