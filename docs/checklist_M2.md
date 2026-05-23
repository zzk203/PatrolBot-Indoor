# M2: Nav2 导航集成 - 任务清单

> 目标：加载静态地图，配置 AMCL 定位，实现单点自主导航。

- [ ] **M2.1** 制作静态地图
  - 在 Gazebo 中运行 SLAM (Cartographer/SLAM Toolbox) 建图
  - 保存地图为 `pgm` 和 `yaml`
- [ ] **M2.2** 配置并启动 Nav2 栈
  - 编写 `nav2_params.yaml`，适配机器人尺寸和传感器
  - 启动 `map_server`、`amcl`、`planner_server`、`controller_server`
- [ ] **M2.3** 测试单点导航
  - 通过 Rviz2 发布 `2D Goal Pose`，观察路径规划与运动执行
  - 微调局部/全局代价地图参数，保证避障顺畅
- [ ] **M2.4** 编写 `navigate_to_pose` 动作客户端工具
  - 用于后续集成行为树时调用
- [ ] **M2.5** 验证动态避障
  - 在 Gazebo 中放置动态障碍物，确认局部规划器能实时避让