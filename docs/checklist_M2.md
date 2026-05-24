# M2: Nav2 导航集成 - 任务清单

> 目标：加载静态地图，配置 AMCL 定位，实现单点自主导航。

- [x] **M2.1** 制作静态地图
  - 在 Gazebo 中运行 SLAM (Cartographer/SLAM Toolbox) 建图
  - 保存地图为 `pgm` 和 `yaml`
  - **完成**: `maps/office.pgm` + `office.yaml` 已存在（200×200 @ 0.05m, 10m×10m, trinary 模式）
  - **审查结论**: 地图分辨率、原点、阈值参数合格，与 patrol_world 场景匹配

- [x] **M2.2** 配置并启动 Nav2 栈
  - 编写 `nav2_params.yaml`，适配机器人尺寸和传感器
  - 启动 `map_server`、`amcl`、`planner_server`、`controller_server`
  - **完成**: `nav2_params.yaml` 包含完整 Nav2 配置，修复了重复 behavior_server 段；`patrol_sim.launch.py` 包含完整 Nav2 节点栈，注释完善

- [x] **M2.3** 测试单点导航
  - 通过 Rviz2 发布 `2D Goal Pose`，观察路径规划与运动执行
  - 微调局部/全局代价地图参数，保证避障顺畅
  - **完成**: 编写 `tests/test_navigate_to_pose.py`（11 个测试用例, Mock Action Server）
  - **完成**: `scripts/nav2_single_goal_test.py` 命令行导航测试脚本

- [x] **M2.4** 编写 `navigate_to_pose` 动作客户端工具
  - 用于后续集成行为树时调用
  - **完成**: 重构 `include/nav2_demo/nav2_demo_node.hpp` + `src/nav2_demo_node.cpp`
  - **改进**: 参数化航点配置、取消功能、目标句柄跟踪、错误信息输出
  - **完成**: `tests/test_nav2_demo_node.py`（Mock Action Server, 10 个测试用例）

- [x] **M2.5** 验证动态避障
  - 在 Gazebo 中放置动态障碍物，确认局部规划器能实时避让
  - **完成**: `tests/test_dynamic_obstacle.py`（11 个测试用例，验证参数配置）
  - **完成**: `scripts/dynamic_obstacle_test.py` 动态障碍物测试运行脚本

## 测试运行

```bash
# M2.3 单点导航测试
colcon test --packages-select nav2_demo --ctest-args -R test_navigate_to_pose

# M2.4 动作客户端测试
colcon test --packages-select nav2_demo --ctest-args -R test_nav2_demo_node

# M2.5 动态避障测试
colcon test --packages-select nav2_demo --ctest-args -R test_dynamic_obstacle

# 全部 M2 测试
colcon test --packages-select nav2_demo --ctest-args -R "test_navigate_to_pose|test_nav2_demo_node|test_dynamic_obstacle"
```

## 文件清单

| 文件 | 模块 | 说明 |
|------|------|------|
| `config/nav2_params.yaml` | M2.2 | Nav2 完整参数（修复重复段） |
| `launch/patrol_sim.launch.py` | M2.2 | 完整启动文件（含 Nav2 注释） |
| `include/nav2_demo/nav2_demo_node.hpp` | M2.4 | 封装的动作客户端头文件 |
| `src/nav2_demo_node.cpp` | M2.4 | 重构的导航节点实现 |
| `tests/test_navigate_to_pose.py` | M2.3 | 单点导航 Mock 测试（11 用例） |
| `tests/test_nav2_demo_node.py` | M2.4 | 动作客户端 Mock 测试（10 用例） |
| `tests/test_dynamic_obstacle.py` | M2.5 | 动态避障参数验证（11 用例） |
| `scripts/nav2_single_goal_test.py` | M2.3 | 命令行单点导航测试 |
| `scripts/dynamic_obstacle_test.py` | M2.5 | 动态障碍物避障测试脚本 |
