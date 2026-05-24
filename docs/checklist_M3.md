# M3: 巡逻序列实现 - 任务清单

> 目标：加载预定义巡逻点，使用行为树逐个导航，实现基本巡逻循环。

- [x] **M3.1** 配置巡逻点
  - [x] 编写 `config/waypoints.yaml`，包含 5 个巡逻点、充电预备点、充电桩坐标
  - [x] YAML 格式验证测试（test_patrol_sequence.py: TestWaypointConfig）
- [x] **M3.2** 实现 `LoadWaypointsNode`
  - [x] 从 YAML 文件读取所有位姿（patrol_points / charge_standby / charge_dock）
  - [x] 写入行为树黑板（patrol_waypoints / waypoints_count / navigation_timeout）
  - [x] 单元测试（test_load_waypoints.cpp: 8 个测试用例）
- [x] **M3.3** 实现 `NavigateToPoseNode` 行为树节点
  - [x] 封装 `navigate_to_pose` 动作客户端（StatefulActionNode）
  - [x] 支持超时端口（timeout），返回 SUCCESS/TIMEOUT/FAILURE
  - [x] 单元测试（test_patrol_sequence.py: TestNavigateToPoseTiming）
- [x] **M3.4** 编写 `patrol_round.xml` 子树
  - [x] 使用 Repeat 遍历 `patrol_waypoints`，逐个调用 NavigateToPoseNode
  - [x] 到达每个点后短暂停留（Wait wait_duration）
  - [x] XML 格式验证 + 节点存在性测试
- [x] **M3.5** 实现堵赛放弃逻辑
  - [x] 导航超时后使用 Fallback 返回 FAILURE → RecordFailureNode 接管
  - [x] 黑板记录失败事件（failure_count / last_failure_reason），继续下一巡逻点
  - [x] 单元测试（test_patrol_bt_nodes.cpp: FallbackRecoveryTest）
- [x] **M3.6** 集成至主行为树，测试巡逻序列
  - [x] patrol_main.xml 顶层集成入口
  - [x] Python 集成测试验证多航点顺序执行与失败恢复（test_patrol_sequence.py）

## 测试统计

| 测试文件 | 类型 | 用例数 | 覆盖模块 |
|----------|------|--------|----------|
| test_load_waypoints.cpp | C++ GTest | 8 | M3.2 LoadWaypointsNode + Waypoint 转换 |
| test_patrol_bt_nodes.cpp | C++ GTest | 9 | M3.3-5 NextWaypointNode + RecordFailureNode + Fallback |
| test_patrol_sequence.py | Python pytest | 10 | M3.1-6 集成测试（配置 + 时序 + 失败恢复 + XML） |

## 生成文件清单

```
nav2_demo/
├── config/waypoints.yaml
├── include/nav2_demo/bt_nodes/
│   ├── waypoint_structs.hpp
│   ├── load_waypoints_node.hpp
│   ├── navigate_to_pose_node.hpp
│   ├── next_waypoint_node.hpp
│   └── record_failure_node.hpp
├── src/bt_nodes/
│   ├── waypoint_structs.cpp
│   ├── load_waypoints_node.cpp
│   ├── navigate_to_pose_node.cpp
│   ├── next_waypoint_node.cpp
│   └── record_failure_node.cpp
├── behavior_trees/
│   ├── patrol_round.xml
│   └── patrol_main.xml
├── tests/
│   ├── test_load_waypoints.cpp
│   ├── test_patrol_bt_nodes.cpp
│   └── test_patrol_sequence.py
└── readme_M3.md
```
