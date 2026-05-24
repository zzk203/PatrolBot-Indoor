完整文件结构
nav2_demo/
├── worlds/patrol_world.sdf              ← Gazebo 世界（墙壁+障碍+机器人）
├── models/patrol_bot/
│   ├── model.sdf                        ← 机器人 SDF（可独立 include）
│   └── model.urdf                       ← 机器人 URDF（TF树定义）
├── maps/
│   ├── office.pgm + office.yaml         ← 占据栅格地图（10×10m）
├── config/
│   ├── nav2_params.yaml                 ← Nav2 全部参数
│   └── gz_bridge.yaml                   ← 桥接配置（备用）
├── behavior_trees/
│   └── navigate_w_replanning.xml        ← Nav2 行为树
├── launch/patrol_sim.launch.py          ← ★ 一键启动全部节点
├── rviz/nav2_view.rviz                  ← RViz2 配置
├── scripts/
│   ├── odom_republisher.py              ← 里程计帧名转换
│   └── gazebo_diff_drive.sh             ← 独立小车演示
└── src/nav2_demo_node.cpp              ← 导航目标客户端
数据流全景
┌─────────────────────────────────────────────────────────────────┐
│  Gazebo 仿真世界 (patrol_world.sdf)                              │
│  ┌──────────────────────┐    ┌──────────────────────┐           │
│  │ patrol_bot 机器人      │    │ 静态物体             │           │
│  │ ├─ DiffDrive 插件      │    │ ├─ 墙壁 (×4)         │           │
│  │ │  /cmd_vel ← 速度指令  │    │ ├─ 柱子              │           │
│  │ │  /odometry → 里程计   │    │ ├─ 障碍物 (×4)       │           │
│  │ ├─ GPU Lidar          │    │ └─ 地面              │           │
│  │ │  /scan → 点云        │    └──────────────────────┘           │
│  │ └─ 车轮 + 万向轮       │                                       │
│  └──────────────────────┘                                       │
└─────────┬───────────────────────────────────────────────────────┘
          │
    ros_gz_bridge (话题翻译)
          │
    ┌─────┼──────────────┬──────────────┬──────────────┐
    ▼     ▼              ▼              ▼              ▼
  /cmd_vel  /scan_cloud  /raw_odom    /clock
    │        │              │
    │   pointcloud_to_   odom_republisher
    │    laserscan        (帧名转换)
    │        │              │
    │       /scan         /odom (odom→base_footprint)
    │        │              │
    └────────┴──────────────┴────────┐
                                     ▼
                            ┌─────────────────┐
                            │  Nav2 导航栈      │
                            │  ├─ map_server   │ ← office.yaml
                            │  ├─ AMCL 定位     │ ← map→odom TF
                            │  ├─ planner      │ ← 全局路径 /plan
                            │  ├─ controller   │ ← 局部控制
                            │  ├─ bt_navigator │ ← 行为树调度
                            │  └─ costmaps     │ ← 代价地图
                            └─────────────────┘
                                     │
                            robot_state_publisher
                            (静态TF: base_footprint→base_link→lidar_link)