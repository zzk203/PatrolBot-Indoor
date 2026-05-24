#!/bin/bash
# ============================================================
#  Gazebo 差速驱动机器人仿真演示脚本
#
#  场景：两个差速驱动小车 (blue/green)
#  控制蓝色小车执行：前进 → 左转 → 右转 → 后退 → 停止
#
#  数据流:
#    ros2 topic pub /cmd_vel  →  ros_gz_bridge  →  Gazebo小车运动
#    Gazebo小车里程计          →  ros_gz_bridge  →  ros2 topic /odom
# ============================================================

# --- 第1步：启动 Gazebo 仿真世界 ---
# -r  : 启动后立即运行物理仿真（不暂停）
# -v 2: 日志详细程度（0-4，越大越详细）
# diff_drive.sdf: 内置示例世界，包含地面、光源、两台小车
echo ">>> [1/5] 启动 Gazebo 仿真世界..."
ign gazebo -r -v 2 diff_drive.sdf &
GAZEBO_PID=$!

# 等待 Gazebo 完全启动（GUI 窗口弹出、物理引擎就绪）
echo "    等待 Gazebo 就绪..."
sleep 6

# --- 第2步：启动 ROS2 ↔ Gazebo 话题桥接 ---
# ros_gz_bridge 是 ROS2 和 Gazebo 之间的"翻译官"
# 格式：/话题名@ROS2消息类型@Gazebo消息类型
#
# 桥接含义：
#   /cmd_vel (ROS2)  →  Gazebo内部 /model/vehicle_blue/cmd_vel
#     ┌─────────────┐               ┌──────────────────┐
#     │ ROS2 节点    │  发布速度指令   │ DiffDrive Plugin │  驱动车轮
#     │ 发布 cmd_vel │──────────────▶│ 接收 cmd_vel     │─────────▶ 小车移动
#     └─────────────┘               └──────────────────┘
#
#   Gazebo odom  →  /odom (ROS2)
#     ┌──────────────────┐          ┌─────────────┐
#     │ DiffDrive Plugin  │ 发布里程计 │ ROS2 节点    │
#     │ 计算里程计        │─────────▶│ 订阅 /odom   │  监控位置
#     └──────────────────┘          └─────────────┘
echo ""
echo ">>> [2/5] 启动 ROS2 ↔ Gazebo 桥接..."
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
BRIDGE_PID=$!
sleep 2

# --- 第3步：演示自动控制序列 ---
echo ""
echo "=================================="
echo "  [3/5] 开始小车自动演示"
echo "  观察 Gazebo 窗口中蓝色小车的运动"
echo "=================================="

# === 演示3.1：前进 ===
echo ""
echo "  ▶ 演示3.1: 小车前进 3 秒"
echo "  ----------------------------------------"
echo "  指令: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist"
echo "        \"{linear: {x: 0.5}, angular: {z: 0.0}}\""
echo ""
echo "  参数说明:"
echo "    linear.x  = 0.5   → 沿X轴前进，速度 0.5 m/s（线速度）"
echo "    linear.y  = 0.0   → 沿Y轴不动（差速车无法横向移动）"
echo "    angular.z = 0.0   → 绕Z轴不转（角速度为0 = 直行）"
echo ""
echo "  消息类型 geometry_msgs/msg/Twist:"
echo "    Vector3 linear   (线速度: x=前进, y=横向, z=升降)"
echo "    Vector3 angular  (角速度: x=翻滚, y=俯仰, z=偏航)"
echo "  ----------------------------------------"
# 持续发送速度指令 3 秒（-r 参数指定发布频率 Hz）
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}" &
PUB_PID=$!
sleep 3
kill $PUB_PID 2>/dev/null

# === 演示3.2：左转 ===
echo ""
echo "  ▶ 演示3.2: 小车原地左转 2 秒"
echo "  ----------------------------------------"
echo "  指令: ros2 topic pub /cmd_vel ... \"{linear: {x: 0.0}, angular: {z: 0.8}}\""
echo ""
echo "  参数说明:"
echo "    linear.x  = 0.0   → 线速度为0（原地不动）"
echo "    angular.z = 0.8   → 绕Z轴角速度 0.8 rad/s ≈ 45°/s"
echo "                        正值=逆时针(左转), 负值=顺时针(右转)"
echo "  ----------------------------------------"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.8}}" &
PUB_PID=$!
sleep 2
kill $PUB_PID 2>/dev/null

# === 演示3.3：右转 ===
echo ""
echo "  ▶ 演示3.3: 小车原地右转 2 秒"
echo "  ----------------------------------------"
echo "  指令: ... angular: {z: -0.8}"
echo "       负值=顺时针旋转（右转）"
echo "  ----------------------------------------"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: -0.8}}" &
PUB_PID=$!
sleep 2
kill $PUB_PID 2>/dev/null

# === 演示3.4：弧线前进（边前进边转弯） ===
echo ""
echo "  ▶ 演示3.4: 弧线运动 3 秒"
echo "  ----------------------------------------"
echo "  同时设置线速度和角速度 → 画弧线"
echo "    linear.x  = 0.3   → 慢速前进"
echo "    angular.z = 0.4   → 同时缓慢左转"
echo "  结果: 小车沿弧线向左前方运动"
echo "  ----------------------------------------"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.4}}" &
PUB_PID=$!
sleep 3
kill $PUB_PID 2>/dev/null

# === 演示3.5：后退 ===
echo ""
echo "  ▶ 演示3.5: 小车后退 2 秒"
echo "  ----------------------------------------"
echo "  线性速度设为负值 = 后退"
echo "    linear.x  = -0.3  → 沿X轴后退，速度 0.3 m/s"
echo "  ----------------------------------------"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: -0.3}, angular: {z: 0.0}}" &
PUB_PID=$!
sleep 2
kill $PUB_PID 2>/dev/null

# === 演示3.6：停止 ===
echo ""
echo "  ▶ 演示3.6: 小车停止"
echo "  ----------------------------------------"
echo "  所有速度为0 = 停止（务必在程序结束时发送）"
echo "  如果不发送停止指令，小车会一直按最后的指令漂移"
echo "  ----------------------------------------"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

# --- 第4步：查看里程计数据 ---
echo ""
echo "=================================="
echo "  [4/5] 查看里程计 (位置反馈)"
echo "=================================="
echo ""
echo "  ros2 topic echo 订阅 /odom 话题，查看小车实时位置:"
echo "  ----------------------------------------"
echo "  里程计消息 nav_msgs/msg/Odometry:"
echo "    pose.pose.position   → 当前坐标 (x, y, z)"
echo "    pose.pose.orientation → 当前朝向 (四元数)"
echo "    twist.twist.linear   → 当前线速度"
echo "    twist.twist.angular  → 当前角速度"
echo "  ----------------------------------------"
# 打印一次里程计数据
ros2 topic echo --once /odom 2>/dev/null || echo "  (里程计桥接需额外配置，可手动运行 ign topic -e -t /model/vehicle_blue/odometry)"

# --- 第5步：额外控制指令说明 ---
echo ""
echo "============================================"
echo "  [5/5] 手动控制指令参考"
echo "============================================"
echo ""
echo "  ■ 在另一个终端进入容器后，可用以下命令手动操控:"
echo ""
echo "  # --- ROS2 方式 ---"
echo ""
echo "  # 持续前进（-r 10 = 每秒发10次）"
echo "  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\"
echo "    \"{linear: {x: 0.3}, angular: {z: 0.0}}\" -r 10"
echo ""
echo "  # 一次性转弯"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \\"
echo "    \"{linear: {x: 0.0}, angular: {z: 1.57}}\""
echo ""
echo "  # --- Gazebo 原生方式（绕开桥接，直接操控）---"
echo ""
echo "  # 控制蓝色小车前进"
echo "  ign topic -t /model/vehicle_blue/cmd_vel \\"
echo "    -m ignition.msgs.Twist \\"
echo "    -p \"linear: {x: 0.5}, angular: {z: 0.0}\""
echo ""
echo "  # 查看蓝色小车里程计"
echo "  ign topic -e -t /model/vehicle_blue/odometry"
echo ""
echo "  # 列出所有 Gazebo 话题"
echo "  ign topic -l"
echo ""
echo "  ■ 仿真将继续运行，关闭 Gazebo 窗口退出"
echo "============================================"

# 等待 Gazebo 进程结束（用户关闭窗口或 Ctrl+C）
wait $GAZEBO_PID 2>/dev/null

# 清理桥接进程
kill $BRIDGE_PID 2>/dev/null
echo ""
echo ">>> 仿真已结束"
