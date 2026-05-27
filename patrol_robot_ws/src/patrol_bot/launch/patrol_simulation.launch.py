"""
===== PatrolBot 仿真启动文件 =====

架构:
  1. Gazebo 仿真世界 — 加载 patrol_world.sdf (含环境 + 机器人模型)
  2. ros_gz_bridge       — ROS ↔ Gazebo 话题桥接
  3. odom_bridge         — 里程计帧名转换 (Gazebo → ROS2 标准)
  4. pointcloud_to_scan  — 点云 → 激光扫描 (Nav2 需要)
  5. robot_state_publisher — URDF → TF 静态变换
  6. 导航栈 + 巡逻节点    — include patrol_bot.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_patrol = get_package_share_directory("patrol_bot")

    world_file = os.path.join(pkg_patrol, "worlds", "patrol_world.sdf")
    urdf_file = os.path.join(pkg_patrol, "models", "patrol_bot", "model.urdf")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    with open(urdf_file, "r") as f:
        robot_desc = f.read()

    robot_description = {"robot_description": robot_desc}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="使用仿真时钟"),

        # === 1. Gazebo 仿真世界 (含机器人模型) ===
        ExecuteProcess(
            cmd=["bash", "-c",
                 f"trap 'kill 0' INT TERM; ign gazebo -r -v 2 {world_file} & "
                 "PID=$!; wait $PID; kill 0 2>/dev/null"],
            output="screen",
            name="gazebo",
        ),

        # === 2. ros_gz_bridge: 桥接所有话题 ===
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            output="screen",
            name="ros_gz_bridge",
        ),

        # === 3. 里程计帧名转换 (Gazebo → ROS2 标准帧名) ===
        Node(
            package="patrol_bot",
            executable="odom_bridge.py",
            output="screen",
            name="odom_bridge",
        ),

        # === 4. 点云 → 激光扫描 ===
        Node(
            package="patrol_bot",
            executable="pointcloud_to_scan.py",
            parameters=[{
                "target_frame": "lidar_link",
                "input_topic": "/scan/points",
            }],
            output="screen",
            name="pointcloud_to_scan",
        ),

        # === 5. 导航栈 + 巡逻业务节点 (含 robot_state_publisher) ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_patrol, "launch", "patrol_bot.launch.py")
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
