import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav2_demo = get_package_share_directory("nav2_demo")

    world_file = os.path.join(pkg_nav2_demo, "worlds", "patrol_world.sdf")
    map_file = os.path.join(pkg_nav2_demo, "maps", "office.yaml")
    urdf_file = os.path.join(pkg_nav2_demo, "models", "patrol_bot", "model.urdf")
    params_file = os.path.join(pkg_nav2_demo, "config", "nav2_params.yaml")
    bridge_config = os.path.join(pkg_nav2_demo, "config", "gz_bridge.yaml")
    rviz_config = os.path.join(pkg_nav2_demo, "rviz", "nav2_view.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    robot_description = {"robot_description": open(urdf_file).read()}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="仿真时钟"),

        # === 1. Gazebo 仿真世界 ===
        # trap + wait 确保 Ctrl+C 时同步终止 Gazebo 服务器和 GUI
        ExecuteProcess(
            cmd=["bash", "-c",
                 f"trap 'kill 0' INT TERM; ign gazebo -r -v 2 {world_file} & "
                 "PID=$!; wait $PID; kill 0 2>/dev/null"],
            output="screen",
            name="gazebo",
        ),

        # === 2. ros_gz_bridge 桥接（inline 格式，不用 remapping） ===
        # 格式: /Gazebo话题@ROS类型[Gazebo类型  ([=GZ→ROS, ]=ROS→GZ)
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=[
                # cmd_vel: ROS2 → Gazebo
                "/model/patrol_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                # odometry: Gazebo → ROS2 (保留 Gazebo 原生话题名)
                "/model/patrol_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                # LiDAR pointcloud: Gazebo → ROS2
                "/scan@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                # clock: Gazebo → ROS2
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
            output="screen",
            name="ros_gz_bridge",
        ),

        # === 3. 里程计帧名转换 ===
        Node(
            package="nav2_demo", executable="odom_republisher.py",
            output="screen",
            name="odom_republisher",
        ),

        # === 4. TF 静态变换 (base_footprint→base_link→lidar_link) ===
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
            output="screen",
            name="robot_state_publisher",
        ),

        # === 5. 点云→激光扫描 ===
        Node(
            package="nav2_demo", executable="pointcloud_to_scan.py",
            parameters=[{"target_frame": "lidar_link"}],
            output="screen",
            name="pointcloud_to_scan",
        ),

        # === 6. 静态地图发布器（替代 lifecycle map_server） ===
        # map_server 是 lifecycle 节点，需要 lifecycle_manager 管理，
        # 直接用 Python 脚本发布，简化时序依赖
        Node(
            package="nav2_demo", executable="static_map_publisher.py",
            parameters=[{
                "yaml_filename": map_file,
                "use_sim_time": use_sim_time,
            }],
            output="screen",
            name="static_map_publisher",
        ),

        # === 7. TF: map → odom（初始静态变换） ===
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--x", "-2.0", "--y", "-2.0", "--yaw", "0.0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            output="screen",
            name="map_to_odom_tf",
        ),

        # === 8. AMCL 定位 ===
        Node(
            package="nav2_amcl", executable="amcl",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            output="screen",
            name="amcl",
        ),

        # === 9. Nav2 导航栈 + 生命周期管理 ===
        # 全局规划器（内部创建 global_costmap）
        Node(
            package="nav2_planner", executable="planner_server",
            parameters=[params_file],
            output="screen",
            name="planner_server",
        ),
        # 局部控制器（内部创建 local_costmap）
        Node(
            package="nav2_controller", executable="controller_server",
            parameters=[params_file],
            output="screen",
            name="controller_server",
        ),
        # 行为树导航器
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            parameters=[params_file],
            output="screen",
            name="bt_navigator",
        ),
        # 生命周期管理器
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                ],
            }],
            output="screen",
            name="lifecycle_manager_navigation",
        ),

        # === 10. RViz2 可视化 ===
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
            name="rviz2",
        ),
    ])
