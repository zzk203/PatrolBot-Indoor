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

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="仿真时钟"),

        # === 1. Gazebo 仿真世界 ===
        ExecuteProcess(
            cmd=["bash", "-c",
                 f"trap 'kill 0' INT TERM; ign gazebo -r -v 2 {world_file} & "
                 "PID=$!; wait $PID; kill 0 2>/dev/null"],
            output="screen",
            name="gazebo",
        ),

        # === 2. ros_gz_bridge 桥接 ===
        # cmd_vel: ROS /cmd_vel → Gazebo /model/patrol_bot/cmd_vel
        # odometry: Gazebo → ROS /model/patrol_bot/odometry
        # scan_cloud: Gazebo /scan → ROS /scan_cloud (PointCloud2)
        # clock: Gazebo → ROS /clock
        # camera: Gazebo → ROS /camera/image_raw
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/scan_cloud@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            ],
            output="screen",
            name="ros_gz_bridge",
        ),

        # === 3. 里程计帧名转换（仿真特有） ===
        Node(
            package="patrol_bot", executable="odom_republisher.py",
            output="screen",
            name="odom_republisher",
        ),

        # === 4. 点云→激光扫描（仿真特有） ===
        Node(
            package="patrol_bot", executable="pointcloud_to_scan.py",
            parameters=[{"target_frame": "lidar_link"}],
            output="screen",
            name="pointcloud_to_scan",
        ),

        # === 5. TF: map → odom（AMCL 初始引导，仿真特有） ===
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--x", "-2.0", "--y", "-2.0", "--yaw", "0.0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            output="screen",
            name="map_to_odom_tf",
        ),

        # === 6. 基础导航栈 + 巡逻节点 ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_patrol, "launch", "patrol_bot.launch.py")
            ),
            launch_arguments={"use_sim_time": "true"}.items(),
        ),
    ])
