import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("patrol_bot")

    config_path = os.path.join(pkg_dir, "config", "patrol_config.yaml")
    params_file = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    map_file = os.path.join(pkg_dir, "maps", "office.yaml")
    urdf_file = os.path.join(pkg_dir, "models", "patrol_bot", "model.urdf")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    robot_description = {"robot_description": open(urdf_file).read()}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="仿真时钟"),

        # === 1. 地图服务 ===
        Node(
            package="nav2_map_server", executable="map_server",
            parameters=[params_file, {"yaml_filename": map_file,
                                       "use_sim_time": use_sim_time}],
            output="screen",
            name="map_server",
        ),

        # === 2. TF 静态变换 (base_footprint→base_link→lidar_link) ===
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
            output="screen",
            name="robot_state_publisher",
        ),

        # === 3. AMCL 定位 ===
        Node(
            package="nav2_amcl", executable="amcl",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            output="screen",
            name="amcl",
        ),

        # === 4. Nav2 导航栈 + 生命周期管理 ===
        Node(
            package="nav2_planner", executable="planner_server",
            parameters=[params_file],
            output="screen",
            name="planner_server",
        ),
        Node(
            package="nav2_controller", executable="controller_server",
            parameters=[params_file],
            output="screen",
            name="controller_server",
        ),
        Node(
            package="nav2_behaviors", executable="behavior_server",
            parameters=[params_file],
            output="screen",
            name="behavior_server",
        ),
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            parameters=[params_file],
            output="screen",
            name="bt_navigator",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "map_server",
                    "amcl",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }],
            output="screen",
            name="lifecycle_manager_navigation",
        ),

        # === 5. patrol_bot_node（巡逻业务节点） ===
        Node(
            package="patrol_bot",
            executable="patrol_bot_node",
            name="patrol_bot_node",
            output="screen",
            parameters=[{"config_path": config_path}],
        ),
    ])
