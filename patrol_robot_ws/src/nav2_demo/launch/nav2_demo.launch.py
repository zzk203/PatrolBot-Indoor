import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_demo_dir = get_package_share_directory("nav2_demo")
    params_file = os.path.join(nav2_demo_dir, "config", "nav2_params.yaml")
    bt_xml_file = os.path.join(
        nav2_demo_dir, "behavior_trees", "navigate_w_replanning.xml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="使用仿真时间"),

        # AMCL 定位节点
        Node(
            package="nav2_amcl", executable="amcl",
            name="amcl",
            output="screen",
            parameters=[params_file],
        ),

        # 全局代价地图
        Node(
            package="nav2_costmap_2d", executable="nav2_costmap_2d",
            name="global_costmap",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        # 局部代价地图
        Node(
            package="nav2_costmap_2d", executable="nav2_costmap_2d",
            name="local_costmap",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        # 全局规划器 (A*)
        Node(
            package="nav2_planner", executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),

        # 局部控制器 (Regulated Pure Pursuit)
        Node(
            package="nav2_controller", executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
        ),

        # 行为树导航器
        Node(
            package="nav2_bt_navigator", executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {
                "default_bt_xml_filename": bt_xml_file,
            }],
        ),

        # 航点跟随器
        Node(
            package="nav2_waypoint_follower", executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[params_file],
        ),

        # 生命周期管理器
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "bt_navigator",
                    "waypoint_follower",
                ],
            }],
        ),
    ])
