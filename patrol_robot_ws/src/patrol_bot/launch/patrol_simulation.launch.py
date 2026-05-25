import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('patrol_bot')
    config_path = os.path.join(pkg_dir, 'config', 'patrol_config.yaml')

    # ============================================================
    # TODO: 集成 Gazebo 仿真环境
    #   例如：
    #     gazebo_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory('gazebo_ros'),
    #             '/launch/gazebo.launch.py'
    #         ]),
    #         launch_arguments={
    #             'world': os.path.join(pkg_dir, 'worlds', 'patrol_world.world'),
    #             'verbose': 'true'
    #         }.items()
    #     )
    #
    # TODO: 集成 Nav2 bringup
    #   例如：
    #     nav2_launch = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             get_package_share_directory('nav2_bringup'),
    #             '/launch/bringup_launch.py'
    #         ]),
    #         launch_arguments={
    #             'use_sim_time': 'true',
    #             'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
    #             'map': os.path.join(pkg_dir, 'maps', 'patrol_map.yaml')
    #         }.items()
    #     )
    #
    # TODO: 生成机器人 spawn 实体（使用 robot_state_publisher + spawn_entity）
    # ============================================================

    patrol_node = Node(
        package='patrol_bot',
        executable='patrol_bot_node',
        name='patrol_bot_node',
        output='screen',
        parameters=[{'config_path': config_path}],
    )

    # 组装 launch 描述
    # 当集成 Gazebo + Nav2 后，将相应 action 加入列表
    return LaunchDescription([
        # gazebo_launch,    # 取消注释以启用 Gazebo
        # nav2_launch,      # 取消注释以启用 Nav2
        patrol_node,
    ])
