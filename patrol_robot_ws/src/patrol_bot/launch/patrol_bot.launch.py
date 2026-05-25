import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('patrol_bot')
    config_path = os.path.join(pkg_dir, 'config', 'patrol_config.yaml')

    patrol_node = Node(
        package='patrol_bot',
        executable='patrol_bot_node',
        name='patrol_bot_node',
        output='screen',
        parameters=[{'config_path': config_path}],
    )

    return LaunchDescription([patrol_node])
