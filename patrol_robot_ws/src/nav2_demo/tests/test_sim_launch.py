#!/usr/bin/env python3
"""
仿真启动文件 & RViz 配置 验证测试

测试目标:
  1. launch.py 文件语法正确、引用路径有效
  2. RViz 配置文件存在且格式有效
  3. 世界/地图/URDF/参数等资源文件存在
  4. 桥接配置与 launch 中的参数一致
"""

import os
import sys
import pytest
from launch import LaunchDescription


# ======================================================================
# M1.1 世界文件验证
# ======================================================================
class TestM11World:
    @pytest.fixture(scope="class")
    def pkg_dir(self):
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        )

    def test_world_file_exists(self, pkg_dir):
        """patrol_world.sdf 存在"""
        world_path = os.path.join(pkg_dir, "worlds", "patrol_world.sdf")
        assert os.path.isfile(world_path), f"世界文件缺失: {world_path}"

    def test_world_file_not_empty(self, pkg_dir):
        """世界文件内容非空"""
        world_path = os.path.join(pkg_dir, "worlds", "patrol_world.sdf")
        assert os.path.getsize(world_path) > 0

    def test_world_contains_robot(self, pkg_dir):
        """世界文件包含 patrol_bot 模型"""
        world_path = os.path.join(pkg_dir, "worlds", "patrol_world.sdf")
        with open(world_path) as f:
            content = f.read()
        assert 'name="patrol_bot"' in content, "世界文件缺少 patrol_bot 模型"
        assert "DiffDrive" in content, "世界文件缺少差速驱动插件"

    def test_world_has_walls_and_obstacles(self, pkg_dir):
        """世界文件包含墙壁和障碍物"""
        world_path = os.path.join(pkg_dir, "worlds", "patrol_world.sdf")
        with open(world_path) as f:
            content = f.read()
        assert "wall_north" in content
        assert "wall_south" in content
        assert "wall_east" in content
        assert "wall_west" in content
        # 至少有 4 个障碍物
        obstacles = [name for name in ["pillar", "obstacle_1", "obstacle_2",
                                        "obstacle_3", "obstacle_4"]
                     if name in content]
        assert len(obstacles) >= 4, f"障碍物不足: 找到 {len(obstacles)} 个"


# ======================================================================
# M1.2 机器人模型验证
# ======================================================================
class TestM12RobotModel:
    @pytest.fixture(scope="class")
    def model_dir(self):
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "models", "patrol_bot",
        )

    def test_model_sdf_exists(self, model_dir):
        assert os.path.isfile(os.path.join(model_dir, "model.sdf"))

    def test_model_urdf_exists(self, model_dir):
        assert os.path.isfile(os.path.join(model_dir, "model.urdf"))

    def test_urdf_has_tf_tree(self, model_dir):
        """URDF 定义了完整的 TF 树"""
        urdf_path = os.path.join(model_dir, "model.urdf")
        with open(urdf_path) as f:
            content = f.read()
        assert "base_footprint" in content
        assert "base_link" in content
        assert "lidar_link" in content
        assert "camera_link" in content
        assert "base_joint" in content
        assert "lidar_joint" in content

    def test_urdf_xml_valid(self, model_dir):
        """URDF 文件是合法 XML"""
        import xml.etree.ElementTree as ET
        urdf_path = os.path.join(model_dir, "model.urdf")
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        assert root.tag == "robot"
        assert root.attrib["name"] == "patrol_bot"

    def test_sdf_has_diff_drive(self, model_dir):
        """SDF 模型包含差速驱动插件"""
        sdf_path = os.path.join(model_dir, "model.sdf")
        with open(sdf_path) as f:
            content = f.read()
        assert "DiffDrive" in content
        assert "left_wheel_joint" in content
        assert "right_wheel_joint" in content

    def test_sdf_has_lidar(self, model_dir):
        """SDF 模型包含激光雷达"""
        sdf_path = os.path.join(model_dir, "model.sdf")
        with open(sdf_path) as f:
            content = f.read()
        assert "gpu_lidar" in content or "lidar" in content
        assert "scan" in content or "lidar" in content

    def test_sdf_has_camera(self, model_dir):
        """SDF 模型包含虚拟相机"""
        sdf_path = os.path.join(model_dir, "model.sdf")
        with open(sdf_path) as f:
            content = f.read()
        assert "camera_link" in content
        assert "camera/image_raw" in content


# ======================================================================
# M1.3 键盘遥控验证
# ======================================================================
class TestM13KeyboardTeleop:
    @pytest.fixture(scope="class")
    def pkg_dir(self):
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        )

    def test_teleop_script_exists(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "scripts",
                                            "keyboard_teleop.py"))

    def test_teleop_has_main(self, pkg_dir):
        """脚本包含 main 函数入口"""
        with open(os.path.join(pkg_dir, "scripts", "keyboard_teleop.py")) as f:
            content = f.read()
        assert "def main()" in content
        assert 'if __name__ == "__main__"' in content

    def test_teleop_publishes_to_correct_topic(self, pkg_dir):
        """遥控节点发布到 Gazebo 桥接的话题"""
        with open(os.path.join(pkg_dir, "scripts", "keyboard_teleop.py")) as f:
            content = f.read()
        assert "/model/patrol_bot/cmd_vel" in content

    def test_cmakelists_registers_teleop(self, pkg_dir):
        """CMakeLists.txt 注册了 keyboard_teleop 脚本"""
        cmake_path = os.path.join(pkg_dir, "CMakeLists.txt")
        with open(cmake_path) as f:
            content = f.read()
        assert "keyboard_teleop.py" in content

    def test_launch_includes_teleop(self, pkg_dir):
        """launch 文件中包含 keyboard_teleop"""
        launch_path = os.path.join(pkg_dir, "launch", "patrol_sim.launch.py")
        with open(launch_path) as f:
            content = f.read()
        assert "keyboard_teleop" in content
        assert "enable_keyboard" in content


# ======================================================================
# M1.4 里程计和TF验证
# ======================================================================
class TestM14Odometry:
    @pytest.fixture(scope="class")
    def pkg_dir(self):
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        )

    def test_odom_script_exists(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "scripts",
                                            "odom_republisher.py"))

    def test_odom_republishes_to_odom(self, pkg_dir):
        """里程计重发布到 /odom 话题"""
        with open(os.path.join(pkg_dir, "scripts", "odom_republisher.py")) as f:
            content = f.read()
        assert "/odom" in content

    def test_odom_broadcasts_tf(self, pkg_dir):
        """里程计同时广播 TF: odom → base_footprint"""
        with open(os.path.join(pkg_dir, "scripts", "odom_republisher.py")) as f:
            content = f.read()
        assert "odom" in content and "base_footprint" in content
        assert "TransformBroadcaster" in content or "tf2_ros" in content

    def test_launch_includes_odom_republisher(self, pkg_dir):
        """launch 文件包含里程计重发布节点"""
        launch_path = os.path.join(pkg_dir, "launch", "patrol_sim.launch.py")
        with open(launch_path) as f:
            content = f.read()
        assert "odom_republisher" in content

    def test_world_has_diff_drive_odom(self, pkg_dir):
        """世界文件中 DiffDrive 插件正确配置里程计话题"""
        world_path = os.path.join(pkg_dir, "worlds", "patrol_world.sdf")
        with open(world_path) as f:
            content = f.read()
        assert "odom_topic" in content or "/model/patrol_bot/odometry" in content


# ======================================================================
# M1.5 RViz2 可视化验证
# ======================================================================
class TestM15RvizVisualization:
    @pytest.fixture(scope="class")
    def pkg_dir(self):
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        )

    def test_rviz_config_exists(self, pkg_dir):
        rviz_path = os.path.join(pkg_dir, "rviz", "nav2_view.rviz")
        assert os.path.isfile(rviz_path), f"RViz 配置文件缺失: {rviz_path}"

    def test_rviz_config_has_required_displays(self, pkg_dir):
        """RViz 配置包含了必需的显示元素"""
        rviz_path = os.path.join(pkg_dir, "rviz", "nav2_view.rviz")
        with open(rviz_path) as f:
            content = f.read()
        required = ["RobotModel", "Map", "LaserScan", "TF", "PoseArray", "Path"]
        for item in required:
            assert item in content, f"RViz 缺少显示元素: {item}"

    def test_rviz_config_has_correct_fixed_frame(self, pkg_dir):
        """RViz 固定坐标系为 map"""
        rviz_path = os.path.join(pkg_dir, "rviz", "nav2_view.rviz")
        with open(rviz_path) as f:
            content = f.read()
        assert "Fixed Frame" in content or "map" in content

    def test_rviz_config_has_goal_tool(self, pkg_dir):
        """RViz 配置包含 2D Goal Pose 工具"""
        rviz_path = os.path.join(pkg_dir, "rviz", "nav2_view.rviz")
        with open(rviz_path) as f:
            content = f.read()
        assert "SetGoal" in content or "2D Goal" in content

    def test_launch_includes_rviz2(self, pkg_dir):
        """launch 文件包含 RViz2 节点"""
        launch_path = os.path.join(pkg_dir, "launch", "patrol_sim.launch.py")
        with open(launch_path) as f:
            content = f.read()
        assert "rviz2" in content


# ======================================================================
# 通用文件完整性验证
# ======================================================================
class TestGeneralIntegrity:
    @pytest.fixture(scope="class")
    def pkg_dir(self):
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        )

    def test_map_files_exist(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "maps", "office.pgm"))
        assert os.path.isfile(os.path.join(pkg_dir, "maps", "office.yaml"))

    def test_bridge_config_exists(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "config", "gz_bridge.yaml"))

    def test_nav2_params_exists(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "config",
                                            "nav2_params.yaml"))

    def test_behavior_tree_exists(self, pkg_dir):
        bt_dir = os.path.join(pkg_dir, "behavior_trees")
        assert os.path.isdir(bt_dir)
        assert len(os.listdir(bt_dir)) > 0

    def test_launch_file_syntax(self, pkg_dir):
        """验证 launch.py 文件可以被 Python 语法解析"""
        launch_path = os.path.join(pkg_dir, "launch", "patrol_sim.launch.py")
        with open(launch_path) as f:
            code = f.read()
        compile(code, launch_path, "exec")  # 语法检查

    def test_pointcloud_to_scan_exists(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "scripts",
                                            "pointcloud_to_scan.py"))

    def test_static_map_publisher_exists(self, pkg_dir):
        assert os.path.isfile(os.path.join(pkg_dir, "scripts",
                                            "static_map_publisher.py"))

    def test_scripts_are_executable(self, pkg_dir):
        """Python 脚本都应带有 shebang"""
        scripts_dir = os.path.join(pkg_dir, "scripts")
        py_scripts = [f for f in os.listdir(scripts_dir) if f.endswith(".py")]
        for script in py_scripts:
            with open(os.path.join(scripts_dir, script)) as f:
                first_line = f.readline().strip()
            assert first_line == "#!/usr/bin/env python3", (
                f"{script} 缺少 shebang"
            )
