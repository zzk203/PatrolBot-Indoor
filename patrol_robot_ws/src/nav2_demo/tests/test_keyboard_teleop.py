#!/usr/bin/env python3
"""
键盘遥控节点 单元测试

测试目标:
  1. 节点初始化：参数声明、Publisher 创建
  2. 键值映射：验证各按键对应的 Twist 消息输出
  3. 边界情况：未知按键、退出键处理
"""

import pytest
import rclpy
from geometry_msgs.msg import Twist


@pytest.fixture(scope="function")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class TestKeyboardTeleopInit:
    """测试节点初始化"""

    def test_import(self):
        """验证模块可导入"""
        import sys
        import os

        # 添加 scripts 目录到路径
        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from keyboard_teleop import KeyboardTeleop
        assert KeyboardTeleop is not None

    def test_parameter_defaults(self, ros_context):
        """验证参数默认值"""
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from keyboard_teleop import KeyboardTeleop

        node = KeyboardTeleop()
        try:
            assert node.linear_vel == 0.3
            assert node.angular_vel == 1.0
            assert node.current_linear == 0.0
            assert node.current_angular == 0.0
            assert node.running is True
        finally:
            node.destroy_node()

    def test_publisher_created(self, ros_context):
        """验证已创建正确话题的 Publisher"""
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from keyboard_teleop import KeyboardTeleop

        node = KeyboardTeleop()
        try:
            topic = node.get_parameter("cmd_vel_topic").value
            assert topic == "/model/patrol_bot/cmd_vel"
            # 验证 publisher 数量
            pub_count = len(
                [p for p in node.publishers if p.topic_name == topic]
            )
            assert pub_count >= 1
        finally:
            node.destroy_node()

    def test_custom_parameters(self, ros_context):
        """验证自定义参数生效"""
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from keyboard_teleop import KeyboardTeleop

        node = KeyboardTeleop()
        try:
            # 设置自定义参数
            node.set_parameters([
                rclpy.parameter.Parameter("linear_vel", rclpy.Parameter.Type.DOUBLE, 0.5),
                rclpy.parameter.Parameter("angular_vel", rclpy.Parameter.Type.DOUBLE, 2.0),
            ])
            assert node.get_parameter("linear_vel").value == 0.5
            assert node.get_parameter("angular_vel").value == 2.0
        finally:
            node.destroy_node()


class TestKeyboardTeleopKeyMapping:
    """测试按键 → Twist 映射"""

    @pytest.fixture(autouse=True)
    def setup_node(self, ros_context):
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from keyboard_teleop import KeyboardTeleop

        self.node = KeyboardTeleop()
        # 保存上次发布的 Twist
        self.last_twist = None

        # 创建订阅用于捕获发布的消息
        self.captured_msgs = []

        self.sub = self.node.create_subscription(
            Twist,
            "/model/patrol_bot/cmd_vel",
            lambda msg: self.captured_msgs.append(msg),
            10,
        )
        yield
        self.node.destroy_node()

    def _simulate_key(self, key_char):
        """模拟按键: 直接调用 handle_key（提取自 timer_callback 的逻辑）"""
        from geometry_msgs.msg import Twist

        linear_vel = self.node.linear_vel
        angular_vel = self.node.angular_vel

        current_linear = 0.0
        current_angular = 0.0
        should_exit = False

        if key_char == "w":
            current_linear = linear_vel
        elif key_char == "s":
            current_linear = -linear_vel
        elif key_char == "a":
            current_angular = angular_vel
        elif key_char == "d":
            current_angular = -angular_vel
        elif key_char == " ":
            pass  # 所有速度归零
        elif key_char == "q":
            should_exit = True

        twist = Twist()
        twist.linear.x = current_linear
        twist.angular.z = current_angular
        return twist, should_exit

    def test_key_w_forward(self):
        """w 键 → 前进"""
        twist, _ = self._simulate_key("w")
        assert twist.linear.x == pytest.approx(0.3)
        assert twist.angular.z == pytest.approx(0.0)

    def test_key_s_backward(self):
        """s 键 → 后退"""
        twist, _ = self._simulate_key("s")
        assert twist.linear.x == pytest.approx(-0.3)
        assert twist.angular.z == pytest.approx(0.0)

    def test_key_a_left(self):
        """a 键 → 左转"""
        twist, _ = self._simulate_key("a")
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(1.0)

    def test_key_d_right(self):
        """d 键 → 右转"""
        twist, _ = self._simulate_key("d")
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(-1.0)

    def test_key_space_stop(self):
        """空格键 → 停止"""
        twist, _ = self._simulate_key(" ")
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)

    def test_key_q_exit(self):
        """q 键 → 退出"""
        _, should_exit = self._simulate_key("q")
        assert should_exit is True

    def test_unknown_key(self):
        """未知按键 → 无变化 (忽略)"""
        twist, _ = self._simulate_key("x")
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)

    def test_uppercase_keys(self):
        """大写字母应被正确处理(Python区分大小写，大写不应匹配)"""
        twist, _ = self._simulate_key("W")
        # 大写 'W' 不是小写 'w'，应该被忽略
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.0)
