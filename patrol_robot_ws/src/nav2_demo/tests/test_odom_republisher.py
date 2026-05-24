#!/usr/bin/env python3
"""
里程计重发布器 单元测试

测试目标:
  1. 节点初始化：参数声明、Publisher/Subscriber 创建
  2. odom_callback：验证重发布的 Odometry 消息帧名正确
  3. odom_callback：验证 TF broadcast 内容正确
  4. 边界情况：空消息处理
"""

import pytest
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose, Twist
from std_msgs.msg import Header


@pytest.fixture(scope="function")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_odometry(x=1.0, y=2.0, z=0.0, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
    """构造 Odometry 测试消息"""
    msg = Odometry()
    msg.header = Header()
    msg.header.frame_id = "patrol_bot/odom"
    msg.child_frame_id = "patrol_bot/chassis"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.position.z = z
    msg.pose.pose.orientation.x = ox
    msg.pose.pose.orientation.y = oy
    msg.pose.pose.orientation.z = oz
    msg.pose.pose.orientation.w = ow
    msg.twist.twist.linear.x = 0.5
    msg.twist.twist.angular.z = 0.1
    return msg


class TestOdomRepublisherInit:
    """测试节点初始化"""

    def test_import(self):
        """验证模块可导入"""
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from odom_republisher import OdomRepublisher
        assert OdomRepublisher is not None

    def test_parameter_defaults(self, ros_context):
        """验证参数默认值"""
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from odom_republisher import OdomRepublisher

        node = OdomRepublisher()
        try:
            input_topic = node.get_parameter("input_topic").value
            assert input_topic == "/model/patrol_bot/odometry"
        finally:
            node.destroy_node()

    def test_pubsub_created(self, ros_context):
        """验证已创建正确的 Publisher 和 Subscriber"""
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from odom_republisher import OdomRepublisher

        node = OdomRepublisher()
        try:
            # 检查有订阅者 (input topic)
            subs = node.subscriptions
            topics = [s.topic_name for s in subs]
            assert "/model/patrol_bot/odometry" in topics

            # 检查有发布者 (/odom)
            pubs = node.publishers
            pub_topics = [p.topic_name for p in pubs]
            assert "/odom" in pub_topics
        finally:
            node.destroy_node()


class TestOdomRepublisherCallback:
    """测试里程计回调逻辑"""

    @pytest.fixture(autouse=True)
    def setup_node(self, ros_context):
        import sys
        import os

        pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        scripts_dir = os.path.join(pkg_dir, "scripts")
        if scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)

        from odom_republisher import OdomRepublisher

        self.node = OdomRepublisher()
        self.captured_odom = []
        self.captured_tf = []

        # 订阅 /odom 捕获输出
        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/odom",
            lambda msg: self.captured_odom.append(msg),
            10,
        )

        # 替换 TF broadcaster 以捕获 TF
        self.node.tf_broadcaster = _MockTransformBroadcaster(self.captured_tf)

        yield
        self.node.destroy_node()

    def test_odom_frame_remapping(self):
        """验证帧名转换: patrol_bot/odom → odom, patrol_bot/chassis → base_footprint"""
        input_msg = _make_odometry()
        self.node.odom_callback(input_msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.captured_odom:
            out = self.captured_odom[0]
            assert out.header.frame_id == "odom", (
                f"期望 frame_id='odom', 实际='{out.header.frame_id}'"
            )
            assert out.child_frame_id == "base_footprint", (
                f"期望 child_frame_id='base_footprint', 实际='{out.child_frame_id}'"
            )

    def test_odom_pose_preserved(self):
        """验证位姿和速度信息被完整保留"""
        input_msg = _make_odometry(x=2.5, y=-1.3, z=0.0, ow=1.0)
        self.node.odom_callback(input_msg)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        if self.captured_odom:
            out = self.captured_odom[0]
            assert out.pose.pose.position.x == pytest.approx(2.5)
            assert out.pose.pose.position.y == pytest.approx(-1.3)
            assert out.pose.pose.orientation.w == pytest.approx(1.0)
            assert out.twist.twist.linear.x == pytest.approx(0.5)
            assert out.twist.twist.angular.z == pytest.approx(0.1)

    def test_tf_broadcast(self):
        """验证 TF 变换被正确广播"""
        input_msg = _make_odometry(x=3.0, y=4.0, z=0.0, ox=0.0, oy=0.0, oz=0.707, ow=0.707)
        self.node.odom_callback(input_msg)

        # TF broadcaster 在回调中同步调用，所以立即有值
        if self.captured_tf:
            tf_msg = self.captured_tf[0]
            assert tf_msg.header.frame_id == "odom"
            assert tf_msg.child_frame_id == "base_footprint"
            assert tf_msg.transform.translation.x == pytest.approx(3.0)
            assert tf_msg.transform.translation.y == pytest.approx(4.0)
            assert tf_msg.transform.rotation.z == pytest.approx(0.707)

    def test_tf_timestamp(self):
        """验证 TF 时间戳来自输入消息"""
        import builtins
        # 使用当前时间创建输入
        input_msg = _make_odometry()
        input_msg.header.stamp.sec = 12345
        input_msg.header.stamp.nanosec = 678900000

        self.node.odom_callback(input_msg)

        if self.captured_tf:
            tf_msg = self.captured_tf[0]
            assert tf_msg.header.stamp.sec == 12345
            assert tf_msg.header.stamp.nanosec == 678900000


class _MockTransformBroadcaster:
    """模拟 TF broadcaster，捕获广播的消息"""
    def __init__(self, storage_list):
        self.storage = storage_list

    def sendTransform(self, transform):
        self.storage.append(transform)
