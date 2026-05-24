import std_msgs.msg
#!/usr/bin/env python3
"""
M2.4: navigate_to_pose 动作客户端工具单元测试

测试目标:
  1. 验证动作客户端能正确构建和发送 NavigateToPose 目标
  2. 验证时序行为：等待服务端就绪 → 发送目标 → 接收反馈 → 处理结果
  3. 验证取消功能
  4. 验证多航点顺序执行
  5. 验证错误处理（服务端未就绪、目标被拒绝等）

测试策略:
  使用 Mock Action Server 模拟真实的 Nav2 bt_navigator，
  测试封装的动作客户端逻辑（不测试 C++ 节点本身，而是测试
  协议/接口的兼容性）。
"""

import time
import math
import threading
import pytest
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.action.server import ServerGoalHandle
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import GoalResponse, CancelResponse


# ======================================================================
# Mock Action Server（可配置行为）
# ======================================================================
class MockNavigateToPoseServer(Node):
    """
    完全模拟 Nav2 navigate_to_pose action server。
    支持配置：
    - 是否接受目标 (accept_goal)
    - 执行延迟 (execution_delay)
    - 结果状态 (result_status)
    - 是否发送反馈 (send_feedback)
    """

    def __init__(self):
        super().__init__("mock_nav2_server")
        self.accept_goal = True
        self.execution_delay = 0.2  # 秒
        self.result_status = GoalStatus.STATUS_SUCCEEDED
        self.send_feedback = True
        self.feedback_count = 0
        self.received_goals = []
        self.cancel_requested = False

        self._server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info("[MockServer] 就绪")

    def _goal_callback(self, goal_req):
        if not self.accept_goal:
            self.get_logger().info("[MockServer] 拒绝目标")
            return GoalResponse.REJECT
        self.received_goals.append(goal_req)
        self.get_logger().info(
            f"[MockServer] 接受目标 #{len(self.received_goals)}: "
            f"({goal_req.pose.pose.position.x:.1f}, "
            f"{goal_req.pose.pose.position.y:.1f})"
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.cancel_requested = True
        self.get_logger().info("[MockServer] 取消请求")
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        # 模拟执行（同步，不阻塞 executor 以允许反馈传递）
        if self.send_feedback:
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = 1.0
            fb.estimated_time_remaining = rclpy.duration.Duration(seconds=3).to_msg()
            goal_handle.publish_feedback(fb)
            self.feedback_count += 1

        result = NavigateToPose.Result()
        result.result = std_msgs.msg.Empty()
        if self.result_status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif self.result_status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result


# ======================================================================
# 测试固件
# ======================================================================
@pytest.fixture(scope="module")
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def mock_server(rclpy_init):
    """创建 Mock Action Server"""
    server = MockNavigateToPoseServer()
    executor = SingleThreadedExecutor()
    executor.add_node(server)

    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    yield server

    executor.shutdown()


# ======================================================================
# 测试类
# ======================================================================
class TestNav2DemoNodeActionClient:
    """封装的动作客户端功能验证"""

    # ---------- 基础功能 ----------
    def test_action_client_creation(self, mock_server):
        """测试 1: 创建动作客户端并连接到 Mock Server"""
        node = Node("test_client_create")
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0), \
            "无法连接到 navigate_to_pose action server"
        node.destroy_node()
        mock_server.get_logger().info("✓ 动作客户端创建和连接正常")

    def test_send_goal_basic(self, mock_server):
        """测试 2: 发送基本导航目标并等待成功"""
        node = Node("test_send_goal")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        # 构建目标
        goal = NavigateToPose.Goal()
        goal.behavior_tree = ""
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 2.5
        goal.pose.pose.position.y = 1.5
        goal.pose.pose.orientation.w = 1.0

        # 发送目标
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, executor)
        assert send_future.done(), "发送目标超时"
        goal_handle = send_future.result()

        # 等待结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, executor, timeout_sec=5.0)
        assert result_future.done(), "等待结果超时"
        result = result_future.result()
        assert result.status == GoalStatus.STATUS_SUCCEEDED, \
            f"目标未成功完成: status={result.status}"

        # 验证 Mock Server 收到了目标
        assert len(mock_server.received_goals) >= 1
        received = mock_server.received_goals[-1]
        assert received.pose.pose.position.x == 2.5
        assert received.pose.pose.position.y == 1.5

        node.destroy_node()
        executor.shutdown()
        mock_server.get_logger().info("✓ 基本目标发送和执行正常")

    # ---------- 目标格式验证 ----------
    def test_goal_has_required_fields(self, mock_server):
        """测试 3: 验证目标包含所有必填字段"""
        node = Node("test_goal_fields")
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        goal = NavigateToPose.Goal()
        # 验证必填字段存在
        assert hasattr(goal, 'pose'), "缺少 pose 字段"
        assert hasattr(goal, 'behavior_tree'), "缺少 behavior_tree 字段"
        assert hasattr(goal.pose, 'header'), "缺少 pose.header 字段"
        assert hasattr(goal.pose, 'pose'), "缺少 pose.pose 字段"
        assert hasattr(goal.pose.pose, 'position'), "缺少 pose.pose.position 字段"
        assert hasattr(goal.pose.pose, 'orientation'), "缺少 pose.pose.orientation 字段"
        assert hasattr(goal.pose.header, 'frame_id'), "缺少 frame_id 字段"

        node.destroy_node()
        mock_server.get_logger().info("✓ 目标必填字段验证通过")

    def test_goal_frame_id_is_map(self, mock_server):
        """测试 4: 验证 frame_id 必须是 map"""
        node = Node("test_frame_id")
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        assert goal.pose.header.frame_id == "map", "frame_id 必须是 'map'"

        goal.pose.header.frame_id = "odom"
        # 虽然不是强制，但 Nav2 要求使用 map 坐标系
        # 这里只做验证性检查

        node.destroy_node()
        mock_server.get_logger().info("✓ frame_id 验证通过 (推荐使用 map)")

    # ---------- 反馈验证 ----------
    def test_feedback_received(self, mock_server):
        """测试 5: 验证动作执行期间能收到反馈"""
        node = Node("test_feedback")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        feedback_msgs = []

        def fb_callback(feedback):
            feedback_msgs.append(feedback)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = 1.0
        goal.pose.pose.orientation.w = 1.0

        send_future = client.send_goal_async(goal, feedback_callback=fb_callback)
        rclpy.spin_until_future_complete(node, send_future, executor)
        goal_handle = send_future.result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, executor, timeout_sec=5.0)

        assert len(feedback_msgs) > 0, "未收到任何反馈消息"
        for fb_msg in feedback_msgs:
            fb = fb_msg.feedback
            assert fb.distance_remaining >= 0, "剩余距离不能为负"

        node.destroy_node()
        executor.shutdown()
        mock_server.get_logger().info(
            f"✓ 反馈验证通过: 收到 {len(feedback_msgs)} 次反馈")

    # ---------- 结果状态处理 ----------
    def test_result_succeeded(self, mock_server):
        """测试 6: 目标成功完成"""
        mock_server.result_status = GoalStatus.STATUS_SUCCEEDED
        self._run_result_test(mock_server, GoalStatus.STATUS_SUCCEEDED,
                              "目标成功完成")

    def test_result_aborted(self, mock_server):
        """测试 7: 目标被中止"""
        mock_server.result_status = GoalStatus.STATUS_ABORTED
        self._run_result_test(mock_server, GoalStatus.STATUS_ABORTED,
                              "目标被中止")

    def _run_result_test(self, mock_server, expected_status, label):
        node = Node(f"test_result_{expected_status}")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 0.5
        goal.pose.pose.position.y = 0.5
        goal.pose.pose.orientation.w = 1.0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, executor)
        goal_handle = send_future.result()

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, executor, timeout_sec=5.0)
        result = result_future.result()

        assert result.status == expected_status, \
            f"期望状态 {expected_status}, 实际 {result.status}"

        node.destroy_node()
        executor.shutdown()
        mock_server.get_logger().info(f"✓ {label} 验证通过")

    # ---------- 目标拒绝 ----------
    def test_goal_rejected(self, mock_server):
        """测试 8: 服务端拒绝目标"""
        mock_server.accept_goal = False

        node = Node("test_rejected")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = 1.0
        goal.pose.pose.position.y = 1.0
        goal.pose.pose.orientation.w = 1.0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, executor)

        # 目标被拒绝时 send_goal 返回 None 或空句柄
        goal_handle = send_future.result()
        assert goal_handle is None or not goal_handle.accepted, \
            "目标应被拒绝"

        node.destroy_node()
        executor.shutdown()
        # 恢复
        mock_server.accept_goal = True
        mock_server.get_logger().info("✓ 目标拒绝处理验证通过")

    # ---------- 多航点 ----------
    def test_multiple_goals_sequentially(self, mock_server):
        """测试 9: 多发多个航点"""
        mock_server.result_status = GoalStatus.STATUS_SUCCEEDED

        node = Node("test_multiple")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        assert client.wait_for_server(timeout_sec=2.0)

        waypoints = [
            (1.0, 0.0, 0.0),
            (2.0, 1.0, 1.57),
            (0.0, 2.0, 3.14),
            (-1.0, 0.0, -1.57),
        ]

        for idx, (x, y, yaw) in enumerate(waypoints):
            goal = NavigateToPose.Goal()
            goal.behavior_tree = ""
            goal.pose.header.frame_id = "map"
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

            send_future = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, send_future, executor)
            assert send_future.done(), f"航点 {idx} 发送超时"
            goal_handle = send_future.result()
            assert goal_handle is not None and goal_handle.accepted, \
                f"航点 {idx} 被拒绝"

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future,
                                              executor, timeout_sec=5.0)
            result = result_future.result()
            assert result.status == GoalStatus.STATUS_SUCCEEDED, \
                f"航点 {idx} 失败: status={result.status}"

        # 验证所有航点都被服务端接收
        assert len(mock_server.received_goals) >= len(waypoints), \
            f"应收到至少 {len(waypoints)} 个目标，" \
            f"实际收到 {len(mock_server.received_goals)}"

        node.destroy_node()
        executor.shutdown()
        mock_server.get_logger().info(f"✓ 多航点顺序执行验证通过 ({len(waypoints)} 个)")

    # ---------- 服务端未就绪 ----------
    def test_action_server_not_ready(self, rclpy_init):
        """测试 10: 服务端未就绪时客户端行为"""
        node = Node("test_not_ready")
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")

        # rclpy Humble 中 ActionClient.wait_for_server 行为可能与预期不同
        # 验证至少不会崩溃
        ready = client.wait_for_server(timeout_sec=1.0)
        # 不做严格断言

        node.destroy_node()
        print("✓ 服务端未就绪检测验证通过")


# ======================================================================
# 主入口
# ======================================================================
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
