#!/usr/bin/env python3
"""
M2.3: 单点导航测试执行脚本

功能:
  1. 连接到 Nav2 navigate_to_pose action server
  2. 发送单个导航目标
  3. 监控执行过程（反馈、状态）
  4. 记录执行结果

用法:
  ros2 run nav2_demo nav2_single_goal_test.py
  ros2 run nav2_demo nav2_single_goal_test.py --ros-args -p x:=2.0 -p y:=1.5 -p yaw:=1.57
"""

import sys
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


class SingleGoalTest(Node):
    """单点导航测试节点"""

    def __init__(self):
        super().__init__("single_goal_test")

        # 声明参数
        self.declare_parameter("x", 1.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("timeout", 60.0)
        self.declare_parameter("use_sim_time", True)

        self.x = self.get_parameter("x").value
        self.y = self.get_parameter("y").value
        self.yaw = self.get_parameter("yaw").value
        self.timeout = self.get_parameter("timeout").value

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._result_future = None
        self._feedback_msgs = []

        self.get_logger().info(
            f"单点导航测试: 目标 ({self.x:.2f}, {self.y:.2f}, θ={self.yaw:.2f})")

    def run(self):
        """执行导航测试"""
        # 等待 Action Server
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ navigate_to_pose action server 未就绪")
            return False

        # 构建目标
        goal = NavigateToPose.Goal()
        goal.behavior_tree = ""
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.x
        goal.pose.pose.position.y = self.y
        goal.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        self.get_logger().info(f"🚀 发送导航目标...")
        start_time = time.time()

        # 发送目标
        send_goal_future = self._client.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("❌ 目标被拒绝")
            return False

        self.get_logger().info("✅ 目标被接受，等待执行...")

        # 等待结果
        self._result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._result_future,
                                         timeout_sec=self.timeout)

        if not self._result_future.done():
            self.get_logger().error(f"❌ 执行超时（{self.timeout}s）")
            # 取消目标
            goal_handle.cancel_goal_async()
            return False

        result = self._result_future.result()
        elapsed = time.time() - start_time

        # 输出结果
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"✅ 导航成功! 耗时: {elapsed:.1f}s, "
                f"反馈次数: {len(self._feedback_msgs)}")
            return True
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(f"⚠️ 导航被取消（{elapsed:.1f}s）")
            return False
        else:
            self.get_logger().error(
                f"❌ 导航失败: status={result.status}, "
                f"error_code={result.result.error_code}")
            return False

    def _feedback_callback(self, feedback_msg):
        """反馈回调"""
        fb = feedback_msg.feedback
        self._feedback_msgs.append(fb)
        self.get_logger().info(
            f"  📊 剩余距离: {fb.distance_remaining:.2f}m")

    def cancel(self):
        """取消当前目标"""
        if self._result_future and not self._result_future.done():
            self._result_future.cancel()
            self.get_logger().warning("⚠️ 取消目标")


def main():
    rclpy.init()
    test = SingleGoalTest()
    success = test.run()
    test.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
