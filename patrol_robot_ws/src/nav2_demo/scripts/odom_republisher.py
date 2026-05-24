#!/usr/bin/env python3
"""
===== 里程计重发布器 + TF 广播 =====

Gazebo DiffDrive 插件发布:
  patrol_bot/odom → patrol_bot/chassis (话题)
  patrol_bot/odom → patrol_bot/chassis (TF)

此节点:
  1. 重发 Odometry: /raw_odom → /odom, 帧名 odom → base_footprint
  2. 广播 TF: static odom → base_footprint 变换
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomRepublisher(Node):
    def __init__(self):
        super().__init__("odom_republisher")

        self.sub = self.create_subscription(
            Odometry,
            "/raw_odom",    # 桥接后的原始里程计话题
            self.odom_callback,
            10,
        )
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            "OdomRepublisher 启动: /raw_odom → /odom + TF odom→base_footprint"
        )

    def odom_callback(self, msg: Odometry):
        # 1. 重发布 Odometry
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = "odom"
        out.child_frame_id = "base_footprint"
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)

        # 2. 广播 TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
