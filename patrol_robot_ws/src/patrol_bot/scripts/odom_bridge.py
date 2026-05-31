#!/usr/bin/env python3
"""
===== 里程计桥接 + TF 广播 =====

订阅 ros_gz_bridge 桥接后的 Gazebo 里程计，将 Gazebo 命名空间帧
转换为标准 ROS2 帧名，发布 /odom 话题和 odom→base_footprint TF。

Gazebo DiffDrive 发布的里程计:
  frame_id:        patrol_bot/odom
  child_frame_id:  patrol_bot/base_link (机器人在 Gazebo 中的 canonical link)

此节点发布:
  /odom:   frame_id=odom, child_frame_id=base_footprint
  TF:      odom → base_footprint
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomBridge(Node):
    def __init__(self):
        super().__init__("odom_bridge")

        self.declare_parameter("input_topic", "/odometry")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_footprint")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.sub = self.create_subscription(
            Odometry, input_topic, self.odom_callback, 10
        )
        self.pub = self.create_publisher(Odometry, output_topic, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info(
            f"OdomBridge: {input_topic} -> {output_topic}"
            f" + TF {self.get_parameter('frame_id').value}"
            f" -> {self.get_parameter('child_frame_id').value}"
        )

    def odom_callback(self, msg: Odometry):
        frame_id = self.get_parameter("frame_id").value
        child_frame_id = self.get_parameter("child_frame_id").value

        # 重新发布里程计消息，修正帧名
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = frame_id
        out.child_frame_id = child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)

        # 广播 TF: odom → base_footprint
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
