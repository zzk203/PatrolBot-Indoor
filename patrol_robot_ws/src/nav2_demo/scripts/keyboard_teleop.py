#!/usr/bin/env python3
"""
键盘遥控节点：通过终端键盘控制 PatrolBot 差速移动。

按键映射:
  w / s : 前进 / 后退
  a / d : 左转 / 右转
  空格   : 急停
  q     : 退出

速度可通过 ROS 参数调节:
  linear_vel  (默认 0.3 m/s)
  angular_vel (默认 1.0 rad/s)
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")

        self.declare_parameter("linear_vel", 0.3)
        self.declare_parameter("angular_vel", 1.0)
        self.declare_parameter("cmd_vel_topic", "/model/patrol_bot/cmd_vel")

        self.linear_vel = self.get_parameter("linear_vel").value
        self.angular_vel = self.get_parameter("angular_vel").value
        topic = self.get_parameter("cmd_vel_topic").value

        self.pub = self.create_publisher(Twist, topic, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.running = True

        print("\n=== PatrolBot 键盘遥控 ===")
        print("w/s: 前进/后退  a/d: 左转/右转  空格: 急停  q: 退出\n")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def timer_callback(self):
        key = self.get_key()
        if key is None:
            return

        if key == "w":
            self.current_linear = self.linear_vel
            self.current_angular = 0.0
        elif key == "s":
            self.current_linear = -self.linear_vel
            self.current_angular = 0.0
        elif key == "a":
            self.current_linear = 0.0
            self.current_angular = self.angular_vel
        elif key == "d":
            self.current_linear = 0.0
            self.current_angular = -self.angular_vel
        elif key == " ":
            self.current_linear = 0.0
            self.current_angular = 0.0
        elif key == "q":
            self.get_logger().info("退出键盘遥控")
            self.running = False
            rclpy.shutdown()
            return

        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.pub.publish(twist)

    def destroy_node(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        while rclpy.ok() and node.running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
