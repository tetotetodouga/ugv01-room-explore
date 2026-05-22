#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__("cmd_vel_watchdog")

        self.declare_parameter("input_topic", "/cmd_vel_web")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("timeout_sec", 0.45)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("max_linear_x", 0.22)
        self.declare_parameter("max_angular_z", 1.10)
        self.declare_parameter("deadband_linear", 0.003)
        self.declare_parameter("deadband_angular", 0.01)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.max_linear_x = float(self.get_parameter("max_linear_x").value)
        self.max_angular_z = float(self.get_parameter("max_angular_z").value)
        self.deadband_linear = float(self.get_parameter("deadband_linear").value)
        self.deadband_angular = float(self.get_parameter("deadband_angular").value)

        self.last_cmd_time: Optional[float] = None
        self.last_cmd = Twist()
        self.was_timed_out = False

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.sub = self.create_subscription(
            Twist,
            self.input_topic,
            self.on_cmd,
            10,
        )

        self.stop_sub = self.create_subscription(
            Empty,
            "/cmd_vel_watchdog/stop",
            self.on_stop,
            10,
        )

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info("=== CMD_VEL WATCHDOG STARTED ===")
        self.get_logger().info(f"input_topic:  {self.input_topic}")
        self.get_logger().info(f"output_topic: {self.output_topic}")
        self.get_logger().info(f"timeout_sec:  {self.timeout_sec}")
        self.get_logger().info(f"limits: linear.x <= {self.max_linear_x}, angular.z <= {self.max_angular_z}")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def clamp(self, value: float, limit: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return max(-limit, min(limit, value))

    def sanitize(self, msg: Twist) -> Twist:
        out = Twist()

        lx = self.clamp(float(msg.linear.x), self.max_linear_x)
        az = self.clamp(float(msg.angular.z), self.max_angular_z)

        if abs(lx) < self.deadband_linear:
            lx = 0.0
        if abs(az) < self.deadband_angular:
            az = 0.0

        out.linear.x = lx
        out.linear.y = 0.0
        out.linear.z = 0.0

        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = az

        return out

    def zero(self) -> Twist:
        return Twist()

    def on_cmd(self, msg: Twist):
        self.last_cmd = self.sanitize(msg)
        self.last_cmd_time = self.now_sec()

        if self.was_timed_out:
            self.get_logger().info("cmd_vel_web restored")
            self.was_timed_out = False

    def on_stop(self, _msg: Empty):
        self.last_cmd = self.zero()
        self.last_cmd_time = None
        self.was_timed_out = True

        for _ in range(4):
            self.pub.publish(self.zero())

        self.get_logger().warn("STOP requested on /cmd_vel_watchdog/stop")

    def on_timer(self):
        if self.last_cmd_time is None:
            return

        age = self.now_sec() - self.last_cmd_time

        if age > self.timeout_sec:
            self.pub.publish(self.zero())

            if not self.was_timed_out:
                self.get_logger().warn(
                    f"cmd_vel_web timeout: {age:.2f}s without command, publishing STOP"
                )
                self.was_timed_out = True

            return

        self.pub.publish(self.last_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for _ in range(6):
                node.pub.publish(Twist())
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
