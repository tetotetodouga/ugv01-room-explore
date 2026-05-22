#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Empty


class CmdVelModeManager(Node):
    def __init__(self):
        super().__init__("cmd_vel_mode_manager")

        self.declare_parameter("joy_topic", "/cmd_vel_joy")
        self.declare_parameter("auto_topic", "/cmd_vel_auto")
        self.declare_parameter("output_topic", "/cmd_vel_web")
        self.declare_parameter("mode_topic", "/ugv01/mode")
        self.declare_parameter("stop_topic", "/ugv01/stop")
        self.declare_parameter("timeout_sec", 0.50)
        self.declare_parameter("publish_hz", 20.0)

        self.joy_topic = self.get_parameter("joy_topic").value
        self.auto_topic = self.get_parameter("auto_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.mode_topic = self.get_parameter("mode_topic").value
        self.stop_topic = self.get_parameter("stop_topic").value
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.mode = "JOY"
        self.pending_auto = False

        self.last_joy_cmd = Twist()
        self.last_auto_cmd = Twist()
        self.last_joy_time: Optional[float] = None
        self.last_auto_time: Optional[float] = None

        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        self.create_subscription(Twist, self.joy_topic, self.on_joy_cmd, 10)
        self.create_subscription(Twist, self.auto_topic, self.on_auto_cmd, 10)
        self.create_subscription(String, self.mode_topic, self.on_mode, 10)
        self.create_subscription(Empty, self.stop_topic, self.on_stop, 10)

        self.timer = self.create_timer(1.0 / max(self.publish_hz, 1.0), self.on_timer)

        self.get_logger().info("=== CMD_VEL MODE MANAGER STARTED ===")
        self.get_logger().info(f"JOY input:  {self.joy_topic}")
        self.get_logger().info(f"AUTO input: {self.auto_topic}")
        self.get_logger().info(f"Output:     {self.output_topic}")
        self.get_logger().info("Default mode: JOY")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def zero(self) -> Twist:
        return Twist()

    def publish_zero(self, repeat: int = 1):
        for _ in range(repeat):
            self.pub.publish(self.zero())

    def on_joy_cmd(self, msg: Twist):
        self.last_joy_cmd = msg
        self.last_joy_time = self.now_sec()

    def on_auto_cmd(self, msg: Twist):
        self.last_auto_cmd = msg
        self.last_auto_time = self.now_sec()

        if self.pending_auto:
            self.mode = "AUTO"
            self.pending_auto = False
            self.get_logger().info("AUTO command received, switched JOY -> AUTO")

    def on_mode(self, msg: String):
        requested = msg.data.strip().upper()

        if requested == "AUTO":
            if self.mode == "AUTO":
                return

            self.pending_auto = True
            self.get_logger().info("AUTO requested, waiting for first /cmd_vel_auto command")
            return

        if requested == "JOY":
            self.mode = "JOY"
            self.pending_auto = False
            self.last_joy_cmd = self.zero()
            self.last_joy_time = None
            self.publish_zero(repeat=5)
            self.get_logger().warn("Switched AUTO -> JOY with STOP")
            return

        if requested in ("STOP", "IDLE"):
            self.mode = "JOY"
            self.pending_auto = False
            self.last_joy_cmd = self.zero()
            self.last_auto_cmd = self.zero()
            self.last_joy_time = None
            self.last_auto_time = None
            self.publish_zero(repeat=8)
            self.get_logger().warn("STOP requested")
            return

        self.get_logger().warn(f"Unknown mode request: {msg.data}")

    def on_stop(self, _msg: Empty):
        self.mode = "JOY"
        self.pending_auto = False
        self.last_joy_cmd = self.zero()
        self.last_auto_cmd = self.zero()
        self.last_joy_time = None
        self.last_auto_time = None
        self.publish_zero(repeat=8)
        self.get_logger().warn("STOP topic received")

    def fresh(self, stamp: Optional[float]) -> bool:
        if stamp is None:
            return False
        return (self.now_sec() - stamp) <= self.timeout_sec

    def on_timer(self):
        if self.mode == "AUTO":
            if self.fresh(self.last_auto_time):
                self.pub.publish(self.last_auto_cmd)
            else:
                self.pub.publish(self.zero())
            return

        if self.mode == "JOY":
            if self.fresh(self.last_joy_time):
                self.pub.publish(self.last_joy_cmd)
            else:
                self.pub.publish(self.zero())
            return

        self.pub.publish(self.zero())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero(repeat=8)
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
