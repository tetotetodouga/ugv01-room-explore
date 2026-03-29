#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def wrap_deg(d: float) -> float:
    while d > 180.0:
        d -= 360.0
    while d < -180.0:
        d += 360.0
    return d


class AutoExplore(Node):
    def __init__(self):
        super().__init__('auto_explore')

        # ===== topics =====
        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # fix lidar
        self.declare_parameter('scan_angle_offset_deg', 90.0)

        # ===== speeds =====
        self.declare_parameter('forward_wall', 0.08)
        self.declare_parameter('forward_explore', 0.12)
        self.declare_parameter('turn_speed', 0.55)
        self.declare_parameter('corner_turn_speed', 0.25)
        self.declare_parameter('back_speed', 0.08)

        # recovery speed
        self.declare_parameter('recovery_turn_speed', 0.28)

        # ===== distances =====
        self.declare_parameter('front_emergency', 0.20)
        self.declare_parameter('front_block', 0.42)
        self.declare_parameter('front_clear', 0.58)
        self.declare_parameter('recovery_front_free', 0.75)

        self.declare_parameter('side_too_close', 0.07)
        self.declare_parameter('wall_target', 0.16)

        # wall 10-20sm
        self.declare_parameter('start_wall_min', 0.07)
        self.declare_parameter('start_wall_max', 0.26)

        # ===== wall-follow logic =====
        self.declare_parameter('wall_bad_timeout', 3.5)
        self.declare_parameter('corner_turn_timeout', 4.5)
        self.declare_parameter('wall_reacquire_stable_count', 2)

        self.declare_parameter('shift_turn_time', 1.0)
        self.declare_parameter('shift_forward_time', 1.0)
        self.declare_parameter('min_loop_path', 4.0)
        self.declare_parameter('loop_return_dist', 0.35)
        self.declare_parameter('loop_return_yaw_deg', 30.0)

        # ===== stuck detection =====
        self.declare_parameter('stuck_time', 4.0)
        self.declare_parameter('stuck_dist', 0.05)

        # recovery timing
        self.declare_parameter('recovery_back_time', 0.6)

        # ===== control =====
        self.declare_parameter('control_hz', 15.0)
        self.declare_parameter('assess_time', 1.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.scan_angle_offset_deg = float(self.get_parameter('scan_angle_offset_deg').value)

        self.forward_wall = float(self.get_parameter('forward_wall').value)
        self.forward_explore = float(self.get_parameter('forward_explore').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.corner_turn_speed = float(self.get_parameter('corner_turn_speed').value)
        self.back_speed = float(self.get_parameter('back_speed').value)
        self.recovery_turn_speed = float(self.get_parameter('recovery_turn_speed').value)

        self.front_emergency = float(self.get_parameter('front_emergency').value)
        self.front_block = float(self.get_parameter('front_block').value)
        self.front_clear = float(self.get_parameter('front_clear').value)
        self.recovery_front_free = float(self.get_parameter('recovery_front_free').value)

        self.side_too_close = float(self.get_parameter('side_too_close').value)
        self.wall_target = float(self.get_parameter('wall_target').value)

        self.start_wall_min = float(self.get_parameter('start_wall_min').value)
        self.start_wall_max = float(self.get_parameter('start_wall_max').value)

        self.wall_bad_timeout = float(self.get_parameter('wall_bad_timeout').value)
        self.corner_turn_timeout = float(self.get_parameter('corner_turn_timeout').value)
        self.wall_reacquire_stable_count = int(self.get_parameter('wall_reacquire_stable_count').value)

        self.shift_turn_time = float(self.get_parameter('shift_turn_time').value)
        self.shift_forward_time = float(self.get_parameter('shift_forward_time').value)
        self.min_loop_path = float(self.get_parameter('min_loop_path').value)
        self.loop_return_dist = float(self.get_parameter('loop_return_dist').value)
        self.loop_return_yaw = math.radians(float(self.get_parameter('loop_return_yaw_deg').value))

        self.stuck_time = float(self.get_parameter('stuck_time').value)
        self.stuck_dist = float(self.get_parameter('stuck_dist').value)
        self.recovery_back_time = float(self.get_parameter('recovery_back_time').value)

        self.control_hz = float(self.get_parameter('control_hz').value)
        self.assess_time = float(self.get_parameter('assess_time').value)

        # ===== ROS =====
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 20)

        # ===== state =====
        self.latest_scan: Optional[LaserScan] = None

        self.x: Optional[float] = None
        self.y: Optional[float] = None
        self.yaw: Optional[float] = None

        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_yaw: Optional[float] = None
        self.total_path = 0.0

        self.progress_ref = None
        self.progress_ref_time = None

        self.state = 'START_ASSESS'
        self.state_since = self.get_clock().now()
        self.bad_wall_since = None
        self.wall_reacquire_count = 0

        self.escape_return_state = 'FREE_EXPLORE'
        self.escape_turn_dir = 1.0
        self.last_escape_turn_dir = 1.0

        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0

        # recovery internals
        self.recovery_turn_dir = 1.0
        self.recovery_prev_yaw = None
        self.recovery_turned = 0.0

        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'AUTO EXPLORE STARTED | scan_angle_offset_deg={self.scan_angle_offset_deg}'
        )

    # =========================================================
    # callbacks
    # =========================================================
    def scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_cb(self, msg: Odometry):
        px = float(msg.pose.pose.position.x)
        py = float(msg.pose.pose.position.y)

        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

        if self.x is not None and self.y is not None:
            step = math.hypot(px - self.x, py - self.y)
            if step < 0.5:
                self.total_path += step

        self.x = px
        self.y = py
        self.yaw = yaw

        if self.start_x is None:
            self.start_x = px
            self.start_y = py
            self.start_yaw = yaw
            self.reset_progress_ref()

    # =========================================================
    # helpers
    # =========================================================
    def seconds_since(self, t) -> float:
        return (self.get_clock().now() - t).nanoseconds / 1e9

    def set_state(self, new_state: str):
        self.state = new_state
        self.state_since = self.get_clock().now()

        if new_state in ('START_WALL_FOLLOW', 'FREE_EXPLORE', 'SHIFT_INWARD_FORWARD'):
            self.reset_progress_ref()

        if new_state != 'START_WALL_FOLLOW':
            self.bad_wall_since = None

        if new_state != 'WALL_CORNER_TURN':
            self.wall_reacquire_count = 0

        if new_state == 'STUCK_RECOVERY_SCAN':
            self.recovery_prev_yaw = self.yaw
            self.recovery_turned = 0.0

        self.get_logger().info(f'STATE -> {new_state}')

    def reset_progress_ref(self):
        if self.x is not None and self.y is not None:
            self.progress_ref = (self.x, self.y)
            self.progress_ref_time = self.get_clock().now()

    def is_stuck(self) -> bool:
        if self.progress_ref is None or self.progress_ref_time is None:
            return False
        if self.x is None or self.y is None:
            return False

        if self.seconds_since(self.progress_ref_time) < self.stuck_time:
            return False

        dist = math.hypot(self.x - self.progress_ref[0], self.y - self.progress_ref[1])

        if dist >= self.stuck_dist:
            self.reset_progress_ref()
            return False

        if abs(self.last_cmd_linear) > 0.04 or abs(self.last_cmd_angular) > 0.20:
            return True

        self.reset_progress_ref()
        return False

    def loop_done(self) -> bool:
        if self.start_x is None or self.start_y is None or self.start_yaw is None:
            return False
        if self.x is None or self.y is None or self.yaw is None:
            return False
        if self.total_path < self.min_loop_path:
            return False

        d = math.hypot(self.x - self.start_x, self.y - self.start_y)
        dyaw = abs(wrap_angle(self.yaw - self.start_yaw))
        return d < self.loop_return_dist and dyaw < self.loop_return_yaw

    def publish_cmd(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_pub.publish(msg)
        self.last_cmd_linear = vx
        self.last_cmd_angular = wz

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def start_recovery(self, left_score: float, right_score: float):
        # recovery free_explore
        self.recovery_turn_dir = 1.0 if left_score >= right_score else -1.0
        self.set_state('STUCK_RECOVERY_BACK')

    def update_recovery_angle(self):
        if self.yaw is None:
            return
        if self.recovery_prev_yaw is None:
            self.recovery_prev_yaw = self.yaw
            return

        delta = abs(wrap_angle(self.yaw - self.recovery_prev_yaw))
        self.recovery_turned += delta
        self.recovery_prev_yaw = self.yaw

    # =========================================================
    # laser helpers
    # =========================================================
    def sector_values(self, deg_min: float, deg_max: float) -> List[float]:
        if self.latest_scan is None:
            return []

        scan = self.latest_scan
        vals = []

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r):
                continue
            if r < max(scan.range_min, 0.02) or r > scan.range_max:
                continue

            ang = scan.angle_min + i * scan.angle_increment
            raw_deg = math.degrees(ang)
            robot_deg = wrap_deg(raw_deg + self.scan_angle_offset_deg)

            if deg_min <= robot_deg <= deg_max:
                vals.append(float(r))

        return vals

    def sector_min(self, deg_min: float, deg_max: float) -> float:
        vals = self.sector_values(deg_min, deg_max)
        return min(vals) if vals else float('inf')

    def sector_avg(self, deg_min: float, deg_max: float) -> float:
        vals = self.sector_values(deg_min, deg_max)
        return sum(vals) / len(vals) if vals else float('inf')

    def sector_score(self, deg_min: float, deg_max: float) -> float:
        vals = self.sector_values(deg_min, deg_max)
        if not vals:
            return 0.0
        clipped = [min(v, 1.5) for v in vals]
        return sum(clipped) / len(clipped)

    def start_wall_ok(self) -> bool:
        # start new
        right_min = self.sector_min(-110.0, -60.0)
        right_avg = self.sector_avg(-105.0, -55.0)
        front = self.sector_min(-20.0, 20.0)

        if not math.isfinite(right_min):
            return False

        if right_min < self.start_wall_min or right_min > self.start_wall_max:
            return False

        if math.isfinite(right_avg) and right_avg > 0.35:
            return False

        if front < self.front_block:
            return False

        return True

    def right_wall_ok(self) -> bool:
        right_min = self.sector_min(-110.0, -60.0)
        right_avg = self.sector_avg(-105.0, -55.0)

        if not math.isfinite(right_min):
            return False

        if right_min > 0.50:
            return False

        if math.isfinite(right_avg) and right_avg > 0.55:
            return False

        return True

    def right_wall_reacquire_ok(self) -> bool:
     
        right_min = self.sector_min(-125.0, -35.0)
        right_avg = self.sector_avg(-120.0, -45.0)
        front = self.sector_min(-20.0, 20.0)
        front_right = self.sector_min(-65.0, -10.0)

        if not math.isfinite(right_min):
            return False

        if right_min > 0.65:
            return False

        if math.isfinite(right_avg) and right_avg > 0.75:
            return False

        if front < 0.38:
            return False

        if front_right > 0.80:
            return False

        return True

    def start_escape(self, return_state: str, left_score: float, right_score: float):
        if abs(left_score - right_score) < 0.05:
            self.escape_turn_dir = -self.last_escape_turn_dir
        else:
            self.escape_turn_dir = 1.0 if left_score >= right_score else -1.0

        self.last_escape_turn_dir = self.escape_turn_dir
        self.escape_return_state = return_state
        self.set_state('ESCAPE_BACK')

    # =========================================================
    # main loop
    # =========================================================
    def control_loop(self):
        if self.latest_scan is None or self.x is None or self.y is None or self.yaw is None:
            self.stop()
            return

        front = self.sector_min(-20.0, 20.0)
        front_left = self.sector_min(20.0, 60.0)
        front_right = self.sector_min(-60.0, -20.0)
        left_min = self.sector_min(60.0, 100.0)
        right_min = self.sector_min(-110.0, -60.0)
        right_avg = self.sector_avg(-105.0, -55.0)

        left_score = self.sector_score(20.0, 100.0)
        right_score = self.sector_score(-100.0, -20.0)

        # ---------------- START_ASSESS ----------------
        if self.state == 'START_ASSESS':
            if self.seconds_since(self.state_since) < self.assess_time:
                self.stop()
                return

            if front < self.front_emergency:
                self.start_escape('START_ASSESS', left_score, right_score)
                return

            if self.start_wall_ok():
                self.set_state('START_WALL_FOLLOW')
            else:
                self.set_state('FREE_EXPLORE')
            return

        # ---------------- START_WALL_FOLLOW ----------------
        if self.state == 'START_WALL_FOLLOW':
            if self.loop_done():
                self.set_state('SHIFT_INWARD_TURN')
                return

            if not self.right_wall_ok():
                if self.bad_wall_since is None:
                    self.bad_wall_since = self.get_clock().now()
                elif self.seconds_since(self.bad_wall_since) > self.wall_bad_timeout:
                    self.set_state('FREE_EXPLORE')
                    return
            else:
                self.bad_wall_since = None

            if self.is_stuck():
                self.start_recovery(left_score, right_score)
                return

            if front < self.front_emergency:
                self.start_escape('START_WALL_FOLLOW', left_score, right_score)
                return

            if front < self.front_block or front_right < 0.16:
                self.set_state('WALL_CORNER_TURN')
                return

            control_dist = right_avg if math.isfinite(right_avg) else right_min
            if not math.isfinite(control_dist):
                control_dist = self.wall_target

            err = self.wall_target - control_dist
            wz = clamp(1.4 * err, -0.30, 0.30)

            vx = self.forward_wall
            if right_min < self.side_too_close:
                wz += 0.12
                vx = 0.05

            self.publish_cmd(vx, wz)
            return

        # ---------------- WALL_CORNER_TURN ----------------
        if self.state == 'WALL_CORNER_TURN':
            if self.seconds_since(self.state_since) > self.corner_turn_timeout:
                self.set_state('FREE_EXPLORE')
                return

            self.publish_cmd(0.0, self.corner_turn_speed)

            if self.right_wall_reacquire_ok():
                self.wall_reacquire_count += 1
            else:
                self.wall_reacquire_count = 0

            if self.wall_reacquire_count >= self.wall_reacquire_stable_count:
                self.set_state('START_WALL_FOLLOW')
            return

        # ---------------- SHIFT_INWARD ----------------
        if self.state == 'SHIFT_INWARD_TURN':
            if self.seconds_since(self.state_since) < self.shift_turn_time:
                self.publish_cmd(0.0, self.turn_speed)
            else:
                self.set_state('SHIFT_INWARD_FORWARD')
            return

        if self.state == 'SHIFT_INWARD_FORWARD':
            if front < self.front_emergency:
                self.start_escape('FREE_EXPLORE', left_score, right_score)
                return

            if self.seconds_since(self.state_since) < self.shift_forward_time:
                self.publish_cmd(self.forward_explore, 0.0)
            else:
                self.set_state('FREE_EXPLORE')
            return

        # ---------------- FREE_EXPLORE ----------------
        if self.state == 'FREE_EXPLORE':
            if self.is_stuck():
                self.start_recovery(left_score, right_score)
                return

            if front < self.front_emergency:
                self.start_escape('FREE_EXPLORE', left_score, right_score)
                return

            if front < self.front_block:
                turn_dir = 1.0 if left_score >= right_score else -1.0
                self.publish_cmd(0.0, turn_dir * self.turn_speed)
                return

            wz = 0.6 * (left_score - right_score)

            if right_min < self.side_too_close:
                wz += 0.25
            if left_min < self.side_too_close:
                wz -= 0.25

            wz = clamp(wz, -self.turn_speed, self.turn_speed)
            self.publish_cmd(self.forward_explore, wz)
            return

        # ---------------- STUCK_RECOVERY_BACK ----------------
        if self.state == 'STUCK_RECOVERY_BACK':
            if self.seconds_since(self.state_since) < self.recovery_back_time:
                self.publish_cmd(-self.back_speed, 0.0)
            else:
                self.set_state('STUCK_RECOVERY_SCAN')
            return

        # ---------------- STUCK_RECOVERY_SCAN ----------------
        if self.state == 'STUCK_RECOVERY_SCAN':
            self.update_recovery_angle()

            if front > self.recovery_front_free:
                self.set_state('FREE_EXPLORE')
                return

            if self.recovery_turned >= 2.0 * math.pi:
                self.set_state('FREE_EXPLORE')
                return

            self.publish_cmd(0.0, self.recovery_turn_dir * self.recovery_turn_speed)
            return

        # ---------------- ESCAPE ----------------
        if self.state == 'ESCAPE_BACK':
            if self.seconds_since(self.state_since) < 0.35:
                self.publish_cmd(-self.back_speed, 0.0)
            else:
                self.set_state('ESCAPE_TURN')
            return

        if self.state == 'ESCAPE_TURN':
            if self.seconds_since(self.state_since) < 0.75:
                self.publish_cmd(0.0, self.escape_turn_dir * self.turn_speed)
            else:
                self.set_state(self.escape_return_state)
            return

    def destroy_node(self):
        try:
            self.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = AutoExplore()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'auto_explore fatal: {e}')
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
