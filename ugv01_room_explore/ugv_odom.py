#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import serial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


class Odom(Node):
    def __init__(self):
        super().__init__('ugv_odom')

        # ===== РАЗМЕРЫ / КАЛИБРОВКА =====
        self.body_length = 0.23154
        self.body_width = 0.13483
        self.total_width = 0.19682
        self.track_width = 0.04450
        self.contact_length = 0.10800

        # Повороты у тебя уже хорошие — не трогаем
        self.track_center = 0.19602

        # ===== НАСТРОЙКИ =====
        self.port = '/dev/ttyAMA0'
        self.baud = 115200

        self.yaw_sign = 1.0

        self.swap_lr = False
        self.lsign = 1.0
        self.rsign = 1.0

        # odl/odr -> метры
        self.odom_scale = 0.01

        # ===== СОСТОЯНИЕ =====
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_t = None
        self.last_odl = None
        self.last_odr = None

        self.last_vl = 0.0
        self.last_vr = 0.0
        self.last_dth_deg = 0.0
        self.last_ds = 0.0

        self.buf = ""

        # cmd_vel passthrough
        self.cmd_vx = 0.0
        self.cmd_wz = 0.0
        self.last_cmd_time = None
        self.zero_sent = False

        # ===== SERIAL =====
        self.ser = serial.Serial(self.port, self.baud, timeout=0.001)

        # ===== ROS =====
        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 20)

        # ===== ИНИЦИАЛИЗАЦИЯ НИЖНЕЙ ПЛАТЫ =====
        self.send({"T": 605, "cmd": 0})
        self.send({"T": 143, "cmd": 0})
        self.send({"T": 900, "main": 3, "module": 0})
        self.send({"T": 131, "cmd": 1})
        self.send({"T": 142, "cmd": 10})

        self.get_logger().info(
            "=== UGV01 WHEEL ODOM STARTED === "
            f"track_center={self.track_center:.5f} m "
            f"track_width={self.track_width:.5f} m "
            f"total_width={self.total_width:.5f} m "
            f"body_length={self.body_length:.5f} m"
        )

        self.create_timer(0.002, self.read_serial)
        self.create_timer(0.05, self.cmd_timer)
        self.create_timer(3.0, self.status)

    def send(self, d):
        try:
            self.ser.write((json.dumps(d, separators=(',', ':')) + '\n').encode())
        except Exception as e:
            self.get_logger().warn(f"send error: {e}")

    def cmd_cb(self, msg):
        self.cmd_vx = float(msg.linear.x)
        self.cmd_wz = float(msg.angular.z)
        self.last_cmd_time = self.get_clock().now()
        self.zero_sent = False

    def cmd_timer(self):
        now = self.get_clock().now()

        if self.last_cmd_time is None:
            if not self.zero_sent:
                self.send({"T": 13, "X": 0.0, "Z": 0.0})
                self.zero_sent = True
            return

        age = (now - self.last_cmd_time).nanoseconds / 1e9

        if age < 0.3:
            # вперед/назад инвертированы специально
            self.send({"T": 13, "X": -self.cmd_vx, "Z": self.cmd_wz})
        else:
            if not self.zero_sent:
                self.send({"T": 13, "X": 0.0, "Z": 0.0})
                self.zero_sent = True

    def read_serial(self):
        try:
            raw = self.ser.read(self.ser.in_waiting or 1)
            if not raw:
                return

            self.buf += raw.decode('utf-8', errors='ignore')

            latest_packet = None

            while '\n' in self.buf:
                line, self.buf = self.buf.split('\n', 1)
                line = line.strip()

                if not line.startswith('{'):
                    continue

                try:
                    d = json.loads(line)
                except Exception:
                    continue

                if d.get("T") == 1001:
                    latest_packet = d

            # Берём только самый свежий пакет
            if latest_packet is not None:
                self.process(latest_packet)

        except Exception as e:
            self.get_logger().warn(f"read error: {e}")

    def process(self, d):
        # Берем стабильный линейный путь из odl/odr
        if "odl" not in d or "odr" not in d:
            return

        try:
            odl = float(d["odl"])
            odr = float(d["odr"])
        except Exception:
            return

        if self.swap_lr:
            odl, odr = odr, odl

        odl *= self.lsign
        odr *= self.rsign

        now = self.get_clock().now()

        if self.last_t is None:
            self.last_t = now
            self.last_odl = odl
            self.last_odr = odr
            return

        dt = (now - self.last_t).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.2:
            self.last_t = now
            self.last_odl = odl
            self.last_odr = odr
            return

        d_left = (odl - self.last_odl) * self.odom_scale
        d_right = (odr - self.last_odr) * self.odom_scale

        self.last_t = now
        self.last_odl = odl
        self.last_odr = odr

        # защита от мусора
        if abs(d_left) > 0.20 or abs(d_right) > 0.20:
            return

        # мертвая зона
        if abs(d_left) < 0.0005:
            d_left = 0.0
        if abs(d_right) < 0.0005:
            d_right = 0.0

        # ЛИНЕЙНОЕ движение
        ds = (d_left + d_right) / 2.0

        # ПОВОРОТ — не трогаем твою калибровку
        dth = self.yaw_sign * ((d_right - d_left) / self.track_center)

        old_th = self.th
        self.th += dth

        avg_th = 0.5 * (old_th + self.th)
        self.x += ds * math.cos(avg_th)
        self.y += ds * math.sin(avg_th)

        vx = ds / dt
        wz = dth / dt

        self.last_ds = ds
        self.last_dth_deg = math.degrees(dth)
        self.last_vl = d_left / dt
        self.last_vr = d_right / dt

        self.publish(now, vx, wz)

    def publish(self, stamp, vx, wz):
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = wz

        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36

        pose_cov[0] = 0.02
        pose_cov[7] = 0.02
        pose_cov[14] = 9999.0
        pose_cov[21] = 9999.0
        pose_cov[28] = 9999.0
        pose_cov[35] = 0.05

        twist_cov[0] = 0.02
        twist_cov[7] = 0.02
        twist_cov[14] = 9999.0
        twist_cov[21] = 9999.0
        twist_cov[28] = 9999.0
        twist_cov[35] = 0.05

        odom.pose.covariance = pose_cov
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)

    def status(self):
        self.get_logger().info(
            f"ODOM x={self.x:.3f} y={self.y:.3f} "
            f"yaw_total={math.degrees(self.th):.1f}° "
            f"dth={self.last_dth_deg:.2f}° "
            f"ds={self.last_ds:.4f} "
            f"vL={self.last_vl:.3f} vR={self.last_vr:.3f} "
            f"track_center={self.track_center:.5f}"
        )

    def destroy_node(self):
        try:
            self.send({"T": 13, "X": 0.0, "Z": 0.0})
            self.send({"T": 131, "cmd": 0})
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Odom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
