#!/usr/bin/env python3
"""
diagnose_slam.py
----------------
Diagnostic node that checks all SLAM prerequisites:
  - /scan topic publishing & rate
  - /odom topic publishing & rate
  - odom → base_link TF available
  - base_link → laser_link TF available
  - map → odom TF available (from SLAM Toolbox)

Run:
  ros2 run bumperbot_hardware diagnose_slam

It prints a status report every 3 seconds.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf2_ros import Buffer, TransformListener


class DiagnoseSlam(Node):

    def __init__(self):
        super().__init__("diagnose_slam")
        self.get_logger().info("=== SLAM Diagnostic Node Started ===")

        # --- Counters ---
        self.scan_count = 0
        self.odom_count = 0
        self.last_scan_time = None
        self.last_odom_time = None

        # --- TF ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)

        # --- Report timer (every 3s) ---
        self.report_interval = 3.0
        self.create_timer(self.report_interval, self.print_report)

    def scan_cb(self, msg):
        self.scan_count += 1
        self.last_scan_time = self.get_clock().now()

    def odom_cb(self, msg):
        self.odom_count += 1
        self.last_odom_time = self.get_clock().now()

    def check_tf(self, parent, child):
        """Check if a TF is available. Returns (ok, age_ms)."""
        try:
            t = self.tf_buffer.lookup_transform(parent, child, Time())
            stamp = Time.from_msg(t.header.stamp)
            now = self.get_clock().now()
            age_ms = (now - stamp).nanoseconds / 1e6
            return True, age_ms
        except Exception:
            return False, -1

    def print_report(self):
        scan_hz = self.scan_count / self.report_interval
        odom_hz = self.odom_count / self.report_interval

        # Reset counters
        self.scan_count = 0
        self.odom_count = 0

        # TF checks
        odom_bl_ok, odom_bl_age = self.check_tf("odom", "base_link")
        bl_laser_ok, bl_laser_age = self.check_tf("base_link", "laser")
        map_odom_ok, map_odom_age = self.check_tf("map", "odom")

        self.get_logger().info("\n" + "=" * 55)
        self.get_logger().info("        SLAM DIAGNOSTIC REPORT")
        self.get_logger().info("=" * 55)

        # /scan
        scan_status = f"✅ OK ({scan_hz:.1f} Hz)" if scan_hz > 0 else "❌ NOT PUBLISHING"
        self.get_logger().info(f"  /scan topic:         {scan_status}")

        # /odom
        odom_status = f"✅ OK ({odom_hz:.1f} Hz)" if odom_hz > 0 else "❌ NOT PUBLISHING"
        self.get_logger().info(f"  /odom topic:         {odom_status}")

        # TF: odom → base_link
        if odom_bl_ok:
            self.get_logger().info(f"  TF odom→base_link:   ✅ OK (age: {odom_bl_age:.0f}ms)")
        else:
            self.get_logger().info(f"  TF odom→base_link:   ❌ MISSING")

        # TF: base_link → laser_link
        if bl_laser_ok:
            self.get_logger().info(f"  TF base→laser_link:  ✅ OK (age: {bl_laser_age:.0f}ms)")
        else:
            self.get_logger().info(f"  TF base→laser_link:  ❌ MISSING (need robot_state_publisher)")

        # TF: map → odom (published by SLAM Toolbox)
        if map_odom_ok:
            self.get_logger().info(f"  TF map→odom:         ✅ OK (age: {map_odom_age:.0f}ms)")
        else:
            self.get_logger().info(f"  TF map→odom:         ❌ MISSING (SLAM not computing)")

        # Summary
        self.get_logger().info("-" * 55)
        all_ok = scan_hz > 0 and odom_hz > 0 and odom_bl_ok and bl_laser_ok
        if all_ok and map_odom_ok:
            self.get_logger().info("  🎉 ALL GOOD — SLAM is running and mapping!")
        elif all_ok and not map_odom_ok:
            self.get_logger().info("  ⚠️  Prerequisites OK but SLAM is not computing yet.")
            self.get_logger().info("      → Move the robot so SLAM gets new scans")
            self.get_logger().info("      → Check slam_toolbox terminal for errors")
        else:
            self.get_logger().info("  ❌ PREREQUISITES MISSING — fix the items marked ❌ above")
            if not bl_laser_ok:
                self.get_logger().info("      → Launch robot_state_publisher with URDF")
            if scan_hz == 0:
                self.get_logger().info("      → Launch rplidar_ros node")
            if odom_hz == 0:
                self.get_logger().info("      → Launch odometry node")
        self.get_logger().info("=" * 55 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = DiagnoseSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Stopping diagnostic...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
