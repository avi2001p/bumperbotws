#!/usr/bin/env python3
"""
obstacle_stop_test.py
---------------------
Standalone STRAIGHT-LINE obstacle-stop test. The robot drives forward holding its
heading for a set distance (default 3 m), and STOPS when something appears ahead —
then RESUMES when the path clears. It also stops on its own once the distance is
reached. No wall-following, no coverage: it tests only "drive straight and stop at
an obstacle", so you can validate the obstacle behaviour on its own before running
it inside the coverage mission.

This deliberately does NOT do the obstacle-vs-wall discrimination that the
coverage node does (there are no walls in a straight-line test — anything ahead is
an obstacle). It is the simplest possible check that detection + stopping works.

The lidar is mounted yaw=pi, so ROBOT-FORWARD = scan angle ~ +/- pi.

Subscribes:  /scan (LaserScan), /odom (Odometry)
Publishes:   /cmd_vel (Twist)

Run (with hardware.launch.py + the RPLiDAR already running):
  ros2 run bumperbot_hardware obstacle_stop_test
  ros2 run bumperbot_hardware obstacle_stop_test --ros-args \
      -p distance:=3.0 -p speed:=0.15 -p stop_distance:=0.30 -p clear_distance:=0.40

SAFETY: hand near the power. It drives forward until it sees something; if the
lidar is blind (sunlight!) it will NOT see the obstacle — so watch the logged
front distance and be ready to stop it.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from bumperbot_hardware.parameters import (
    CMD_VEL_TOPIC,
    ODOM_TOPIC,
    MAX_LINEAR_SPEED,
    KP_HEADING,
    MAX_HEADING_CORRECTION,
)


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ObstacleStopTest(Node):

    def __init__(self):
        super().__init__("obstacle_stop_test")

        self.declare_parameter("distance", 3.0)           # metres to travel then stop
        self.declare_parameter("speed", 0.15)             # m/s forward
        self.declare_parameter("stop_distance", 0.30)     # stop when front <= this
        self.declare_parameter("clear_distance", 0.40)    # resume when front > this
        self.declare_parameter("front_cone_deg", 12.0)    # half-angle of the front look
        self.declare_parameter("min_valid_range", 0.08)   # lidar floor (blind below this)
        self.declare_parameter("max_valid_range", 8.0)
        self.declare_parameter("min_points", 2)           # rays needed to trust a reading
        self.declare_parameter("scan_timeout", 0.5)       # halt if scans go stale
        self.declare_parameter("heading_gain", KP_HEADING)

        self.target_distance = self.get_parameter("distance").value
        self.speed = min(self.get_parameter("speed").value, MAX_LINEAR_SPEED)
        self.stop_distance = self.get_parameter("stop_distance").value
        self.clear_distance = self.get_parameter("clear_distance").value
        self.front_cone = math.radians(self.get_parameter("front_cone_deg").value)
        self.min_valid = self.get_parameter("min_valid_range").value
        self.max_valid = self.get_parameter("max_valid_range").value
        self.min_points = self.get_parameter("min_points").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.heading_gain = self.get_parameter("heading_gain").value

        self.theta = 0.0
        self.start_theta = None
        self.x = 0.0
        self.y = 0.0
        self.start_x = None
        self.start_y = None
        self.d_front = None
        self.scan_stamp = self.get_clock().now()
        self.stopped = False        # obstacle hysteresis latch
        self.finished = False       # reached target distance

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_cb, 10)
        self.create_subscription(LaserScan, "scan", self.scan_cb, 10)
        self.timer = self.create_timer(0.05, self.loop)   # 20 Hz

        self.get_logger().info(
            f"Obstacle-stop straight-line test: drive {self.target_distance:.1f} m at "
            f"{self.speed:.2f} m/s, STOP at any obstacle within {self.stop_distance:.2f} m "
            f"(resume past {self.clear_distance:.2f} m). Waiting for /odom and /scan..."
        )

    def odom_cb(self, msg):
        self.theta = yaw_from_quaternion(msg.pose.pose.orientation)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        if self.start_theta is None:
            self.start_theta = self.theta
            self.start_x = self.x
            self.start_y = self.y
            self.get_logger().info("Odometry received — driving straight.")

    def scan_cb(self, msg):
        """Closest valid ray in the front cone (robot-forward = scan +/- pi)."""
        vals = []
        for idx, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.min_valid or r > self.max_valid:
                continue
            angle = normalize_angle(msg.angle_min + idx * msg.angle_increment)
            if (math.pi - abs(angle)) < self.front_cone:   # within the forward cone
                vals.append(r)
        self.d_front = min(vals) if len(vals) >= self.min_points else None
        self.scan_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

    def scan_fresh(self):
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        return age <= self.scan_timeout

    def loop(self):
        if self.start_theta is None or self.finished:
            return

        # Reached the target distance -> stop and finish.
        traveled = math.hypot(self.x - self.start_x, self.y - self.start_y)
        if traveled >= self.target_distance:
            self.cmd_pub.publish(Twist())
            self.finished = True
            self.get_logger().info(
                f"=== DONE === reached {self.target_distance:.1f} m "
                f"(odom {traveled:.2f} m). Stopped."
            )
            return

        # Fail-safe: no fresh scan -> stop (never drive blind).
        if not self.scan_fresh():
            self.cmd_pub.publish(Twist())
            self.get_logger().warn("No fresh /scan — stopping (lidar lost).",
                                   throttle_duration_sec=2.0)
            return

        front = self.d_front
        # Hysteresis: stop at stop_distance, only release past clear_distance.
        if front is not None:
            if front <= self.stop_distance:
                self.stopped = True
            elif front > self.clear_distance:
                self.stopped = False

        if self.stopped:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"OBSTACLE at {front:.2f} m — STOPPED, waiting for it to clear.",
                throttle_duration_sec=1.0,
            )
            return

        # Drive forward, holding the starting heading.
        err = normalize_angle(self.theta - self.start_theta)
        tw = Twist()
        tw.linear.x = self.speed
        tw.angular.z = max(-MAX_HEADING_CORRECTION,
                           min(MAX_HEADING_CORRECTION, -self.heading_gain * err))
        self.cmd_pub.publish(tw)
        shown = f"{front:.2f} m" if front is not None else "clear"
        self.get_logger().info(
            f"driving {self.speed:.2f} m/s | traveled {traveled:.2f}/{self.target_distance:.1f} m "
            f"| front={shown}",
            throttle_duration_sec=0.5,
        )

    def stop(self):
        try:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStopTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
