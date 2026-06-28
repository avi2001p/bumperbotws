#!/usr/bin/env python3
"""
wall_follow_coverage.py
-----------------------
Lidar wall-following coverage for the stadium arena.

The robot keeps the boundary wall on ONE side (default RIGHT) at a FIXED lidar
distance and follows it all the way around — straights AND the curved ends — so
it holds a constant gap from the border everywhere. After each full lap it steps
one lane inward, tracing concentric ovals that spiral to the centre.

It drives on the lidar relative to the wall (NO map / global localization), which
suits the symmetric arena. Odometry HEADING is used only to count laps.

Subscribes:  /scan, /odom, /water_cleaning_active
Publishes:   /cmd_vel

Run:
  ros2 run bumperbot_coverage wall_follow_coverage
  ros2 run bumperbot_coverage wall_follow_coverage --ros-args -p target_offset:=0.15 -p linear_speed_max:=0.10

SAFETY: start slow, hand near the power. The DISTANCE term always steers AWAY
from a wall it gets too close to, so the worst case is wobble, not a collision.
Set `-p curve_ff_enable:=false` to disable the curve feed-forward and rely on the
(self-stabilising) PD wall-follower alone.
"""

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from bumperbot_hardware.parameters import (
    ROBOT_WIDTH,
    ROBOT_LENGTH,
    GROUND_SEMICIRCLE_RADIUS,
    COVERAGE_OVERLAP,
    MAX_HEADING_CORRECTION,
    CMD_VEL_TOPIC,
    ODOM_TOPIC,
    WATER_CLEANING_TOPIC,
)


# --- States ---
IDLE = "IDLE"
FOLLOWING = "FOLLOWING"
PAUSED_OBSTACLE = "PAUSED_OBSTACLE"
PAUSED_WATER = "PAUSED_WATER"
LIDAR_LOST = "LIDAR_LOST"
STEP_INWARD = "STEP_INWARD"
COMPLETE = "COMPLETE"


def yaw_from_quaternion(q):
    """Extract yaw (rad) from a list/tuple [x, y, z, w]."""
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class WallFollowCoverageNode(Node):

    def __init__(self):
        super().__init__("wall_follow_coverage")

        # --- Geometry / coverage params ---
        self.declare_parameter("follow_side", "right")     # "right" -> S=-1, "left" -> +1
        self.declare_parameter("wall_clearance", 0.05)     # lane-0 side gap beyond half-width
        self.declare_parameter("overlap", COVERAGE_OVERLAP)
        self.declare_parameter("inner_margin", 0.04)       # stop margin from centre
        self.declare_parameter("max_laps", 8)              # hard backstop

        # --- Speed ---
        self.declare_parameter("linear_speed_max", 0.10)
        self.declare_parameter("linear_speed_min", 0.05)
        self.declare_parameter("turn_slow_k", 0.5)
        self.declare_parameter("curve_slow_near", 0.25)
        self.declare_parameter("curve_slow_far", 0.50)
        self.declare_parameter("curve_min_frac", 0.5)

        # --- Steering gains (PD on side distance) ---
        self.declare_parameter("k_dist", 5.0)              # rad/s per m of lateral error
        self.declare_parameter("k_angle", 1.5)             # parallel/damping term
        self.declare_parameter("curve_ff_enable", True)
        self.declare_parameter("curve_margin", 0.25)       # how early to start rounding
        self.declare_parameter("r_min", 0.15)              # tightest feed-forward radius

        # --- Lidar cones (deg, half-angle) ---
        self.declare_parameter("side_cone_deg", 25.0)
        self.declare_parameter("diag_cone_deg", 18.0)
        self.declare_parameter("front_cone_deg", 8.0)

        # --- Lidar gating ---
        self.declare_parameter("min_valid_range", 0.08)
        self.declare_parameter("max_valid_range", 4.0)
        self.declare_parameter("min_cone_points", 3)
        self.declare_parameter("scan_timeout", 0.5)

        # --- Safety ---
        self.declare_parameter("use_lidar_safety", True)
        self.declare_parameter("safety_distance", 0.10)    # close+narrow head-on e-stop
        self.declare_parameter("safety_cone_deg", 8.0)
        self.declare_parameter("obstacle_resume_sec", 3.0)  # auto-resume (static arena)
        self.declare_parameter("lap_timeout_sec", 90.0)     # per-lap watchdog

        self.declare_parameter("auto_start", True)
        self.declare_parameter("pose_source", "odom")

        # --- Read params ---
        side = self.get_parameter("follow_side").value
        self.S = -1.0 if side == "right" else 1.0
        self.wall_clearance = self.get_parameter("wall_clearance").value
        self.overlap = self.get_parameter("overlap").value
        self.inner_margin = self.get_parameter("inner_margin").value
        self.max_laps = self.get_parameter("max_laps").value

        self.v_max = self.get_parameter("linear_speed_max").value
        self.v_min = self.get_parameter("linear_speed_min").value
        self.turn_slow_k = self.get_parameter("turn_slow_k").value
        self.curve_slow_near = self.get_parameter("curve_slow_near").value
        self.curve_slow_far = self.get_parameter("curve_slow_far").value
        self.curve_min_frac = self.get_parameter("curve_min_frac").value

        self.k_dist = self.get_parameter("k_dist").value
        self.k_angle = self.get_parameter("k_angle").value
        self.curve_ff_enable = self.get_parameter("curve_ff_enable").value
        self.curve_margin = self.get_parameter("curve_margin").value
        self.r_min = self.get_parameter("r_min").value

        self.side_cone = math.radians(self.get_parameter("side_cone_deg").value)
        self.diag_cone = math.radians(self.get_parameter("diag_cone_deg").value)
        self.front_cone = math.radians(self.get_parameter("front_cone_deg").value)

        self.min_valid_range = self.get_parameter("min_valid_range").value
        self.max_valid_range = self.get_parameter("max_valid_range").value
        self.min_cone_points = self.get_parameter("min_cone_points").value
        self.scan_timeout = self.get_parameter("scan_timeout").value

        self.use_lidar = self.get_parameter("use_lidar_safety").value
        self.safety_distance = self.get_parameter("safety_distance").value
        self.safety_cone = math.radians(self.get_parameter("safety_cone_deg").value)
        self.obstacle_resume_sec = self.get_parameter("obstacle_resume_sec").value
        self.lap_timeout_sec = self.get_parameter("lap_timeout_sec").value
        self.auto_start = self.get_parameter("auto_start").value
        self.pose_source = self.get_parameter("pose_source").value

        # Spiral schedule
        self.target_offset = ROBOT_WIDTH / 2.0 + self.wall_clearance   # lane 0 (~0.16 m)
        self.lane_step = ROBOT_WIDTH - self.overlap                    # ~0.20 m
        self.max_offset = GROUND_SEMICIRCLE_RADIUS - self.inner_margin  # ~0.56 m

        # Allow target_offset override (after computing the default)
        self.declare_parameter("target_offset", self.target_offset)
        self.target_offset = self.get_parameter("target_offset").value

        # --- Pose ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False

        # --- Lidar readings ---
        self.d_front = None
        self.d_side = None
        self.d_fwd_side = None
        self.d_back_side = None
        self.scan_stamp = self.get_clock().now()
        self.obstacle_detected = False

        # --- State / lap tracking ---
        self.state = IDLE
        self.water_cleaning_active = False
        self.lap_count = 0
        self.lap_yaw = 0.0
        self.prev_theta = None
        self.end_caps_seen = 0
        self._was_on_curve = False
        self.v_cmd = self.v_max
        self.pause_start = None
        self.lap_start = None

        # --- ROS wiring ---
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.create_subscription(Bool, WATER_CLEANING_TOPIC, self.water_callback, 10)
        self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)   # 20 Hz

        self.get_logger().info(
            f"Wall-follow coverage: follow={side} offset0={self.target_offset:.2f}m "
            f"lane_step={self.lane_step:.2f}m max_offset={self.max_offset:.2f}m "
            f"v={self.v_max:.2f} K_DIST={self.k_dist} K_ANGLE={self.k_angle}"
        )

    # ===================================================================
    #  LIDAR
    # ===================================================================

    def cone_distance(self, msg, bearing, half_angle):
        """Conservative (20th-percentile) distance in a robot-frame cone.
        Lidar is yaw=pi mounted -> robot bearing b = normalize(scan_angle - pi).
        Returns None if too few valid rays."""
        vals = []
        angle_min = msg.angle_min
        inc = msg.angle_increment
        for idx, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            if r < self.min_valid_range or r > self.max_valid_range:
                continue
            a = normalize_angle(angle_min + idx * inc)
            b = normalize_angle(a - math.pi)
            if abs(normalize_angle(b - bearing)) <= half_angle:
                vals.append(r)
        if len(vals) < self.min_cone_points:
            return None
        vals.sort()
        i = max(0, int(0.2 * len(vals)) - 1)
        return vals[i]

    def scan_callback(self, msg):
        # Distances in the four robot-frame cones (S flips left/right)
        self.d_front = self.cone_distance(msg, 0.0, self.front_cone)
        self.d_side = self.cone_distance(msg, self.S * math.pi / 2.0, self.side_cone)
        self.d_fwd_side = self.cone_distance(msg, self.S * math.pi / 4.0, self.diag_cone)
        self.d_back_side = self.cone_distance(msg, self.S * 3.0 * math.pi / 4.0, self.diag_cone)
        self.scan_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        # --- Head-on e-stop (close + narrow; reused convention) ---
        if not self.use_lidar:
            self.obstacle_detected = False
            return
        found = False
        for idx, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < self.min_valid_range:
                continue
            angle = normalize_angle(msg.angle_min + idx * msg.angle_increment)
            angle_from_front = math.pi - abs(angle)   # robot-forward = lidar +/-pi
            if angle_from_front < self.safety_cone and r < self.safety_distance:
                found = True
                break
        self.obstacle_detected = found

    def fresh(self, value):
        """value if the last scan is recent enough, else None (fail-safe)."""
        if value is None:
            return None
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        if age > self.scan_timeout:
            return None
        return value

    # ===================================================================
    #  ODOM / WATER
    # ===================================================================

    def odom_callback(self, msg):
        if self.pose_source != "odom":
            return
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion([q.x, q.y, q.z, q.w])
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Odometry received.")

    def water_callback(self, msg):
        self.water_cleaning_active = msg.data

    # ===================================================================
    #  CONTROL
    # ===================================================================

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def arm_lap(self):
        """Reset the lap detector for a fresh lap."""
        self.lap_yaw = 0.0
        self.end_caps_seen = 0
        self._was_on_curve = False
        self.prev_theta = self.theta
        self.lap_start = self.now_sec()

    def control_loop(self):
        if not self.odom_received:
            return

        d_side = self.fresh(self.d_side)
        d_front = self.fresh(self.d_front)
        d_fwd = self.fresh(self.d_fwd_side)
        d_back = self.fresh(self.d_back_side)

        # --- IDLE -> FOLLOWING ---
        if self.state == IDLE:
            if self.auto_start and d_side is not None:
                self.state = FOLLOWING
                self.arm_lap()
                self.get_logger().info("=== WALL-FOLLOW COVERAGE STARTED ===")
            return

        if self.state == COMPLETE:
            self.stop_robot()
            return

        if self.state == PAUSED_WATER:
            self.stop_robot()
            if not self.water_cleaning_active:
                self.state = FOLLOWING
            return

        if self.state == PAUSED_OBSTACLE:
            self.stop_robot()
            # Auto-resume when clear, OR after a timeout (static arena: the
            # "obstacle" is a permanent wall the follower will handle).
            if not self.obstacle_detected:
                self.state = FOLLOWING
            elif (self.now_sec() - self.pause_start) > self.obstacle_resume_sec:
                self.get_logger().warn("Obstacle pause timed out — resuming (static wall).")
                self.state = FOLLOWING
            return

        if self.state == LIDAR_LOST:
            self.stop_robot()
            if d_side is not None and d_front is not None:
                self.state = FOLLOWING
                self.get_logger().info("Lidar reading back — resuming.")
            return

        # --- FOLLOWING ---
        # Accumulate lap heading ONLY while actively following (signed, wrap-safe)
        if self.prev_theta is not None:
            self.lap_yaw += normalize_angle(self.theta - self.prev_theta)
        self.prev_theta = self.theta

        if self.obstacle_detected:
            self.state = PAUSED_OBSTACLE
            self.pause_start = self.now_sec()
            self.stop_robot()
            return
        if self.water_cleaning_active:
            self.state = PAUSED_WATER
            self.stop_robot()
            return

        # FAIL-SAFE: never wall-follow blind
        if d_side is None or d_front is None:
            self.state = LIDAR_LOST
            self.stop_robot()
            self.get_logger().warn(
                "Lidar side/front reading lost — halting (no blind driving).",
                throttle_duration_sec=2.0,
            )
            return

        # --- Steering: PD on side distance + parallel/damping term ---
        e_dist = d_side - self.target_offset          # + => too far from wall
        if d_fwd is not None and d_back is not None:
            # psi>0 => nose toed TOWARD the wall (verified against ray geometry)
            psi = math.atan2(d_back - d_fwd, d_fwd + d_back)
        else:
            psi = 0.0
        # Distance term steers AWAY when too close; psi term is subtracted to DAMP
        steer = self.S * (self.k_dist * e_dist - self.k_angle * psi)

        # --- Curve feed-forward: round the end cap at radius (R_wall - offset) ---
        front_anticipate = self.target_offset + ROBOT_LENGTH / 2.0 + self.curve_margin
        on_curve = d_front < front_anticipate
        if self.curve_ff_enable and on_curve:
            path_radius = max(GROUND_SEMICIRCLE_RADIUS - self.target_offset, self.r_min)
            kappa_ff = self.v_cmd / path_radius
            steer += -self.S * kappa_ff     # right wall (S=-1) -> +kappa = LEFT/CCW
        # End-cap rising-edge counter for lap detection
        if on_curve and not self._was_on_curve:
            self.end_caps_seen += 1
        self._was_on_curve = on_curve

        steer = max(-MAX_HEADING_CORRECTION, min(MAX_HEADING_CORRECTION, steer))

        # --- Speed: ease on turns and near the end wall ---
        v = self.v_max * (1.0 - self.turn_slow_k * abs(steer) / MAX_HEADING_CORRECTION)
        denom = (self.curve_slow_far - self.curve_slow_near)
        if denom > 1e-6:
            frac = (d_front - self.curve_slow_near) / denom
            v *= max(self.curve_min_frac, min(1.0, frac))
        v = max(self.v_min, min(self.v_max, v))
        self.v_cmd = v

        tw = Twist()
        tw.linear.x = v
        tw.angular.z = steer
        self.cmd_vel_pub.publish(tw)

        self.get_logger().info(
            f"[{self.state}] lap{self.lap_count} off={self.target_offset:.2f} "
            f"side={d_side:.2f} e={e_dist:+.2f} front={d_front:.2f} "
            f"steer={steer:+.2f} v={v:.2f} caps={self.end_caps_seen} "
            f"yaw={math.degrees(self.lap_yaw):+.0f}",
            throttle_duration_sec=0.5,
        )

        # --- Lap complete? (heading ~2pi AND both caps seen) OR watchdog ---
        lap_done = (self.end_caps_seen >= 2
                    and abs(self.lap_yaw) >= 2.0 * math.pi - 0.30)
        lap_timed_out = (self.lap_start is not None
                         and (self.now_sec() - self.lap_start) > self.lap_timeout_sec)
        if lap_timed_out:
            self.get_logger().warn("Lap watchdog timeout — stepping inward.")
        if lap_done or lap_timed_out:
            self.step_inward()

    def step_inward(self):
        self.target_offset += self.lane_step
        self.lap_count += 1
        if self.target_offset > self.max_offset or self.lap_count >= self.max_laps:
            self.state = COMPLETE
            self.stop_robot()
            self.get_logger().info("=== COVERAGE COMPLETE ===")
        else:
            self.get_logger().info(
                f"Lap done -> stepping inward to offset {self.target_offset:.2f} m"
            )
            self.arm_lap()
            self.state = FOLLOWING

    def stop_robot(self):
        if rclpy.ok():
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Coverage interrupted.")
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
