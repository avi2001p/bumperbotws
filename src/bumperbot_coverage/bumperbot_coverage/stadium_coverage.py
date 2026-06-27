#!/usr/bin/env python3
"""
stadium_coverage.py
-------------------
Autonomous concentric-lap coverage path planner for a stadium-shaped
(rectangle + semicircular ends) prototype cricket ground.

Ground shape:
        ┌──────────┐
       /            \\
      │   straight   │
      │   section    │
       \\            /
        └──────────┘
        ←  width  →

Coverage strategy:
  - Start at the outer perimeter
  - Follow the stadium boundary (straights + semicircle arcs)
  - After each complete lap, shrink inward by (robot_width - overlap)
  - Repeat until the innermost ring is covered
  - If /water_cleaning_active is True, pause until cleaning is done

Subscribes:
  /odom                   — robot pose feedback
  /water_cleaning_active  — pause signal from water_actuator

Publishes:
  /cmd_vel                — velocity commands
  /water_detected         — (placeholder) for water sensor trigger
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from bumperbot_hardware.parameters import (
    GROUND_WIDTH,
    GROUND_STRAIGHT_LENGTH,
    GROUND_SEMICIRCLE_RADIUS,
    ROBOT_WIDTH,
    WHEEL_BASE,
    COVERAGE_OVERLAP,
    CMD_VEL_TOPIC,
    ODOM_TOPIC,
    WATER_CLEANING_TOPIC,
    KP_HEADING,
    KI_HEADING,
    K_CROSSTRACK,
    MAX_HEADING_CORRECTION,
    HEADING_INTEGRAL_LIMIT,
    HEADING_DEADBAND,
)


def yaw_from_quaternion(q):
    """Extract yaw (rad) from a list/tuple [x, y, z, w]."""
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


# === Segment types for the path ===
STRAIGHT = "STRAIGHT"
ARC = "ARC"

# === Coverage states ===
IDLE = "IDLE"
COVERING = "COVERING"
PAUSED_WATER = "PAUSED_WATER"
PAUSED_OBSTACLE = "PAUSED_OBSTACLE"
TRANSITIONING = "TRANSITIONING"
COMPLETE = "COMPLETE"


class StadiumCoverageNode(Node):

    def __init__(self):
        super().__init__("stadium_coverage_node")

        # --- Declare ROS parameters (all overridable at launch) ---
        self.declare_parameter("ground_width", GROUND_WIDTH)
        self.declare_parameter("ground_straight_length", GROUND_STRAIGHT_LENGTH)
        self.declare_parameter("robot_coverage_width", ROBOT_WIDTH)
        self.declare_parameter("overlap", COVERAGE_OVERLAP)
        # Gap (m) between the robot's SIDE and the boundary wall — applies on the
        # straights AND the outer swing of the turns. 0.05 = 5 cm each side (keeps
        # coverage tight in the limited 1.2 m arena). NOTE: the arc runs ~10% wide,
        # so if the robot scrapes a wall on a U-turn, bump this one number to 0.07.
        self.declare_parameter("wall_clearance", 0.05)
        self.declare_parameter("linear_speed", 0.08)
        self.declare_parameter("auto_start", True)
        # Where the planner reads the robot pose from:
        #   "odom" -> raw wheel odometry (drifts at the turns)
        #   "map"  -> the LOCALIZED pose (map->base_link TF from slam_toolbox
        #             localization), which corrects that drift. Needs
        #             localization.launch.py running with the saved 'arena' map.
        self.declare_parameter("pose_source", "odom")
        self.declare_parameter("use_lidar_safety", True)
        # Emergency-stop distance for a HEAD-ON obstacle. Kept small so the
        # arena's side walls (~0.6 m away) don't trip a permanent pause.
        self.declare_parameter("safety_distance", 0.18)  # meters
        # Half-angle of the forward emergency cone (degrees). Narrow so only a
        # genuine obstacle directly ahead stops the robot, not nearby walls.
        self.declare_parameter("safety_cone_deg", 20.0)

        self.ground_w = self.get_parameter("ground_width").value
        self.ground_sl = self.get_parameter("ground_straight_length").value
        self.coverage_w = self.get_parameter("robot_coverage_width").value
        self.overlap = self.get_parameter("overlap").value
        self.wall_clearance = self.get_parameter("wall_clearance").value
        self.linear_speed = self.get_parameter("linear_speed").value
        auto_start = self.get_parameter("auto_start").value
        self.pose_source = self.get_parameter("pose_source").value
        self.use_lidar = self.get_parameter("use_lidar_safety").value
        self.safety_distance = self.get_parameter("safety_distance").value
        self.safety_cone = math.radians(
            self.get_parameter("safety_cone_deg").value
        )

        self.semicircle_r = self.ground_w / 2.0

        # --- Robot pose from odometry ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.odom_received = False

        # --- State machine ---
        self.state = IDLE
        self.water_cleaning_active = False
        self.obstacle_detected = False

        # --- Path plan ---
        self.path_segments = []      # list of (type, param_dict)
        self.current_segment_idx = 0
        self.segment_start_x = 0.0
        self.segment_start_y = 0.0
        self.segment_start_theta = 0.0
        self.heading_integral = 0.0

        # --- Publishers / Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10
        )

        self.water_sub = self.create_subscription(
            Bool, WATER_CLEANING_TOPIC, self.water_callback, 10
        )

        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )

        # --- TF listener: used when pose_source == "map" to read the LOCALIZED
        #     pose (map -> base_link) published by slam_toolbox localization. ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Control loop at 20 Hz ---
        self.timer = self.create_timer(0.05, self.control_loop)

        # --- Generate the coverage path ---
        self.generate_coverage_path()

        if auto_start:
            self.get_logger().info("Auto-start enabled. Will begin coverage when odometry is available.")

        self.get_logger().info(
            f"Stadium Coverage initialized: "
            f"ground={self.ground_w:.2f}m × {self.ground_sl + 2*self.semicircle_r:.2f}m, "
            f"coverage_width={self.coverage_w:.3f}m, "
            f"use_lidar={self.use_lidar} ({self.safety_distance:.2f}m), "
            f"{len(self.path_segments)} segments planned"
        )

    # ===================================================================
    #  PATH GENERATION
    # ===================================================================

    def generate_coverage_path(self):
        """
        Generate concentric stadium laps, from outermost to innermost.

        Each lap consists of 4 segments:
          1. Straight forward (one side)
          2. Semicircle arc (top/bottom end)
          3. Straight forward (other side, opposite direction)
          4. Semicircle arc (other end)

        After each lap, offset inward by (coverage_width - overlap).
        """
        self.path_segments = []
        step = self.coverage_w - self.overlap
        # Start so the robot's SIDE clears the wall by wall_clearance (not just
        # its centre at half-width), so it never scrapes on straights or turns.
        offset = self.coverage_w / 2.0 + self.wall_clearance

        ring = 0
        while True:
            # Current ring dimensions
            arc_radius = self.semicircle_r - offset
            straight_len = self.ground_sl  # straight section doesn't shrink with simple inward offset

            if arc_radius <= 0.05:   # too small to navigate
                self.get_logger().info(
                    f"Coverage planned: {ring} rings, {len(self.path_segments)} total segments"
                )
                break

            # Segment 1: Straight forward
            self.path_segments.append((STRAIGHT, {
                "distance": straight_len,
                "ring": ring,
                "label": f"Ring {ring} — Straight 1"
            }))

            # Segment 2: Semicircle arc (180°)
            self.path_segments.append((ARC, {
                "radius": arc_radius,
                "angle": math.pi,    # 180 degrees
                "ring": ring,
                "label": f"Ring {ring} — Arc top"
            }))

            # Segment 3: Straight back (same length, opposite direction)
            self.path_segments.append((STRAIGHT, {
                "distance": straight_len,
                "ring": ring,
                "label": f"Ring {ring} — Straight 2"
            }))

            # Segment 4: Semicircle arc (180°)
            self.path_segments.append((ARC, {
                "radius": arc_radius,
                "angle": math.pi,
                "ring": ring,
                "label": f"Ring {ring} — Arc bottom"
            }))

            offset += step
            ring += 1

            # Safety limit
            if ring > 20:
                self.get_logger().warn("Ring limit reached (20). Stopping path generation.")
                break

    # ===================================================================
    #  CALLBACKS
    # ===================================================================

    def odom_callback(self, msg):
        """Update robot pose from wheel odometry (only when pose_source='odom')."""
        if self.pose_source != "odom":
            return
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.theta = yaw_from_quaternion(q)

        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Odometry received. Ready to start coverage.")

    def update_pose_from_tf(self):
        """Read the LOCALIZED pose (map -> base_link) when pose_source='map'.
        slam_toolbox localization publishes the map->odom correction, so this
        transform is the robot's drift-corrected pose in the map frame."""
        try:
            t = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
        except Exception:
            return  # localization/TF not ready yet — keep waiting
        self.x = t.transform.translation.x
        self.y = t.transform.translation.y
        q = t.transform.rotation
        self.theta = yaw_from_quaternion([q.x, q.y, q.z, q.w])
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Localized (map) pose received. Ready to start coverage.")

    def water_callback(self, msg):
        """Receive water cleaning pause/resume signal."""
        self.water_cleaning_active = msg.data

    def scan_callback(self, msg):
        """LiDAR safety scan callback — detects front obstacles."""
        if not self.use_lidar:
            self.obstacle_detected = False
            return

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        obstacle_found = False

        for idx, r in enumerate(msg.ranges):
            # Ignore invalid readings, infinity, and self-reflections
            if math.isinf(r) or math.isnan(r) or r < 0.08:
                continue

            # Calculate actual angle of the ray
            angle = angle_min + idx * angle_increment
            angle = self.normalize_angle(angle)

            # The lidar is mounted yaw=3.14 (URDF), so the ROBOT-FORWARD
            # direction corresponds to lidar angle ±pi. A ray is in the
            # forward emergency cone when it is within `safety_cone` of ±pi.
            angle_from_front = math.pi - abs(angle)
            if angle_from_front < self.safety_cone and r < self.safety_distance:
                obstacle_found = True
                break

        self.obstacle_detected = obstacle_found

    # ===================================================================
    #  CONTROL LOOP
    # ===================================================================

    def control_loop(self):
        """Main control loop — runs at 20 Hz."""

        # When localizing against the map, refresh the corrected pose each cycle.
        if self.pose_source == "map":
            self.update_pose_from_tf()

        # --- Wait for a pose (odometry or localization) ---
        if not self.odom_received:
            return

        # --- State: IDLE → start covering ---
        if self.state == IDLE:
            if self.get_parameter("auto_start").value:
                self.state = COVERING
                self.current_segment_idx = 0
                self.mark_segment_start()
                self.get_logger().info("=== COVERAGE STARTED ===")
            return

        # --- State: PAUSED (water cleaning) ---
        if self.state == PAUSED_WATER:
            self.stop_robot()
            if not self.water_cleaning_active:
                self.state = COVERING
                self.get_logger().info("Water cleaning done. Resuming coverage.")
            return

        # --- State: PAUSED (obstacle detected) ---
        if self.state == PAUSED_OBSTACLE:
            self.stop_robot()
            if not self.obstacle_detected:
                self.state = COVERING
                self.get_logger().info("Obstacle cleared. Resuming coverage.")
            return

        # --- State: COMPLETE ---
        if self.state == COMPLETE:
            self.stop_robot()
            return

        # --- State: COVERING ---
        if self.state == COVERING:
            # Check if obstacle triggered safety stop
            if self.obstacle_detected:
                self.state = PAUSED_OBSTACLE
                self.stop_robot()
                self.get_logger().warn("Obstacle detected! Pausing coverage...")
                return

            # Check if water cleaning triggered
            if self.water_cleaning_active:
                self.state = PAUSED_WATER
                self.stop_robot()
                self.get_logger().info("Water detected! Pausing for cleaning...")
                return

            # Check if all segments done
            if self.current_segment_idx >= len(self.path_segments):
                self.state = COMPLETE
                self.stop_robot()
                self.get_logger().info("=== COVERAGE COMPLETE ===")
                return

            # Execute current segment
            seg_type, seg_params = self.path_segments[self.current_segment_idx]

            if seg_type == STRAIGHT:
                done = self.execute_straight(seg_params)
            elif seg_type == ARC:
                done = self.execute_arc(seg_params)
            else:
                done = True

            if done:
                self.get_logger().info(
                    f"Segment {self.current_segment_idx + 1}/{len(self.path_segments)} done: "
                    f"{seg_params['label']}"
                )
                self.current_segment_idx += 1
                if self.current_segment_idx < len(self.path_segments):
                    self.mark_segment_start()

    # ===================================================================
    #  SEGMENT EXECUTION
    # ===================================================================

    def mark_segment_start(self):
        """Record the robot pose at the start of a new segment."""
        self.segment_start_x = self.x
        self.segment_start_y = self.y
        self.segment_start_theta = self.theta
        # Reset heading integral so each straight starts clean
        self.heading_integral = 0.0

    def execute_straight(self, params):
        """
        Drive straight for the specified distance.
        Returns True when the segment is complete.
        """
        target_dist = params["distance"]

        # Distance traveled since segment start
        dx = self.x - self.segment_start_x
        dy = self.y - self.segment_start_y
        dist_traveled = math.sqrt(dx * dx + dy * dy)

        if dist_traveled >= target_dist:
            self.stop_robot()
            return True

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.heading_correction()
        self.cmd_vel_pub.publish(twist)
        return False

    def heading_correction(self):
        """Steer back onto the segment's start LINE using heading + cross-track.

        Heading-hold alone keeps the robot pointing straight but lets a sideways
        offset persist; the cross-track term pulls it back onto the line.
        """
        dx = self.x - self.segment_start_x
        dy = self.y - self.segment_start_y

        heading_error = self.normalize_angle(self.theta - self.segment_start_theta)
        # Deadband: ignore sub-degree noise so we don't micro-steer on jitter.
        if abs(heading_error) < HEADING_DEADBAND:
            heading_error = 0.0

        # Integral term (PI): nulls the small steady drift a P-only hold leaves,
        # so each straight segment returns to its exact heading. heading_integral
        # is reset to 0 at the start of every segment (see mark_segment_start).
        self.heading_integral += heading_error * 0.05   # control loop dt = 0.05 s
        self.heading_integral = max(-HEADING_INTEGRAL_LIMIT,
                                    min(HEADING_INTEGRAL_LIMIT, self.heading_integral))

        # signed sideways offset from the line (+ = robot is LEFT of the line)
        cross_track = (-dx * math.sin(self.segment_start_theta)
                       + dy * math.cos(self.segment_start_theta))

        correction = -(KP_HEADING * heading_error
                       + KI_HEADING * self.heading_integral
                       + K_CROSSTRACK * cross_track)
        return max(-MAX_HEADING_CORRECTION,
                   min(MAX_HEADING_CORRECTION, correction))

    def execute_arc(self, params):
        """
        Follow a circular arc (semicircle = π radians).
        Returns True when the arc is complete.
        """
        target_angle = params["angle"]
        arc_radius = params["radius"]

        # Heading change since segment start
        d_theta = abs(self.normalize_angle(self.theta - self.segment_start_theta))

        if d_theta >= target_angle * 0.95:  # 95% threshold to handle overshoot
            self.stop_robot()
            return True

        # Arc motion: v = speed, omega = speed / radius
        twist = Twist()
        twist.linear.x = self.linear_speed
        if arc_radius > 0.01:
            twist.angular.z = self.linear_speed / arc_radius
        else:
            # Very tight turn — pivot in place
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)
        return False

    # ===================================================================
    #  HELPERS
    # ===================================================================

    def stop_robot(self):
        """Publish zero velocity."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = StadiumCoverageNode()
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
