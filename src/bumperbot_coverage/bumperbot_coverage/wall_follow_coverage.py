#!/usr/bin/env python3
"""
wall_follow_coverage.py
-----------------------
Lidar wall-following coverage. Handles TWO arena shapes:

  arena_shape:=stadium    straights + curved end caps (the viva arena)
  arena_shape:=rectangle  four straight walls + square corners

The robot keeps the boundary wall on ONE side (default RIGHT) at a FIXED lidar
distance and follows it all the way around. After each full lap it steps one lane
inward, spiralling to the centre.

The two shapes need different behaviour at the ends:

  STADIUM   — the wall curves away gradually, so the side-distance error grows on
              its own and the PD follower tracks it. A curve feed-forward rounds
              the cap at the known semicircle radius.
  RECTANGLE — the side wall stays at a CONSTANT distance right up to the corner,
              so the side-distance error never grows and the PD alone commands NO
              turn: the robot would drive straight into the end wall. A square
              corner therefore needs an EXPLICIT turn — when the front wall closes
              in, stop and pivot 90 deg away from the followed wall, then hand back
              to the follower.

It drives on the lidar relative to the wall (NO map / global localization), which
suits the symmetric arena. Odometry HEADING is used only to count laps and to
measure the corner pivot.

Subscribes:  /scan, /odom, /water_cleaning_active
Publishes:   /cmd_vel

Run:
  ros2 run bumperbot_coverage wall_follow_coverage --ros-args -p arena_shape:=rectangle
  ros2 run bumperbot_coverage wall_follow_coverage --ros-args -p arena_shape:=stadium

SAFETY: start slow, hand near the power. The DISTANCE term always steers AWAY
from a wall it gets too close to, so the worst case is wobble, not a collision.
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
    MOTOR_AXLE_FROM_FRONT,
    GROUND_WIDTH,
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
CORNER_TURN = "CORNER_TURN"      # rectangle only: pivoting 90 deg at a corner
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
        # 0.04 => straight target_offset = half_width(0.11) + 0.04 = 0.15 m
        # (robot EDGE ~4 cm from the wall, so the wheels clear on the straights)
        self.declare_parameter("wall_clearance", 0.04)     # lane-0 side gap beyond half-width
        self.declare_parameter("overlap", COVERAGE_OVERLAP)
        self.declare_parameter("inner_margin", 0.04)       # stop margin from centre
        self.declare_parameter("max_laps", 8)              # hard backstop

        # --- Speed ---
        self.declare_parameter("linear_speed_max", 0.15)   # faster cruise
        self.declare_parameter("linear_speed_min", 0.08)   # keep moving on curves
        self.declare_parameter("turn_slow_k", 0.3)         # shed less speed on turns
        self.declare_parameter("curve_slow_near", 0.25)
        self.declare_parameter("curve_slow_far", 0.50)
        self.declare_parameter("curve_min_frac", 0.7)      # don't crawl through the cap

        # --- Steering gains (PD on side distance) ---
        # Higher K_DIST pulls back to the setpoint FAST so the straight after a
        # semicircle holds the border instead of drifting ~15 cm inward.
        self.declare_parameter("k_dist", 7.0)              # rad/s per m of lateral error
        self.declare_parameter("k_angle", 2.0)             # parallel/damping term
        self.declare_parameter("curve_ff_enable", True)
        self.declare_parameter("curve_margin", 0.25)       # how early to start rounding
        self.declare_parameter("r_min", 0.15)              # tightest feed-forward radius
        # Extra gap to hold ON the curved end only, so the swinging wheels clear
        # the border (also makes the semicircle a bit smaller). With straight
        # offset now 0.15, 0.03 keeps the curve at ~0.18 m (where it already works).
        self.declare_parameter("curve_extra_offset", 0.03)

        # --- Lidar cones (deg, half-angle) ---
        self.declare_parameter("side_cone_deg", 25.0)
        self.declare_parameter("diag_cone_deg", 18.0)
        self.declare_parameter("front_cone_deg", 8.0)

        # --- Lidar gating ---
        self.declare_parameter("min_valid_range", 0.08)
        self.declare_parameter("max_valid_range", 4.0)
        self.declare_parameter("min_cone_points", 3)
        self.declare_parameter("scan_timeout", 0.5)
        # How long a cone may stay empty before the robot HALTS. A single dropped
        # scan (or a cone that momentarily returns too few rays) must NOT slam the
        # brakes — at 0.18 m/s the robot travels ~9 cm in 0.5 s, so briefly
        # coasting on the last good reading is far safer than lurching stop-go.
        self.declare_parameter("lidar_grace", 0.5)

        # --- Safety ---
        self.declare_parameter("use_lidar_safety", True)
        self.declare_parameter("safety_distance", 0.07)    # very close head-on only —
        #   so the CURVED end wall does not trip the e-stop and stall the robot
        self.declare_parameter("safety_cone_deg", 8.0)
        self.declare_parameter("obstacle_resume_sec", 1.5)  # recover fast if it does pause
        self.declare_parameter("lap_timeout_sec", 90.0)     # per-lap watchdog

        self.declare_parameter("auto_start", True)
        self.declare_parameter("pose_source", "odom")

        # --- Arena shape ---
        # "stadium"   -> curved end caps, 2 per lap, curve feed-forward rounds them
        # "rectangle" -> square corners, 4 per lap, explicit 90 deg pivot at each
        self.declare_parameter("arena_shape", "stadium")
        # Short side of the arena — half of it is how far inward the spiral can go.
        self.declare_parameter("arena_short_side", GROUND_WIDTH)   # 1.2 m
        # Rectangle corner behaviour
        self.declare_parameter("corner_turn_speed", 0.8)      # rad/s pivot rate
        self.declare_parameter("corner_angle_deg", 90.0)      # square corner
        self.declare_parameter("corner_clearance", 0.03)      # gap kept while pivoting
        # How far ahead of a corner the robot starts easing AWAY from the side
        # wall. At the corner it must clear BOTH walls with its swinging tail, and
        # the tight lane-following offset is not enough room to spin in. Rather
        # than drive the whole lap further out, it widens only for this last
        # stretch and tucks back in after the turn.
        self.declare_parameter("corner_approach", 0.55)       # m

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
        self.curve_extra_offset = self.get_parameter("curve_extra_offset").value

        self.side_cone = math.radians(self.get_parameter("side_cone_deg").value)
        self.diag_cone = math.radians(self.get_parameter("diag_cone_deg").value)
        self.front_cone = math.radians(self.get_parameter("front_cone_deg").value)

        self.min_valid_range = self.get_parameter("min_valid_range").value
        self.max_valid_range = self.get_parameter("max_valid_range").value
        self.min_cone_points = self.get_parameter("min_cone_points").value
        self.scan_timeout = self.get_parameter("scan_timeout").value
        self.lidar_grace = self.get_parameter("lidar_grace").value

        self.use_lidar = self.get_parameter("use_lidar_safety").value
        self.safety_distance = self.get_parameter("safety_distance").value
        self.safety_cone = math.radians(self.get_parameter("safety_cone_deg").value)
        self.obstacle_resume_sec = self.get_parameter("obstacle_resume_sec").value
        self.lap_timeout_sec = self.get_parameter("lap_timeout_sec").value
        self.auto_start = self.get_parameter("auto_start").value
        self.pose_source = self.get_parameter("pose_source").value

        shape = str(self.get_parameter("arena_shape").value).lower()
        if shape not in ("stadium", "rectangle"):
            raise ValueError(
                f"arena_shape must be 'stadium' or 'rectangle', got '{shape}'"
            )
        self.is_rect = (shape == "rectangle")
        self.short_side = self.get_parameter("arena_short_side").value
        self.corner_turn_speed = self.get_parameter("corner_turn_speed").value
        self.corner_angle = math.radians(self.get_parameter("corner_angle_deg").value)
        self.corner_clearance = self.get_parameter("corner_clearance").value
        self.corner_approach = self.get_parameter("corner_approach").value

        # A square corner needs 4 turns per lap; a stadium has 2 end caps.
        self.ends_per_lap = 4 if self.is_rect else 2
        # The curve feed-forward is tuned to the semicircle — meaningless on a
        # rectangle, where the explicit pivot does the turning instead.
        if self.is_rect:
            self.curve_ff_enable = False

        # --- Spiral schedule ---
        self.target_offset = ROBOT_WIDTH / 2.0 + self.wall_clearance   # lane 0 (~0.16 m)
        self.max_offset = self.short_side / 2.0 - self.inner_margin    # ~0.56 m

        # How far inward each completed lap steps.
        #
        # Stepping a FULL robot width leaves nothing to spare: any wall-following
        # error, wheel slip, or lane-to-lane drift opens a real strip of floor
        # that the extraction head never passes over, and on the outward lap that
        # missed strip is invisible — the robot cannot tell it was skipped.
        #
        # Stepping HALF a robot width makes consecutive lanes overlap by 50%, so a
        # strip missed on one lap is swept on the next. It costs roughly twice the
        # laps, and buys coverage that does not depend on tracking being perfect.
        default_step = (ROBOT_WIDTH / 2.0 if self.is_rect
                        else ROBOT_WIDTH - self.overlap)
        self.declare_parameter("lane_step", default_step)
        self.lane_step = self.get_parameter("lane_step").value

        # Allow target_offset override (after computing the default)
        self.declare_parameter("target_offset", self.target_offset)
        self.target_offset = self.get_parameter("target_offset").value

        # Radius swept by the chassis corners when the robot spins on the spot.
        #
        # A differential drive rotates about the midpoint of its DRIVEN AXLE, not
        # about its geometric centre. This robot's axle sits MOTOR_AXLE_FROM_FRONT
        # (7 cm) behind the nose, i.e. well FORWARD of centre — so the long end is
        # the TAIL, and the tail corners are what swing widest. Measuring the sweep
        # from the geometric centre under-reports it by ~1.7 cm, which is enough to
        # clip a wall the robot "should" have cleared.
        axle_to_front = MOTOR_AXLE_FROM_FRONT                       # 0.070 m
        axle_to_rear = ROBOT_LENGTH - MOTOR_AXLE_FROM_FRONT         # 0.117 m
        self.pivot_radius = math.hypot(
            max(axle_to_front, axle_to_rear), ROBOT_WIDTH / 2.0
        )                                                            # ~0.161 m

        # Floor: the lidar is at the robot CENTRE, so center-to-wall must stay
        # above the body half-width or the robot scrapes the wall.
        offset_floor = ROBOT_WIDTH / 2.0 + 0.02   # ~0.13 m (2 cm body clearance)
        if self.target_offset < offset_floor:
            self.get_logger().warn(
                f"target_offset {self.target_offset:.2f} m below safe floor "
                f"{offset_floor:.2f} m (robot is {ROBOT_WIDTH:.2f} m wide) — clamping."
            )
            self.target_offset = offset_floor

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
        self.cone_stats = {}      # cone name -> (rays in cone, rays that passed the filter)
        self.scan_points = 0

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

        # --- Lidar dropout ride-through ---
        # Last usable side/front pair, and when the current dropout began. Lets a
        # brief gap coast instead of halting (see lidar_grace).
        self.last_side = None
        self.last_front = None
        self.miss_start = None

        # --- Rectangle corner pivot ---
        # Angle still owed on the current corner. Kept across a water/obstacle
        # pause so an interrupted pivot RESUMES rather than restarting (restarting
        # would over-rotate by however much it had already turned).
        self.corner_remaining = 0.0
        self.resume_to_corner = False

        # --- ROS wiring ---
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.create_subscription(Bool, WATER_CLEANING_TOPIC, self.water_callback, 10)
        self.create_subscription(LaserScan, "scan", self.scan_callback, 10)
        self.timer = self.create_timer(0.05, self.control_loop)   # 20 Hz

        lanes = []
        off = self.target_offset
        while off <= self.max_offset and len(lanes) < self.max_laps:
            lanes.append(off)
            off += self.lane_step
        overlap_pct = max(0.0, (1.0 - self.lane_step / ROBOT_WIDTH)) * 100.0

        self.get_logger().info(
            f"Wall-follow coverage: shape={shape} follow={side} "
            f"v={self.v_max:.2f} K_DIST={self.k_dist} K_ANGLE={self.k_angle}"
        )
        self.get_logger().info(
            f"Lane plan: {len(lanes)} laps at offsets "
            f"[{', '.join(f'{o:.2f}' for o in lanes)}] m — "
            f"step={self.lane_step:.2f} m on a {ROBOT_WIDTH:.2f} m body "
            f"= {overlap_pct:.0f}% lane overlap (max_offset={self.max_offset:.2f} m)"
        )

        if self.is_rect:
            self.get_logger().info(
                f"Corners: tail sweeps {self.pivot_radius:.3f} m about the axle, so "
                f"the robot eases out from {self.target_offset:.2f} m to "
                f"{self.pivot_offset:.2f} m over the last "
                f"{self.corner_approach:.2f} m before each corner "
                f"({self.corner_clearance * 100:.0f} cm tail clearance), then tucks "
                f"back in."
            )

    # ===================================================================
    #  LIDAR
    # ===================================================================

    def cone_distance(self, msg, bearing, half_angle, name=None, inf_is_open=False):
        """Conservative (20th-percentile) distance in a robot-frame cone.
        Lidar is yaw=pi mounted -> robot bearing b = normalize(scan_angle - pi).
        Returns None if too few valid rays.

        Also records, per cone, how many rays FELL IN the cone vs how many of
        those were VALID — the two numbers that tell you whether an empty cone
        means "lidar is not seeing that direction" or "everything there was
        filtered out as too near/far".
        """
        vals = []
        in_cone = 0
        n_inf = 0        # no return at all
        n_near = 0       # something CLOSER than min_valid_range (or r=0 "invalid")
        n_far = 0        # beyond max_valid_range
        angle_min = msg.angle_min
        inc = msg.angle_increment
        for idx, r in enumerate(msg.ranges):
            a = normalize_angle(angle_min + idx * inc)
            b = normalize_angle(a - math.pi)
            if abs(normalize_angle(b - bearing)) > half_angle:
                continue
            in_cone += 1
            if math.isinf(r) or math.isnan(r):
                n_inf += 1
                continue
            if r < self.min_valid_range:
                n_near += 1
                continue
            if r > self.max_valid_range:
                n_far += 1
                continue
            vals.append(r)

        if name is not None:
            self.cone_stats[name] = (in_cone, len(vals), n_inf, n_near, n_far)

        if len(vals) < self.min_cone_points:
            # No usable return. For the FRONT cone that is not blindness — it means
            # the beam went out and hit nothing, i.e. OPEN FLOOR ahead (an open end
            # of the arena, or a wall too far / too oblique to echo). Reporting it
            # as "clear at max range" is both true and safe: no corner is triggered,
            # and a wall that really is approaching WILL start returning rays long
            # before the robot reaches it.
            #
            # The SIDE cone gets no such treatment: no return there means there is
            # no wall to follow, and driving on would be genuinely blind.
            if inf_is_open and n_inf >= self.min_cone_points:
                return self.max_valid_range
            return None
        vals.sort()
        i = max(0, int(0.2 * len(vals)) - 1)
        return vals[i]

    def scan_callback(self, msg):
        # Distances in the four robot-frame cones (S flips left/right)
        self.d_front = self.cone_distance(
            msg, 0.0, self.front_cone, "front", inf_is_open=True
        )
        self.d_side = self.cone_distance(msg, self.S * math.pi / 2.0, self.side_cone, "side")
        self.d_fwd_side = self.cone_distance(msg, self.S * math.pi / 4.0, self.diag_cone, "fwd")
        self.d_back_side = self.cone_distance(msg, self.S * 3.0 * math.pi / 4.0, self.diag_cone, "back")
        self.scan_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        # Cone health, with the REASON each ray was rejected — the three reasons
        # mean completely different faults:
        #   inf  -> no return: the beam hits nothing (or a surface it cannot see)
        #   near -> a return CLOSER than min_valid_range: something is physically
        #           in the beam path (an occluding part of the robot itself), or
        #           the driver is reporting r=0 for an invalid measurement
        #   far  -> beyond max_valid_range
        self.scan_points = len(msg.ranges)
        if any(v is None for v in (self.d_front, self.d_side)):
            stats = " ".join(
                f"{k}:in={i},ok={o}(inf={inf},near={nr},far={fr})"
                for k, (i, o, inf, nr, fr) in self.cone_stats.items()
            )
            self.get_logger().warn(
                f"EMPTY CONE (need >= {self.min_cone_points} valid rays, "
                f"range window {self.min_valid_range:.2f}-{self.max_valid_range:.1f} m) | "
                f"scan {self.scan_points} rays | {stats}",
                throttle_duration_sec=1.0,
            )

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
        self.corner_remaining = 0.0
        self.resume_to_corner = False
        self.prev_theta = self.theta
        self.lap_start = self.now_sec()

    @property
    def pivot_offset(self):
        """Distance the robot must keep from a wall to SPIN beside it safely.

        Its tail sweeps `pivot_radius` about the axle, so anything closer than
        that plus a clearance gets clipped. This is the binding constraint at a
        corner and it is usually LARGER than the lane-following offset — which is
        why the robot eases out on the approach instead of following this far out
        for the whole lap.
        """
        return self.pivot_radius + self.corner_clearance

    @property
    def corner_trigger(self):
        """Front-wall distance at which the pivot begins.

        A 90 deg pivot turns the wall that was in FRONT into the wall on the SIDE,
        at the same distance — so stopping at pivot_offset leaves the robot able to
        spin clear of BOTH the front wall and the side wall it was following.
        """
        return self.pivot_offset

    def resume_state(self):
        """Where to go after a pause — back into an interrupted corner pivot if
        there was one, otherwise straight back to following."""
        return CORNER_TURN if self.resume_to_corner else FOLLOWING

    def check_lap_complete(self):
        """A lap = every end/corner seen AND ~2pi of accumulated heading."""
        if (self.end_caps_seen >= self.ends_per_lap
                and abs(self.lap_yaw) >= 2.0 * math.pi - 0.30):
            self.step_inward()
            return True
        return False

    def control_loop(self):
        if not self.odom_received:
            return

        d_side = self.fresh(self.d_side)
        d_front = self.fresh(self.d_front)
        d_fwd = self.fresh(self.d_fwd_side)
        d_back = self.fresh(self.d_back_side)

        # Wrap-safe heading delta since the last cycle. Computed every cycle so
        # prev_theta never goes stale, but only ADDED to the lap total in the
        # driving states below — a pause must not inject phantom rotation.
        dtheta = 0.0
        if self.prev_theta is not None:
            dtheta = normalize_angle(self.theta - self.prev_theta)
        self.prev_theta = self.theta

        # --- IDLE -> FOLLOWING ---
        if self.state == IDLE:
            if self.auto_start and d_side is not None:
                self.state = FOLLOWING
                self.arm_lap()
                self.get_logger().info(
                    f"=== WALL-FOLLOW COVERAGE STARTED "
                    f"({'RECTANGLE' if self.is_rect else 'STADIUM'}) ==="
                )
            return

        if self.state == COMPLETE:
            self.stop_robot()
            return

        if self.state == PAUSED_WATER:
            self.stop_robot()
            if not self.water_cleaning_active:
                self.state = self.resume_state()
            return

        if self.state == PAUSED_OBSTACLE:
            self.stop_robot()
            # Auto-resume when clear, OR after a timeout (static arena: the
            # "obstacle" is a permanent wall the follower will handle).
            if not self.obstacle_detected:
                self.state = self.resume_state()
            elif (self.now_sec() - self.pause_start) > self.obstacle_resume_sec:
                self.get_logger().warn("Obstacle pause timed out — resuming (static wall).")
                self.state = self.resume_state()
            return

        if self.state == LIDAR_LOST:
            self.stop_robot()
            if d_side is not None and d_front is not None:
                self.state = self.resume_state()
                self.get_logger().info("Lidar reading back — resuming.")
            return

        # ---------------- driving states: FOLLOWING and CORNER_TURN -------------
        # The corner pivot is a real part of the lap's 2pi, so the total accrues
        # here rather than in FOLLOWING alone.
        self.lap_yaw += dtheta

        if self.obstacle_detected:
            self.state = PAUSED_OBSTACLE
            self.pause_start = self.now_sec()
            self.stop_robot()
            return
        if self.water_cleaning_active:
            self.state = PAUSED_WATER
            self.stop_robot()
            return

        # --- CORNER_TURN: pivot in place until the 90 deg is spent ---
        # Spending down a remaining angle (rather than comparing against a start
        # heading) is what lets a pause interrupt the pivot and resume it mid-way.
        if self.state == CORNER_TURN:
            self.corner_remaining -= abs(dtheta)
            if self.corner_remaining <= 0.0:
                self.end_caps_seen += 1
                self.resume_to_corner = False
                self.state = FOLLOWING
                self.get_logger().info(
                    f"Corner {self.end_caps_seen}/{self.ends_per_lap} turned — following again."
                )
                self.check_lap_complete()
                return

            tw = Twist()
            tw.linear.x = 0.0
            # Turn AWAY from the followed wall: right wall (S=-1) -> +z (CCW/left)
            tw.angular.z = -self.S * self.corner_turn_speed
            self.cmd_vel_pub.publish(tw)
            self.get_logger().info(
                f"[CORNER_TURN] {math.degrees(self.corner_remaining):.0f} deg to go",
                throttle_duration_sec=0.5,
            )
            return

        # ------------------------------ FOLLOWING ------------------------------
        # FAIL-SAFE: never wall-follow blind — but ride out BRIEF dropouts.
        # Halting on a single missing scan makes the robot lurch stop-go, which is
        # both useless and harder on the drivetrain than coasting 9 cm on a reading
        # that is 0.5 s old. Only a SUSTAINED loss means we are truly blind.
        if d_side is None or d_front is None:
            now = self.now_sec()
            if self.miss_start is None:
                self.miss_start = now
            missing_for = now - self.miss_start

            if (missing_for < self.lidar_grace
                    and self.last_side is not None
                    and self.last_front is not None):
                self.get_logger().warn(
                    f"Lidar gap ({'side' if d_side is None else ''}"
                    f"{'+' if d_side is None and d_front is None else ''}"
                    f"{'front' if d_front is None else ''}) "
                    f"{missing_for:.2f}s — coasting on last reading.",
                    throttle_duration_sec=1.0,
                )
                d_side = self.last_side
                d_front = self.last_front
            else:
                self.state = LIDAR_LOST
                self.stop_robot()
                self.get_logger().warn(
                    f"Lidar side/front lost for {missing_for:.2f}s "
                    f"(side={'--' if d_side is None else 'ok'}, "
                    f"front={'--' if d_front is None else 'ok'}) — halting.",
                    throttle_duration_sec=2.0,
                )
                return
        else:
            self.miss_start = None
            self.last_side = d_side
            self.last_front = d_front

        # --- RECTANGLE: square corner ahead -> pivot, don't steer ---
        # The side wall holds a CONSTANT distance into a square corner, so e_dist
        # stays ~0 and the PD below would command no turn at all. The front cone
        # is the only thing that sees the corner coming.
        if self.is_rect and d_front <= self.corner_trigger:
            self.corner_remaining = self.corner_angle
            self.resume_to_corner = True
            self.state = CORNER_TURN
            self.stop_robot()
            self.get_logger().info(
                f"Corner ahead: front wall at {d_front:.2f} m — pivoting "
                f"{math.degrees(self.corner_angle):.0f} deg "
                f"{'LEFT' if self.S < 0 else 'RIGHT'}."
            )
            return

        # Are we rounding a stadium end cap? (front wall close)
        front_anticipate = self.target_offset + ROBOT_LENGTH / 2.0 + self.curve_margin
        on_curve = (not self.is_rect) and d_front < front_anticipate

        # On the curve, follow at a LARGER gap so the swinging wheels clear the
        # curved border (and the arc is a bit tighter / smaller diameter).
        eff_offset = self.target_offset + (self.curve_extra_offset if on_curve else 0.0)

        # RECTANGLE corner approach: ease AWAY from the side wall so there is room
        # for the tail to swing when the pivot starts. The lane offset is tuned for
        # tight coverage, not for spinning — following that close all lap is fine,
        # but arriving at a corner that close means the tail clips the side wall no
        # matter how early the turn is triggered. So widen for the last stretch
        # only; the follower tucks back in to the lane offset after the turn.
        if self.is_rect and d_front < self.corner_approach:
            eff_offset = max(eff_offset, self.pivot_offset)

        # --- Steering: PD on side distance + parallel/damping term ---
        e_dist = d_side - eff_offset                  # + => too far from wall
        if d_fwd is not None and d_back is not None:
            # psi>0 => nose toed TOWARD the wall (verified against ray geometry)
            psi = math.atan2(d_back - d_fwd, d_fwd + d_back)
        else:
            psi = 0.0
        # Distance term steers AWAY when too close; psi term is subtracted to DAMP
        steer = self.S * (self.k_dist * e_dist - self.k_angle * psi)

        # --- Curve feed-forward: round the end cap at radius (R_wall - eff_offset) ---
        if self.curve_ff_enable and on_curve:
            path_radius = max(GROUND_SEMICIRCLE_RADIUS - eff_offset, self.r_min)
            kappa_ff = self.v_cmd / path_radius
            steer += -self.S * kappa_ff     # right wall (S=-1) -> +kappa = LEFT/CCW
        # End-cap rising-edge counter for lap detection (stadium; the rectangle
        # counts its corners as it finishes each pivot instead)
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
            f"steer={steer:+.2f} v={v:.2f} "
            f"ends={self.end_caps_seen}/{self.ends_per_lap} "
            f"yaw={math.degrees(self.lap_yaw):+.0f}",
            throttle_duration_sec=0.5,
        )

        # --- Lap complete? OR watchdog ---
        lap_timed_out = (self.lap_start is not None
                         and (self.now_sec() - self.lap_start) > self.lap_timeout_sec)
        if self.check_lap_complete():
            return
        if lap_timed_out:
            self.get_logger().warn("Lap watchdog timeout — stepping inward.")
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
