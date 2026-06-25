#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SpiralCoverageNode(Node):
    def __init__(self):
        super().__init__('spiral_coverage_node')
        
        # --- Publishers & Subscribers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # --- Control Loop Timer (Update at 20Hz) ---
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # --- Robot State Trackers ---
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = None
        self.start_y = None
        
        # --- Spiral Parameters (Optimized for 0.6m Oval) ---
        self.time_elapsed = 0.0
        self.max_radius = 0.6          # Stop when reaching boundary limits
        self.spiral_expansion = 0.008   # Controls distance between nested spiral rings
        self.base_linear_speed = 0.08  # Steady m/s velocity
        self.is_running = True
        
        self.get_logger().info("Autonomous Spiral Coverage Node Initialized.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Capture starting coordinate on first reception
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

    def control_loop(self):
        if not self.is_running:
            return
            
        # Compute real-time distance from the origin point
        if self.start_x is not None:
            distance_from_start = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
            
            # Boundary Safety Check
            if distance_from_start >= self.max_radius:
                self.get_logger().info("Arena Boundary Reached. Terminating Mission.")
                self.stop_robot()
                self.is_running = False
                return

        # Increment time variable for tracking step increments
        self.time_elapsed += 0.05
        
        # Archimedean Spiral Trajectory Derivation
        # Target radius expands over time
        radius = self.spiral_expansion * self.time_elapsed
        
        twist = Twist()
        if radius > 0.01:
            twist.linear.x = self.base_linear_speed
            # Angular velocity decreases as turn radius expands (v = r * w -> w = v / r)
            twist.angular.z = self.base_linear_speed / radius
        else:
            # Initial pivot spin to kickstart trajectory tracking
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpiralCoverageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down coverage path execution.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
