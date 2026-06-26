#!/usr/bin/env python3
"""
calibrate_encoders.py
---------------------
Calibrate TICKS_PER_REV by the ROLLING-DISTANCE method (accurate, no driving).

Instead of judging "exactly one wheel revolution" by eye, you PUSH the whole
robot a measured straight distance and the node counts the encoder ticks. It
then computes how many ticks correspond to one wheel revolution, tied directly
to real ground distance — which is exactly what the odometry needs.

How to use (robot powered, on the floor):
  1. Mark a straight line on the floor exactly DISTANCE metres long (default 2 m).
  2. Terminal 1:  ros2 run bumperbot_hardware encoder_reader
  3. Terminal 2:  ros2 run bumperbot_hardware calibrate_encoders
        (to use a different distance:  --ros-args -p distance:=1.5)
  4. Line the robot's wheels up on the START mark. The node zeroes itself.
  5. Push the robot SLOWLY and STRAIGHT to the END mark (keep it on the line).
  6. Press Ctrl+C. The node prints the TICKS_PER_REV value to use.

Then tell me the printed numbers and I'll set TICKS_PER_REV in parameters.py.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from bumperbot_hardware.parameters import WHEEL_TICKS_TOPIC, WHEEL_CIRCUMFERENCE


class CalibrateEncoders(Node):

    def __init__(self):
        super().__init__("calibrate_encoders")

        self.declare_parameter("distance", 2.0)   # metres pushed
        self.distance = self.get_parameter("distance").value

        self.left_ticks = 0
        self.right_ticks = 0
        self.base_left = None        # baseline captured on first message
        self.base_right = None
        self.have_data = False

        self.create_subscription(
            Int32MultiArray,
            WHEEL_TICKS_TOPIC,
            self.tick_callback,
            10
        )

        self.get_logger().info("=" * 60)
        self.get_logger().info("  ENCODER CALIBRATION — rolling-distance method")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  Push distance : {self.distance:.3f} m")
        self.get_logger().info(f"  Wheel circumf.: {WHEEL_CIRCUMFERENCE:.4f} m "
                               f"({self.distance / WHEEL_CIRCUMFERENCE:.3f} revs)")
        self.get_logger().info("  Make sure encoder_reader is running.")
        self.get_logger().info("  Line robot on START, then push to END, then Ctrl+C.")
        self.get_logger().info("=" * 60)

        self.print_timer = self.create_timer(0.5, self.print_live)

    def tick_callback(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]
        if self.base_left is None:
            # First reading = the START baseline (robot lined up on START mark)
            self.base_left = self.left_ticks
            self.base_right = self.right_ticks
            self.get_logger().info(
                f"Baseline captured at START — L:{self.base_left}  R:{self.base_right}"
            )
        self.have_data = True

    def deltas(self):
        if self.base_left is None:
            return 0, 0
        return (self.left_ticks - self.base_left,
                self.right_ticks - self.base_right)

    def print_live(self):
        dl, dr = self.deltas()
        self.get_logger().info(f"  ticks since START — L:{dl:+7d}  R:{dr:+7d}")

    def report(self):
        dl, dr = self.deltas()
        revs = self.distance / WHEEL_CIRCUMFERENCE
        tpr_l = abs(dl) / revs if revs else 0.0
        tpr_r = abs(dr) / revs if revs else 0.0
        tpr_avg = (tpr_l + tpr_r) / 2.0

        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("  CALIBRATION RESULT")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"  Distance pushed : {self.distance:.3f} m  "
                               f"({revs:.3f} wheel revs)")
        self.get_logger().info(f"  Ticks  LEFT={abs(dl)}   RIGHT={abs(dr)}")
        self.get_logger().info(f"  TICKS_PER_REV  left ={tpr_l:.1f}")
        self.get_logger().info(f"  TICKS_PER_REV  right={tpr_r:.1f}")
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"  >>> USE  TICKS_PER_REV = {tpr_avg:.0f}")
        self.get_logger().info("=" * 60)
        if dl != 0 and dr != 0:
            ratio = abs(dl) / abs(dr)
            self.get_logger().info(
                f"  (L/R tick ratio = {ratio:.3f}; far from 1.0 means the two "
                f"wheels rolled different amounts — tell me if so.)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateEncoders()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.have_data:
            node.report()
        else:
            node.get_logger().warn(
                "No /wheel_ticks received — is encoder_reader running?"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
