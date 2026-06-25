"""
hardware.launch.py
------------------
Base hardware launch file for BumperBot.
Starts the core hardware nodes for the Raspberry Pi direct-control pipeline:
  - encoder_reader: reads wheel encoders via GPIO
  - pid: closed-loop PID speed controller
  - motor_driver: L298N motor driver via GPIO
  - odometry: computes robot pose from encoder ticks
  - static TF: base_link -> laser (from URDF values)
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # --- Static TF: base_link -> laser (values from URDF) ---
    # URDF laser_joint: xyz="-0.0050526 -0.0023221 0.1208" rpy="0 0 3.14"
    base_to_laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        arguments=[
            "--x", "-0.0050526",
            "--y", "-0.0023221",
            "--z", "0.1208",
            "--roll", "0",
            "--pitch", "0",
            "--yaw", "3.14",
            "--frame-id", "base_link",
            "--child-frame-id", "laser",
        ],
    )

    encoder_reader = Node(
        package="bumperbot_hardware",
        executable="encoder_reader",
        name="encoder_reader",
        output="screen",
    )

    pid_controller = Node(
        package="bumperbot_hardware",
        executable="pid",
        name="pid_controller",
        output="screen",
        parameters=[{
            "kp": 0.3,
            "ki": 0.3,
            "kd": 0.0,
        }],
    )

    motor_driver = Node(
        package="bumperbot_hardware",
        executable="motor_driver",
        name="motor_driver",
        output="screen",
    )

    odometry = Node(
        package="bumperbot_hardware",
        executable="odometry",
        name="odometry",
        output="screen",
    )

    return LaunchDescription([
        base_to_laser_tf,
        encoder_reader,
        pid_controller,
        motor_driver,
        odometry,
    ])
