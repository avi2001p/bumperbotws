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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bumperbot_hardware.parameters import KFF, INTEGRAL_WINDUP_LIMIT


def generate_launch_description():

    # Feed-forward gain — the knob for a TORQUE shortfall. The default was
    # calibrated on smooth tile; a rougher floor needs more PWM for the same
    # speed, so raise this if the robot strains but will not move.
    #   ros2 launch bumperbot_hardware hardware.launch.py kff:=0.55
    kff_arg = DeclareLaunchArgument("kff", default_value=str(KFF))
    i_limit_arg = DeclareLaunchArgument(
        "integral_limit", default_value=str(INTEGRAL_WINDUP_LIMIT)
    )

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
            "kp": 0.0,
            "ki": 0.5,   # slow integral balances the wheels; integral_limit caps it
                          # so it can't overshoot into a left turn
            "kd": 0.0,
            "kff": LaunchConfiguration("kff"),
            "integral_limit": LaunchConfiguration("integral_limit"),
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
        kff_arg,
        i_limit_arg,
        base_to_laser_tf,
        encoder_reader,
        pid_controller,
        motor_driver,
        odometry,
    ])
