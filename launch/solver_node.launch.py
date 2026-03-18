# LAUNCH FILE FOR RUNTIME HARDWARE CONTROL
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import math


def deg2rad(d):
    return d * math.pi / 180.0


def generate_launch_description():
    pkg = FindPackageShare("joystick_rsu")

    urdf_path = PathJoinSubstitution([pkg, "robot_model", "rsu_for_prototype.urdf"])
    param_file = PathJoinSubstitution([pkg, "config", "rsu_solver.yaml"])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["cat ", urdf_path]),
            }],
            arguments=["--ros-args", "--log-level", "error"],
        ),

        Node(
            package="joystick_rsu",
            executable="rsu_solver_node.py",
            name="rsu_solver_node",
            output="screen",
            parameters=[
                param_file,
                {"REALTIME_CONTROL_MODE": True},
            ],
            arguments=["--ros-args", "--log-level", "warn"],
        ),
    ])