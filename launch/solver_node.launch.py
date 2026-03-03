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
    rviz_path = PathJoinSubstitution([pkg, "config", "rsu_2dof.rviz"])

    world_frame = LaunchConfiguration("world_frame")

    gate_publish_by_tf_check = LaunchConfiguration("gate_publish_by_tf_check")
    tf_check_rate_hz = LaunchConfiguration("tf_check_rate_hz")
    tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
    len_tol_m = LaunchConfiguration("len_tol_m")
    ang_min_deg = LaunchConfiguration("ang_min_deg")
    ang_max_deg = LaunchConfiguration("ang_max_deg")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("world_frame", default_value="base_link"),

        DeclareLaunchArgument("gate_publish_by_tf_check", default_value="true"),
        DeclareLaunchArgument("tf_check_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("tf_timeout_sec", default_value="0.05"),
        DeclareLaunchArgument("len_tol_m", default_value="0.002"),
        DeclareLaunchArgument("ang_min_deg", default_value="70.0"),
        DeclareLaunchArgument("ang_max_deg", default_value="120.0"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": Command(["cat ", urdf_path]),
            }],
        ),

        Node(
            package="joystick_rsu",
            executable="rsu_solver_node.py",
            name="rsu_solver_node",
            output="screen",
            parameters=[{
                # ===== RSU IK params =====
                "a_W_mm_flat": [0.0,  36.0, 169.5,  0.0, -36.0, 81.0],
                "b_F_mm_flat": [-30.0, 36.0, 0.0,  -30.0, -36.0, 0.0],
                "c_mm": [30.0, -30.0],
                "r_mm": [169.5, 81.0],
                "psi_rad": [deg2rad(90.0), deg2rad(-90.0)],

                # ===== joints =====
                "joint_ankle_pitch": "ankle_pitch",
                "joint_ankle_roll": "ankle_roll",
                "joint_upper_crank": "upper_crank",
                "joint_lower_crank": "lower_crank",

                "hold_alpha_on_infeasible": True,

                # ===== TF-based crank sanity check params =====
                "world_frame": world_frame,
                "tf_timeout_sec": tf_timeout_sec,
                "tf_check_rate_hz": tf_check_rate_hz,
                "gate_publish_by_tf_check": gate_publish_by_tf_check,

                # frames (keep identical with plotter)
                "c1_frame": "point_c1_1",
                "c2_frame": "point_c2_1",
                "u1_frame": "point_u1_1",
                "u2_frame": "point_u2_1",

                # targets + tolerances (meters)
                "target_len_1_m": 0.1695,
                "target_len_2_m": 0.0810,
                "len_tol_m": len_tol_m,

                # angle range (deg) - ALL 4 enforced inside node
                "ang_min_deg": ang_min_deg,
                "ang_max_deg": ang_max_deg,

                "tf_log_rate_hz": 1.0,
            }],
            # arguments=["--ros-args", "--log-level", "info"],
        ),
    ])