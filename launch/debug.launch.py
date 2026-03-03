# LAUNCH FILE FOR DEBUGGING SOLVER NODE (WITH RVIZ2 AND LOGITECH F310 GAMEPAD INPUT)
"""
launch.debug.launch의 Docstring

[rsu_solver_node.py-3]   L1 = 0.1542 m (target=0.1540 m, err=0.2 mm)
[rsu_solver_node.py-3]   L2 = 0.0662 m (target=0.0660 m, err=0.2 mm)
[rsu_solver_node.py-3]   pc1 = [-0.09584910445914857, 0.0398, 0.22824101379440248]
[rsu_solver_node.py-3]   pu1 = [-0.0965, 0.040667, 0.07400000000000001]
[rsu_solver_node.py-3]   pu1-pc1 = [-0.0006508955408514316, 0.0008669999999999997, -0.15424101379440247]
[rsu_solver_node.py-3]   pc2 = [-0.09584957853964238, -0.0392, 0.140172172715111]
[rsu_solver_node.py-3]   pu2 = [-0.0965, -0.040667, 0.07400000000000001]
[rsu_solver_node.py-3]   pu2-pc2 = [-0.0006504214603576253, -0.001467000000000003, -0.066172172715111]
[rsu_solver_node.py-3]   angles (deg): C1=89.7, U1=90.3, C2=91.3, U2=88.7 (tf_hardware_safetycheck() at /home/lwj/colcon_ws/ins
"""




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
        DeclareLaunchArgument("ang_min_deg", default_value="65.0"),
        DeclareLaunchArgument("ang_max_deg", default_value="115.0"),

        # Robot description
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
            executable="gamepad_rpy_node.py",
            name="gamepad_rpy_node",
            output="screen",
            parameters=[{
                "rate_hz": 50.0,
                "wait_first_input": True,
                "input_deadzone": 0.05,
                "roll_rate_scale": 1.5,
                "pitch_rate_scale": 1.5,
                "roll_max_limit_rad": deg2rad(7.0),
                "roll_min_limit_rad": deg2rad(-7.0),
                "pitch_max_limit_rad": deg2rad(20.0),
                "pitch_min_limit_rad": deg2rad(-30.0),
            }],
        ),

        Node(
            package="joystick_rsu",
            executable="rsu_solver_node.py",
            name="rsu_solver_node",
            output="screen",
            parameters=[{
                # ===== RSU IK params =====
                "a_W_mm_flat": [0.0,  36.0, 170.0,  0.0, -36.0, 82.0],
                "b_F_mm_flat": [-20.0, 36.0, 16.0,  -20.0, -36.0, 16.0],
                "c_mm": [30.0, -30.0],
                "r_mm": [154.0, 66.0],
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

                # # targets + tolerances (meters)
                # "target_len_1_m": 0.1695,
                # "target_len_2_m": 0.0810,
                "len_tol_m": len_tol_m,

                # angle range (deg) - ALL 4 enforced inside node
                "ang_min_deg": ang_min_deg,
                "ang_max_deg": ang_max_deg,

                "tf_log_rate_hz": 1.0,
            }],
        ),

        # Marker plotter (unchanged)
        Node(
            package="joystick_rsu",
            executable="rsu_link_plotter_node",
            name="rsu_link_plotter",
            output="screen",
            parameters=[{
                "world_frame": world_frame,
                "publish_rate_hz": 60.0,
                "radius": 0.004,
                "c1_frame": "point_c1_1",
                "c2_frame": "point_c2_1",
                "u1_frame": "point_u1_1",
                "u2_frame": "point_u2_1",
                "draw_cross_pairs": False,
                "draw_u_bar": False,
                "draw_c_bar": False,
                "target_len_1_m": 0.1547,
                "target_len_2_m": 0.0662,
                "link1_r": 1.0, "link1_g": 0.1, "link1_b": 0.1, "link1_a": 0.9,
                "link2_r": 0.1, "link2_g": 0.4, "link2_b": 1.0, "link2_a": 0.9,
            }],
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_path],
            condition=None,  # keep your original behavior
        ),
    ])




# # LAUNCH FILE FOR DEBUGGING SOLVER NODE (WITH RVIZ2 AND LOGITECH F310 GAMEPAD INPUT)
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# import math


# def deg2rad(d):
#     return d * math.pi / 180.0


# def generate_launch_description():
#     pkg = FindPackageShare("joystick_rsu")

#     urdf_path = PathJoinSubstitution([pkg, "robot_model", "rsu_for_prototype.urdf"])
#     rviz_path = PathJoinSubstitution([pkg, "config", "rsu_2dof.rviz"])

#     world_frame = LaunchConfiguration("world_frame")

#     gate_publish_by_tf_check = LaunchConfiguration("gate_publish_by_tf_check")
#     tf_check_rate_hz = LaunchConfiguration("tf_check_rate_hz")
#     tf_timeout_sec = LaunchConfiguration("tf_timeout_sec")
#     len_tol_m = LaunchConfiguration("len_tol_m")
#     ang_min_deg = LaunchConfiguration("ang_min_deg")
#     ang_max_deg = LaunchConfiguration("ang_max_deg")

#     return LaunchDescription([
#         DeclareLaunchArgument("use_rviz", default_value="true"),
#         DeclareLaunchArgument("world_frame", default_value="base_link"),

#         DeclareLaunchArgument("gate_publish_by_tf_check", default_value="true"),
#         DeclareLaunchArgument("tf_check_rate_hz", default_value="30.0"),
#         DeclareLaunchArgument("tf_timeout_sec", default_value="0.05"),
#         DeclareLaunchArgument("len_tol_m", default_value="0.002"),
#         DeclareLaunchArgument("ang_min_deg", default_value="65.0"),
#         DeclareLaunchArgument("ang_max_deg", default_value="115.0"),

#         # Robot description
#         Node(
#             package="robot_state_publisher",
#             executable="robot_state_publisher",
#             name="robot_state_publisher",
#             output="screen",
#             parameters=[{
#                 "robot_description": Command(["cat ", urdf_path]),
#             }],
#         ),

#         Node(
#             package="joystick_rsu",
#             executable="gamepad_rpy_node.py",
#             name="gamepad_rpy_node",
#             output="screen",
#             parameters=[{
#                 "rate_hz": 50.0,
#                 "wait_first_input": True,
#                 "input_deadzone": 0.05,
#                 "roll_rate_scale": 1.5,
#                 "pitch_rate_scale": 1.5,
#                 "roll_max_limit_rad": deg2rad(15.0),
#                 "roll_min_limit_rad": deg2rad(-15.0),
#                 "pitch_max_limit_rad": deg2rad(30.0),
#                 "pitch_min_limit_rad": deg2rad(-45.0),
#             }],
#         ),

#         Node(
#             package="joystick_rsu",
#             executable="rsu_solver_node.py",
#             name="rsu_solver_node",
#             output="screen",
#             parameters=[{
#                 # ===== RSU IK params =====
#                 "a_W_mm_flat": [0.0,  36.0, 169.5,  0.0, -36.0, 81.0],
#                 "b_F_mm_flat": [-30.0, 36.0, 0.0,  -30.0, -36.0, 0.0],
#                 "c_mm": [30.0, -30.0],
#                 "r_mm": [169.5, 81.0],
#                 "psi_rad": [deg2rad(90.0), deg2rad(-90.0)],

#                 # ===== joints =====
#                 "joint_ankle_pitch": "ankle_pitch",
#                 "joint_ankle_roll": "ankle_roll",
#                 "joint_upper_crank": "upper_crank",
#                 "joint_lower_crank": "lower_crank",

#                 "hold_alpha_on_infeasible": True,

#                 # ===== TF-based crank sanity check params =====
#                 "world_frame": world_frame,
#                 "tf_timeout_sec": tf_timeout_sec,
#                 "tf_check_rate_hz": tf_check_rate_hz,
#                 "gate_publish_by_tf_check": gate_publish_by_tf_check,

#                 # frames (keep identical with plotter)
#                 "c1_frame": "point_c1_1",
#                 "c2_frame": "point_c2_1",
#                 "u1_frame": "point_u1_1",
#                 "u2_frame": "point_u2_1",

#                 # targets + tolerances (meters)
#                 "target_len_1_m": 0.1695,
#                 "target_len_2_m": 0.0810,
#                 "len_tol_m": len_tol_m,

#                 # angle range (deg) - ALL 4 enforced inside node
#                 "ang_min_deg": ang_min_deg,
#                 "ang_max_deg": ang_max_deg,

#                 "tf_log_rate_hz": 1.0,
#             }],
#         ),

#         # Marker plotter (unchanged)
#         Node(
#             package="joystick_rsu",
#             executable="rsu_link_plotter_node",
#             name="rsu_link_plotter",
#             output="screen",
#             parameters=[{
#                 "world_frame": world_frame,
#                 "publish_rate_hz": 60.0,
#                 "radius": 0.004,
#                 "c1_frame": "point_c1_1",
#                 "c2_frame": "point_c2_1",
#                 "u1_frame": "point_u1_1",
#                 "u2_frame": "point_u2_1",
#                 "draw_cross_pairs": False,
#                 "draw_u_bar": False,
#                 "draw_c_bar": False,
#                 "target_len_1_m": 0.1695,
#                 "target_len_2_m": 0.0810,
#                 "link1_r": 1.0, "link1_g": 0.1, "link1_b": 0.1, "link1_a": 0.9,
#                 "link2_r": 0.1, "link2_g": 0.4, "link2_b": 1.0, "link2_a": 0.9,
#             }],
#         ),

#         # RViz
#         Node(
#             package="rviz2",
#             executable="rviz2",
#             name="rviz2",
#             output="screen",
#             arguments=["-d", rviz_path],
#             condition=None,  # keep your original behavior
#         ),
#     ])