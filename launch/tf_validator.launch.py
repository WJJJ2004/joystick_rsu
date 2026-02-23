# tf_validator.launch.py

"""
how to run

ros2 launch joystick_rsu tf_validator.launch.py world_frame:=base_link

"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    world_frame = LaunchConfiguration("world_frame")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world_frame",
            default_value="base_link",   # 또는 "world" / "map" / 네 TF 트리의 루트
            description="World/Shank frame name (W frame) for RSU solver validation",
        ),
        Node(
            package="joystick_rsu",
            executable="tf_rsu_param_validator.py",
            name="tf_rsu_param_validator",
            output="screen",
            parameters=[{
                "world_frame": "base_link",
                "frame_u0": "dummy_foot_1",   # ★ 교차점(U0)
                "frame_u1": "point_u1_1",
                "frame_u2": "point_u2_1",
                "frame_c1": "point_c1_1",
                "frame_c2": "point_c2_1",
                "urdf_path": urdf_path,       # ★ axis 파싱용
                "b_F_mm_flat": [-30.0, 36.0, 0.0,  -30.0, -36.0, 0.0],
                "r_mm": [170.0, 82.0],
                "tol_mm": 1.0,
            }],
        ),
        # Node(
        #     package="joystick_rsu",
        #     executable="tf_rsu_param_validator.py",
        #     name="tf_rsu_param_validator",
        #     output="screen",
        #     parameters=[{
        #         "frame_W": world_frame,
        #         "frame_foot": "dummy_foot_1",
        #         "frame_c1": "point_c1_1",
        #         "frame_c2": "point_c2_1",
        #         "frame_u1": "point_u1_1",
        #         "frame_u2": "point_u2_1",

        #         "a_W_mm_flat": [0.0,  36.0, 170.0,  0.0, -36.0,  82.0],
        #         "b_F_mm_flat": [-30.0,  36.0, 0.0, -30.0, -36.0, 0.0],
        #         "c_mm": [30.0, 30.0],
        #         "r_mm": [170.0, 82.0],

        #         "tol_mm": 1.0,
        #         "rate_hz": 1.0,
        #     }],
        # ),
    ])