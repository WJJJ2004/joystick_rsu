from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
		pkg = FindPackageShare("joystick_rsu")

		urdf_path = PathJoinSubstitution([pkg, "robot_model", "rsu_2dof.urdf"])
		rviz_path = PathJoinSubstitution([pkg, "config", "rsu_2dof.rviz"])

		use_rviz = LaunchConfiguration("use_rviz")
		world_frame = LaunchConfiguration("world_frame")

		return LaunchDescription([
				DeclareLaunchArgument("use_rviz", default_value="true"),
				DeclareLaunchArgument("world_frame", default_value="base_link"),

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
						executable="joy_joint_state_node.py",
						name="joy_joint_state_node",
						output="screen",
							parameters=[{
							"a_W_mm_flat": [0.0, 36.0, 169.0,  0.0, -36.0, 81.0],
							"b_F_mm_flat": [-30.0, 36.0, 0.0,  -30.0, -36.0, 0.0],
							"c_mm": [30.0, 30.0],
							"r_mm": [169.5, 81.0],
							"psi_rad": [ -1.57079632679, 1.57079632679 ],  # -90deg, +90deg
						}]
				),
				# Node(
				# 		package="joystick_rsu",
				# 		executable="joy_joint_state_node.py",
				# 		name="joy_joint_state_node",
				# 		output="screen",
				# 		parameters=[{
				# 			"rate_hz": 50.0,
				# 			"joint_ankle_pitch": "ankle_pitch",
				# 			"joint_ankle_roll": "ankle_roll",
				# 			"joint_upper_crank": "upper_crank",
				# 			"joint_lower_crank": "lower_crank",
				# 			"always_publish_4": True,
				# 			"wait_first_input": True,
				# 			"input_deadzone": 0.05,
				# 		}],
				# ),

				# Our marker plotter
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
								"target_len_1_m": 0.1695,
								"target_len_2_m": 0.0810,
								"link1_r": 1.0,"link1_g": 0.1, "link1_b": 0.1, "link1_a": 0.9,
								"link2_r": 0.1, "link2_g": 0.4, "link2_b": 1.0, "link2_a": 0.9,

						}],
				),

				# RViz
				Node(
						condition=None,
						package="rviz2",
						executable="rviz2",
						name="rviz2",
						output="screen",
						arguments=["-d", rviz_path],
				),
		])
