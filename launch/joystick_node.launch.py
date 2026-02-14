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
							"rate_hz": 50.0,
							"joint_ankle_pitch": "ankle_pitch",
							"joint_ankle_roll": "ankle_roll",
							"joint_upper_crank": "upper_crank",
							"joint_lower_crank": "lower_crank",
							"always_publish_4": True,
							"wait_first_input": True,
							"input_deadzone": 0.05,
						}],
				),
		])
