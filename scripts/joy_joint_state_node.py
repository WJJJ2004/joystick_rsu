#!/usr/bin/env python3
import os, sys
sys.path.insert(0, os.path.dirname(__file__))  # gamepad_reader 같은 로컬 모듈 안전

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from util.gamepad_reader import Gamepad

# RSU solver python binding
from util.rsu_solver import RSUParams, RSUSolver


def clamp(x, lo, hi):
	return max(lo, min(hi, x))


def wrap_to_pi(a):
	while a >= math.pi:
		a -= 2.0 * math.pi
	while a < -math.pi:
		a += 2.0 * math.pi
	return a


def deg2rad(d): 
	return d * math.pi / 180.0


class JoyJointStateNode(Node):
	def __init__(self):
		super().__init__("joy_joint_state_node")

		# ===== publish rate =====
		self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
		self.input_deadzone = float(self.declare_parameter("input_deadzone", 0.05).value)
		self.wait_first_input = bool(self.declare_parameter("wait_first_input", True).value)

		# ===== joints (URDF names) =====
		self.joint_ankle_pitch = str(self.declare_parameter("joint_ankle_pitch", "ankle_pitch").value)
		self.joint_ankle_roll  = str(self.declare_parameter("joint_ankle_roll", "ankle_roll").value)
		self.joint_upper_crank = str(self.declare_parameter("joint_upper_crank", "upper_crank").value)
		self.joint_lower_crank = str(self.declare_parameter("joint_lower_crank", "lower_crank").value)

		# ===== roll/pitch integration params =====
		# 조이스틱 출력([-1,1])을 rad/s로 해석
		self.roll_rate_scale  = float(self.declare_parameter("roll_rate_scale", 1.5).value)
		self.pitch_rate_scale = float(self.declare_parameter("pitch_rate_scale", 1.5).value)

		self.roll_limit  = float(self.declare_parameter("roll_limit_rad",  math.radians(30.0)).value)
		self.pitch_limit = float(self.declare_parameter("pitch_limit_rad", math.radians(30.0)).value)
		self.prev_alpha_solver = np.array([0.0, 0.0], dtype=np.float64)


		a_W_flat = self.declare_parameter(
			"a_W_mm_flat",
			[0.0,  36.0, 170.0,
			0.0, -36.0,  82.0]
		).value

		b_F_flat = self.declare_parameter(
			"b_F_mm_flat",
			[-30.0,  36.0, 0.0,
			-30.0, -36.0, 0.0]
		).value

		c_list = self.declare_parameter("c_mm", [30.0, -30.0]).value
		r_list = self.declare_parameter("r_mm", [170.0, 82.0]).value
		

		psi_list = self.declare_parameter(
			"psi_rad",
			[deg2rad(90.0), deg2rad(-90.0)]
		).value

		# print all params for sanity check
		self.get_logger().info(
			f"RSU Params:\n"
			f"  a_W_mm_flat: {a_W_flat}\n"
			f"  b_F_mm_flat: {b_F_flat}\n"
			f"  c_mm: {c_list}\n"
			f"  r_mm: {r_list}\n"
			f"  psi_rad: {psi_list}"
		)


		# 타입/길이 검증(터지면 원인 바로 보이게)
		if len(a_W_flat) != 6:
			raise RuntimeError(f"a_W_mm_flat must have length 6, got {len(a_W_flat)}")
		if len(b_F_flat) != 6:
			raise RuntimeError(f"b_F_mm_flat must have length 6, got {len(b_F_flat)}")
		if len(c_list) != 2 or len(r_list) != 2 or len(psi_list) != 2:
			raise RuntimeError(f"c_mm/r_mm/psi_rad must have length 2 (got c={len(c_list)}, r={len(r_list)}, psi={len(psi_list)})")

		self.a_W = np.array(a_W_flat, dtype=np.float64).reshape(2, 3)
		self.b_F = np.array(b_F_flat, dtype=np.float64).reshape(2, 3)
		self.c   = np.array(c_list, dtype=np.float64).reshape(2,)
		self.r   = np.array(r_list, dtype=np.float64).reshape(2,)
		self.psi = np.array(psi_list, dtype=np.float64).reshape(2,)

		# solver continuity용 prev alpha
		self.prev_alpha = np.array([0.0, 0.0], dtype=np.float64)

		# infeasible 처리(지금은: infeasible이면 alpha 유지)
		self.hold_alpha_on_infeasible = bool(
			self.declare_parameter("hold_alpha_on_infeasible", True).value
		)

		# ===== init solver =====
		# 회전 방향 반대 처리
		# p = RSUParams(
		# 	a_W=self.a_W,
		# 	b_F=self.b_F,
		# 	c=self.c,
		# 	r=self.r,
		# 	psi=self.psi,
		# )

		p = RSUParams(
    a_W=np.array([[0,  36, 169.5],
                  [0, -36,  81]]),
    b_F=np.array([[-30,  36, 0],
                  [-30, -36, 0]]),
    c=np.array([30, -30]),
    r=np.array([169.5, 81.0]),
    psi=np.array([deg2rad(90), deg2rad(-90)]),
)
		self.solver = RSUSolver(p)

		# ===== Gamepad =====
		self.device_path = str(self.declare_parameter("device_path", "").value)
		self.vendor_id = int(self.declare_parameter("vendor_id", 0x046D).value)
		self.product_id = int(self.declare_parameter("product_id", 0xC219).value)

		self.gamepad = Gamepad(
			vendor_id=self.vendor_id,
			product_id=self.product_id,
			vel_scale_x=1.0,
			vel_scale_y=1.0,
			vel_scale_rot=1.0,
			device_path=(self.device_path if self.device_path else None),
			prefer_name_contains="Logitech",
		)

		# ===== ROS pub =====
		self.pub = self.create_publisher(JointState, "/joint_states", 10)

		# ===== state =====
		self.roll = 0.0
		self.pitch = 0.0
		self.alpha1 = 0.0
		self.alpha2 = 0.0

		self.received_first_input = False
		self.last_t = time.time()
		self.last_debug_t = 0.0

		period = 1.0 / max(1.0, self.rate_hz)
		self.timer = self.create_timer(period, self.on_timer)

		self.get_logger().info(
			f"Started. /joint_states @ {self.rate_hz}Hz. "
			f"roll/pitch only by joystick, cranks by RSU solver."
		)

	def publish_joint_states(self):
		msg = JointState()
		msg.header.stamp = self.get_clock().now().to_msg()

		# 항상 4개를 채워서 publish
		msg.name = [
			self.joint_ankle_pitch,
			self.joint_ankle_roll,
			self.joint_upper_crank,
			self.joint_lower_crank,
		]
		msg.position = [
			float(self.pitch),   # ankle_pitch
			float(self.roll),    # ankle_roll
			float(self.alpha1),  # upper_crank (actuator1)
			float(self.alpha2),  # lower_crank (actuator2)
		]
		self.pub.publish(msg)

		# 1Hz 디버그
		now = time.time()
		if now - self.last_debug_t > 1.0:
			self.last_debug_t = now
			self.get_logger().info(
				f"RP=(roll={self.roll:+.3f}, pitch={self.pitch:+.3f}) rad | "
				f"alpha=({self.alpha1:+.3f}, {self.alpha2:+.3f}) rad"
			)

	def on_timer(self):
		# 게임패드 죽으면 0 유지(발목/크랭크 다 0)로 TF 안정화
		if not self.gamepad.is_running:
			self.roll = 0.0
			self.pitch = 0.0
			self.alpha1 = 0.0
			self.alpha2 = 0.0
			self.prev_alpha[:] = [0.0, 0.0]
			self.publish_joint_states()
			self.get_logger().error(
				"Gamepad not running. Publishing zeros.",
				throttle_duration_sec=2.0
			)
			return

		now = time.time()
		dt = clamp(now - self.last_t, 0.0, 0.1)
		self.last_t = now

		cmd = self.gamepad.get_command()  # [vx, vy, wz]
		vx = float(cmd[0])  # pitch
		vy = float(cmd[1])  # roll
		# vy = 0.0

		mag = max(abs(vx), abs(vy))
		if (not self.received_first_input) and mag > self.input_deadzone:
			self.received_first_input = True
			self.get_logger().info(f"First input detected (mag={mag:.3f}).")

		if self.wait_first_input and (not self.received_first_input):
			# 첫 입력 전: 0 유지
			self.roll = 0.0
			self.pitch = 0.0
			self.alpha1 = 0.0
			self.alpha2 = 0.0
			self.prev_alpha[:] = [0.0, 0.0]
			self.publish_joint_states()
			return

		# integrate roll/pitch
		roll_dot  = vy * self.roll_rate_scale
		pitch_dot = vx * self.pitch_rate_scale

		self.roll  = clamp(self.roll  + roll_dot  * dt, -self.roll_limit,  self.roll_limit)
		self.pitch = clamp(self.pitch + pitch_dot * dt, -self.pitch_limit, self.pitch_limit)

		# RSU solve -> alpha
		# TODO ROLL / PITCH 
		res = self.solver.solve(self.roll, self.pitch, self.prev_alpha_solver)

		if bool(res.feasible):
			a_solver = np.array(res.alpha, dtype=np.float64).reshape(2,)
			self.alpha1 =  float(a_solver[0])   # 1번 축 반대라면
			self.alpha2 =  float(a_solver[1])
			self.prev_alpha_solver[:] = a_solver
		else:
			# infeasible이면: alpha 유지(또는 0으로)
			if not self.hold_alpha_on_infeasible:
				self.alpha1 = 0.0
				self.alpha2 = 0.0
				self.prev_alpha_solver[:] = [0.0, 0.0]

			self.get_logger().warn(
				f"RSU infeasible for roll={self.roll:+.3f}, pitch={self.pitch:+.3f}. Holding alpha.",
				throttle_duration_sec=0.5
			)

		self.publish_joint_states()


def main():
	rclpy.init()
	node = JoyJointStateNode()
	try:
		rclpy.spin(node)
	finally:
		try:
			node.gamepad.stop()
		except Exception:
			pass
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()
