#!/usr/bin/env python3
import os, sys
sys.path.insert(0, os.path.dirname(__file__))  # util 모듈 안전 import

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
from util.gamepad_reader import Gamepad

def clamp(x, lo, hi):
    return max(lo, min(hi, x))
def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)



class GamepadRPYNode(Node):
    def __init__(self):
        super().__init__("gamepad_rpy_node")

        # ===== publish rate =====
        self.rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        self.input_deadzone = float(self.declare_parameter("input_deadzone", 0.05).value)
        self.wait_first_input = bool(self.declare_parameter("wait_first_input", True).value)

        # ===== roll/pitch integration params =====
        # 조이스틱 출력([-1,1])을 rad/s로 해석
        self.roll_rate_scale = float(self.declare_parameter("roll_rate_scale", 1.5).value)
        self.pitch_rate_scale = float(self.declare_parameter("pitch_rate_scale", 1.5).value)

        self.roll_max_limit = float(self.declare_parameter("roll_max_limit_rad", math.radians(30.0)).value)
        self.roll_min_limit = float(self.declare_parameter("roll_min_limit_rad", math.radians(-30.0)).value)
        self.pitch_max_limit = float(self.declare_parameter("pitch_max_limit_rad", math.radians(30.0)).value)
        self.pitch_min_limit = float(self.declare_parameter("pitch_min_limit_rad", math.radians(-30.0)).value)

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
        self.pub_cmd = self.create_publisher(Vector3Stamped, "/request_to_solver", 10)

        # ===== ROS SUB =====
        self.sub_feasibility = self.create_subscription(Vector3Stamped, "/solver_answer", self._on_feasibility, 10)
        self.sub_joint_state = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        # ===== state =====
        self.roll = 0.0
        self.pitch = 0.0
        self.prev_roll = 0.0
        self.prev_pitch = 0.0

        self.received_first_input = False
        self.last_t = time.time()
        self.last_debug_t = 0.0

        # ===== timestamp =====
        self._last_stamp = None

        period = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Started. /request_to_solver(Vector3Stamped) @ {self.rate_hz}Hz. "
            f"(x=roll, y=pitch)"
        )
        # ===== current pose feasibility =====
        self.currently_feasible = True

    def publish_rpy(self):
        msg = Vector3Stamped()
        msg.header.stamp = self._last_stamp = self.get_clock().now().to_msg()
        if self.currently_feasible:
            msg.vector.x = float(self.roll)
            msg.vector.y = float(self.pitch)
            msg.vector.z = 0.0
        else:
            msg.vector.x = float(self.prev_roll)
            msg.vector.y = float(self.prev_pitch)
            msg.vector.z = 0.0
        self.pub_cmd.publish(msg)

    def _on_joint_state(self, msg: JointState):
        pass
        # # joint_states 메시지 수신 시 로그 출력 (추후 RViz Marker로 시각화하는 방향으로 개발 고려)
        # joint_info = ", ".join([f"{n}={p:.3f}" for n, p in zip(msg.name, msg.position)])
        # self.get_logger().debug(f"Received /joint_states: {joint_info}")
        # self._last_stamp = msg.header.stamp
    def _on_feasibility(self, msg: Vector3Stamped):
        # feasibility 메시지 수신 시 로그 출력 (추후 RViz Marker로 시각화하는 방향으로 개발 고려)
        feasible = (msg.vector.z > 0.5)
        if stamp_to_ns(self._last_stamp) == stamp_to_ns(msg.header.stamp):
            if feasible:
                self.currently_feasible = True
                # self.get_logger().info("Current pose is FEASIBLE.")
            else:
                self.currently_feasible = False
                # self.get_logger().warn("Current pose is INFEASIBLE.")
        else:
            self.get_logger().error("Feasibility info received but timestamp does not match latest joint state.")
    def on_timer(self):
        # 게임패드 죽으면 0 유지
        if not self.gamepad.is_running:
            self.roll = 0.0
            self.pitch = 0.0
            self.prev_roll = 0.0
            self.prev_pitch = 0.0
            self.received_first_input = False
            self.publish_rpy()
            self.get_logger().error(
                "Gamepad not running. Publishing zeros to /request_to_solver.",
                throttle_duration_sec=2.0,
            )
            return

        now = time.time()
        dt = clamp(now - self.last_t, 0.0, 0.1)  # 기존 그대로
        self.last_t = now

        cmd = self.gamepad.get_command()  # [vx, vy, wz]
        vx = float(cmd[0])  # pitch
        vy = float(cmd[1])  # roll

        mag = max(abs(vx), abs(vy))
        if (not self.received_first_input) and mag > self.input_deadzone:
            self.received_first_input = True
            self.get_logger().info(f"First input detected (mag={mag:.3f}).")

        if self.wait_first_input and (not self.received_first_input):
            # 첫 입력 전: 0 유지 (기존 그대로)
            self.roll = 0.0
            self.pitch = 0.0
            self.prev_roll = 0.0
            self.prev_pitch = 0.0
            self.publish_rpy()
            return

        # integrate roll/pitch (기존 그대로)
        roll_dot = vy * self.roll_rate_scale
        pitch_dot = vx * self.pitch_rate_scale

        self.prev_roll = self.roll
        self.prev_pitch = self.pitch
        self.roll = clamp(self.roll + roll_dot * dt, self.roll_min_limit, self.roll_max_limit)
        self.pitch = clamp(self.pitch + pitch_dot * dt, self.pitch_min_limit, self.pitch_max_limit)

        self.publish_rpy()


def main():
    rclpy.init()
    node = GamepadRPYNode()
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