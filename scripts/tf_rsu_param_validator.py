#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET
import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener


def _vec3_from_str(s: str) -> np.ndarray:
    vals = [float(x) for x in s.strip().split()]
    if len(vals) != 3:
        raise RuntimeError(f"Expected 3 floats, got: {s}")
    return np.array(vals, dtype=np.float64)


def _quat_to_R(qx, qy, qz, qw) -> np.ndarray:
    # ROS quaternion (x,y,z,w) -> rotation matrix
    x, y, z, w = qx, qy, qz, qw
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy+zz),     2*(xy-wz),       2*(xz+wy)],
        [2*(xy+wz),         1 - 2*(xx+zz),   2*(yz-wx)],
        [2*(xz-wy),         2*(yz+wx),       1 - 2*(xx+yy)],
    ], dtype=np.float64)


class TFRSUParamValidator(Node):
    """
    Validate RSU params against TF produced by robot_state_publisher + joint_states.

    Key change (per your intent):
      U0 = intersection of ankle_pitch and ankle_roll axes
         = origin of ankle_roll joint
         = origin of dummy_foot_1 link (child of ankle_roll)
    """

    def __init__(self):
        super().__init__("tf_rsu_param_validator")

        # ---- frames (match your URDF) ----
        self.world_frame = self.declare_parameter("world_frame", "base_link").value

        self.frame_u0 = self.declare_parameter("frame_u0", "dummy_foot_1").value
        self.frame_u1 = self.declare_parameter("frame_u1", "point_u1_1").value
        self.frame_u2 = self.declare_parameter("frame_u2", "point_u2_1").value
        self.frame_c1 = self.declare_parameter("frame_c1", "point_c1_1").value
        self.frame_c2 = self.declare_parameter("frame_c2", "point_c2_1").value

        # ---- expected params (from your solver params yaml) ----
        # b_F is in FOOT frame at U0
        b_F_flat = self.declare_parameter(
            "b_F_mm_flat",
            [-30.0,  36.0, 0.0,
             -30.0, -36.0, 0.0]
        ).value
        self.b_F_param = np.array(b_F_flat, dtype=np.float64).reshape(2, 3)

        # rod lengths expected
        r_list = self.declare_parameter("r_mm", [170.0, 82.0]).value
        self.r_param = np.array(r_list, dtype=np.float64).reshape(2,)

        # tolerance
        self.tol_mm = float(self.declare_parameter("tol_mm", 1.0).value)

        # ---- URDF parsing (for axis sign print / sanity) ----
        self.urdf_path = self.declare_parameter("urdf_path", "").value
        self.joint_axis = {}
        if self.urdf_path:
            try:
                self._parse_urdf_axes(self.urdf_path)
            except Exception as e:
                self.get_logger().warn(f"Failed to parse URDF axes: {e}")

        # ---- TF ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f"TF RSU Param Validator started. world_frame={self.world_frame}, "
            f"U0(frame)={self.frame_u0} (roll/pitch axis intersection)."
        )

        if self.joint_axis:
            up = self.joint_axis.get("upper_crank", None)
            lo = self.joint_axis.get("lower_crank", None)
            ap = self.joint_axis.get("ankle_pitch", None)
            ar = self.joint_axis.get("ankle_roll", None)
            self.get_logger().info(f"[URDF axis] ankle_pitch={ap}, ankle_roll={ar}, upper_crank={up}, lower_crank={lo}")

    def _parse_urdf_axes(self, urdf_path: str):
        tree = ET.parse(urdf_path)
        root = tree.getroot()

        for j in root.findall("joint"):
            name = j.get("name", "")
            axis = j.find("axis")
            if axis is not None and axis.get("xyz") is not None:
                a = _vec3_from_str(axis.get("xyz"))
                self.joint_axis[name] = a

    def _lookup(self, target: str, source: str):
        # target_frame <- source_frame
        return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())

    def on_timer(self):
        try:
            # positions in meters in world_frame
            T_w_u0 = self._lookup(self.world_frame, self.frame_u0)
            T_w_u1 = self._lookup(self.world_frame, self.frame_u1)
            T_w_u2 = self._lookup(self.world_frame, self.frame_u2)
            T_w_c1 = self._lookup(self.world_frame, self.frame_c1)
            T_w_c2 = self._lookup(self.world_frame, self.frame_c2)

            pW_u0 = np.array([T_w_u0.transform.translation.x,
                              T_w_u0.transform.translation.y,
                              T_w_u0.transform.translation.z], dtype=np.float64)
            pW_u1 = np.array([T_w_u1.transform.translation.x,
                              T_w_u1.transform.translation.y,
                              T_w_u1.transform.translation.z], dtype=np.float64)
            pW_u2 = np.array([T_w_u2.transform.translation.x,
                              T_w_u2.transform.translation.y,
                              T_w_u2.transform.translation.z], dtype=np.float64)
            pW_c1 = np.array([T_w_c1.transform.translation.x,
                              T_w_c1.transform.translation.y,
                              T_w_c1.transform.translation.z], dtype=np.float64)
            pW_c2 = np.array([T_w_c2.transform.translation.x,
                              T_w_c2.transform.translation.y,
                              T_w_c2.transform.translation.z], dtype=np.float64)

            # R_WF = rotation of foot frame (dummy_foot_1) in world
            q = T_w_u0.transform.rotation
            R_WF = _quat_to_R(q.x, q.y, q.z, q.w)

            # ---- rod length check ----
            L1_mm = float(np.linalg.norm(pW_c1 - pW_u1) * 1000.0)
            L2_mm = float(np.linalg.norm(pW_c2 - pW_u2) * 1000.0)
            e1 = L1_mm - float(self.r_param[0])
            e2 = L2_mm - float(self.r_param[1])

            # ---- b_F reconstruction ----
            # b_i_F = R_WF^T * (U_i - U0)
            b1_F = (R_WF.T @ (pW_u1 - pW_u0)) * 1000.0
            b2_F = (R_WF.T @ (pW_u2 - pW_u0)) * 1000.0

            # compare with param
            d1 = float(np.linalg.norm(b1_F - self.b_F_param[0]))
            d2 = float(np.linalg.norm(b2_F - self.b_F_param[1]))
            ok_b = (d1 <= self.tol_mm) and (d2 <= self.tol_mm)
            ok_r = (abs(e1) <= self.tol_mm) and (abs(e2) <= self.tol_mm)

            # ---- logs ----
            self.get_logger().info(
                f"[U0] pW_u0={pW_u0} m  (u1={pW_u1} m, u2={pW_u2} m)"
            )
            self.get_logger().info(
                f"[TF] L1=||C1-U1||={L1_mm:7.2f} mm (err {e1:+7.2f}) | "
                f"L2=||C2-U2||={L2_mm:7.2f} mm (err {e2:+7.2f})"
            )
            self.get_logger().info(
                f"[TF->b_F] b1_F={b1_F.round(3)} mm, diff={d1:6.2f} | "
                f"b2_F={b2_F.round(3)} mm, diff={d2:6.2f}"
            )
            self.get_logger().info(
                f"[OK?] b_F:{ok_b}  rod(r):{ok_r} (tol={self.tol_mm:.2f} mm)"
            )

            # helpful reminder about crank axis sign
            if self.joint_axis:
                up = self.joint_axis.get("upper_crank", None)
                lo = self.joint_axis.get("lower_crank", None)
                if up is not None and lo is not None:
                    # just show sign along Y
                    self.get_logger().info(
                        f"[AxisSign] upper_crank axis={up} | lower_crank axis={lo} "
                        f"(note: lower is -Y in URDF)"
                    )

        except Exception as e:
            self.get_logger().warn(f"TF lookup/compute failed: {e}", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = TFRSUParamValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()