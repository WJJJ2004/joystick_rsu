#!/usr/bin/env python3

from enum import Enum
import os, sys
sys.path.insert(0, os.path.dirname(__file__))

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import JointState
# from std_msgs.msg import Float32MultiArray
from roa_interfaces.msg import RsuTarget, RsuSolution, MotorStateArray,RsuStateArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from tf2_ros import Buffer, TransformListener

# RSU solver python binding
from util.rsu_solver import RSUParams, RSUSolver
from util.rsu_state_estimator import RSUStateEstimator, RSUStateEstimatorConfig

class CONTROL_MODE(Enum):
    RT_CONTROL = 1
    DEBUG = 2

def deg2rad(d):
    return d * math.pi / 180.0

def stamp_to_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def norm3(v: np.ndarray) -> float:
    return float(np.linalg.norm(v))


def unit3(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = norm3(v)
    if n < eps:
        return np.zeros(3, dtype=np.float64)
    return v / n


def quat_to_R(qx, qy, qz, qw):
    # geometry_msgs quaternion (x,y,z,w) -> 3x3 rotation matrix
    x, y, z, w = qx, qy, qz, qw
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], dtype=np.float64)


def angle_between(u: np.ndarray, v: np.ndarray, eps: float = 1e-12) -> float:
    uu = unit3(u, eps)
    vv = unit3(v, eps)
    d = float(np.dot(uu, vv))
    d = clamp(d, -1.0, 1.0)
    return math.acos(d)  # rad


def rad2deg(r):
    return r * 180.0 / math.pi


class RSUSolverNode(Node):
    def __init__(self):
        super().__init__("rsu_solver_node")

        rt = bool(self.declare_parameter("REALTIME_CONTROL_MODE", False).value)
        self._control_mode = CONTROL_MODE.RT_CONTROL if rt else CONTROL_MODE.DEBUG

        # ===== joints (URDF names) =====
        self.joint_ankle_pitch = str(self.declare_parameter("joint_ankle_pitch", "ankle_pitch").value)
        self.joint_ankle_roll  = str(self.declare_parameter("joint_ankle_roll",  "ankle_roll").value)
        self.joint_upper_crank = str(self.declare_parameter("joint_upper_crank", "upper_crank").value)
        self.joint_lower_crank = str(self.declare_parameter("joint_lower_crank", "lower_crank").value)

        # infeasible 처리: infeasible이면 alpha 유지 or (사용자 정책대로)
        self.hold_alpha_on_infeasible = bool(self.declare_parameter("hold_alpha_on_infeasible", True).value)

        # ===== RSU Params =====
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

        # ======= RSU State Estimator Params =======
        left_ac1_id  = self.declare_parameter("left_ac1_id", 18).value
        left_ac2_id  = self.declare_parameter("left_ac2_id", 20).value
        right_ac1_id = self.declare_parameter("right_ac1_id", 19).value
        right_ac2_id = self.declare_parameter("right_ac2_id", 21).value

        self.motor_state = {
            "left_ac1": {"pos": None, "vel": None, "id": left_ac1_id},
            "left_ac2": {"pos": None, "vel": None, "id": left_ac2_id},
            "right_ac1": {"pos": None, "vel": None, "id": right_ac1_id},
            "right_ac2": {"pos": None, "vel": None, "id": right_ac2_id},
        }

        jac_lambda = self.declare_parameter("jac_lambda", 1e-6).value
        jac_h = self.declare_parameter("jac_h", 5e-5).value
        beta_jac = self.declare_parameter("beta_jac", 1.0).value
        vel_lpf_tau = self.declare_parameter("vel_lpf_tau", 0.0).value
        motor_vel_lpf_tau = self.declare_parameter("motor_vel_lpf_tau",0.0).value

        self.get_logger().info(
            f"RSU Params:\n"
            f"  a_W_mm_flat: {a_W_flat}\n"
            f"  b_F_mm_flat: {b_F_flat}\n"
            f"  c_mm: {c_list}\n"
            f"  r_mm: {r_list}\n"
            f"  psi_rad: {psi_list}"
            f"  jac_h: {jac_h}\n"
            f"  beta_jac: {beta_jac}\n"
            f"  vel_lpf_tau: {vel_lpf_tau}\n"
            f"  motor_vel_lpf_tau: {motor_vel_lpf_tau}\n"
        )

        if len(a_W_flat) != 6:
            raise RuntimeError(f"a_W_mm_flat must have length 6, got {len(a_W_flat)}")
        if len(b_F_flat) != 6:
            raise RuntimeError(f"b_F_mm_flat must have length 6, got {len(b_F_flat)}")
        if len(c_list) != 2 or len(r_list) != 2 or len(psi_list) != 2:
            raise RuntimeError(
                f"c_mm/r_mm/psi_rad must have length 2 "
                f"(got c={len(c_list)}, r={len(r_list)}, psi={len(psi_list)})"
            )

        a_W = np.array(a_W_flat, dtype=np.float64).reshape(2, 3)
        b_F = np.array(b_F_flat, dtype=np.float64).reshape(2, 3)
        c   = np.array(c_list, dtype=np.float64).reshape(2,)
        r   = np.array(r_list, dtype=np.float64).reshape(2,)
        psi = np.array(psi_list, dtype=np.float64).reshape(2,)

        self._last_seq = None  # uint32 monotonic check

        p = RSUParams(a_W=a_W, b_F=b_F, c=c, r=r, psi=psi)
        self.solver = RSUSolver(p)

        # self.estimator = RSUStateEstimator(
        #     solver=self.solver,
        #     cfg=RSUStateEstimatorConfig(
        #             max_iter=5,
        #             lambda_pos=1e-6,
        #             jac_h=1e-5,
        #             jac_lambda = jac_lambda,
        #             jac_h = jac_h,
        #             beta_jac = beta_jac,
        #             vel_lpf_tau = vel_lpf_tau,
        #             motor_vel_lpf_tau = motor_vel_lpf_tau,
        #             cond_warn=50.0,
        #             cond_fail=200.0,
        #             sigma_min_thresh=1e-5,
        #             q_init=np.array([0.0, 0.0]),
        #         )
        # )

        est_cfg = RSUStateEstimatorConfig(
            max_iter=5,
            lambda_pos=1e-6,
            jac_h=jac_h,
            jac_lambda=jac_lambda,
            beta_jac=beta_jac,
            vel_lpf_tau=vel_lpf_tau,
            motor_vel_lpf_tau=motor_vel_lpf_tau,
            cond_warn=50.0,
            cond_fail=200.0,
            sigma_min_thresh=1e-5,
            q_init=np.array([0.0, 0.0]),
        )

        self.left_estimator = RSUStateEstimator(
            solver=self.solver,
            cfg=est_cfg,
        )

        self.right_estimator = RSUStateEstimator(
            solver=self.solver,
            cfg=est_cfg,
        )

        self._last_motor_state_stamp_ns = None
        self._rsu_state_seq = 0
        self.rsu_state_frame_id = str(
            self.declare_parameter("rsu_state_frame_id", "base_link").value
        )

        # solver continuity용 prev alpha
        self.prev_alpha_1D = np.array([0.0, 0.0], dtype=np.float64)

        """
            [L_alpha1, L_alpha2],   # left foot
            [R_alpha1, R_alpha2]    # right foot
        """

        self.prev_alpha_2D = np.zeros((2, 2), dtype=np.float64)

        # ===== state =====
        self.roll = 0.0
        self.pitch = 0.0
        self.alpha1 = 0.0
        self.alpha2 = 0.0

        self.last_debug_t = 0.0

        # ===== TF-based sanity checks (ported from C++ plotter) =====
        self.world_frame = str(self.declare_parameter("world_frame", "base_link").value)
        self.tf_timeout_sec = float(self.declare_parameter("tf_timeout_sec", 0.05).value)

        self.c1_frame = str(self.declare_parameter("c1_frame", "point_c1_1").value)
        self.c2_frame = str(self.declare_parameter("c2_frame", "point_c2_1").value)
        self.u1_frame = str(self.declare_parameter("u1_frame", "point_u1_1").value)
        self.u2_frame = str(self.declare_parameter("u2_frame", "point_u2_1").value)

        # Target lengths (meters) - same defaults as C++ plotter
        # self.target_len_1_m = float(self.declare_parameter("target_len_1_m", 0.1695).value)
        # self.target_len_2_m = float(self.declare_parameter("target_len_2_m", 0.0810).value)
        self.target_len_1_m = float(r[0]) / 1000.0
        self.target_len_2_m = float(r[1]) / 1000.0

        # tolerance (meters): 2mm default
        self.len_tol_m = float(self.declare_parameter("len_tol_m", 0.002).value)

        # angle range (degrees): 70~110 default
        self.ang_min_deg = float(self.declare_parameter("ang_min_deg", 70.0).value)
        self.ang_max_deg = float(self.declare_parameter("ang_max_deg", 110.0).value)

        # gating behavior
        self.gate_publish_by_tf_check = bool(self.declare_parameter("gate_publish_by_tf_check", True).value)

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # latest check result latch
        self._tf_check_ok = False
        self._tf_check_diag = "not computed yet"

        # ===== ROS pub/sub =====
        self._last_stamp = None
        # TF2용 joint state pubsliher
        self.pub_joint_state = self.create_publisher(
            JointState, "/joint_states", 10
        )
        if self._control_mode == CONTROL_MODE.DEBUG:
            # 디버그 모드용 조이패드에서 읽어온 누적 커맨드 적분값 입력 (*왼발만 해당)
            self.pub_solver_respond = self.create_publisher(
                Vector3Stamped, "/solver_answer", 10
            )
            # 디버그 모드용 조이패드에서 읽어온 누적 커맨드 적분값 입력 (*왼발만 해당)
            self.sub_solver_request = self.create_subscription(
                Vector3Stamped, "/request_to_solver", self._on_rpy, 10
            )
        elif self._control_mode == CONTROL_MODE.RT_CONTROL:
            rsu_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            motor_status_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            # 실시간 제어에서 활용할 좌우 발에서 계산한 해 (* 양발)
            self.pub_both_foot_solution = self.create_publisher(
                RsuSolution, "/rsu/solution" ,rsu_qos
            )

            # 실시간 제어용 > 제어기에서 받아온 목표 위치값 좌우 발 모두 해당함 (* 양발)
            self.sub_both_foot_request = self.create_subscription(
                RsuTarget, "/rsu/target" , self._on_both_foot_request ,rsu_qos
            )
            self.sub_motor_status = self.create_subscription(
                MotorStateArray, "/hardware_interface/state" , self._on_motor_state , motor_status_qos
            )
            self.pub_rsu_state = self.create_publisher(
                RsuStateArray, "/rsu/state",rsu_qos
            )

        self.get_logger().info(
            "Started. Subscribing /request_to_solver, publishing /joint_states.\n"
            f"TF check: world={self.world_frame}, frames: C1={self.c1_frame} U1={self.u1_frame} "
            f"| C2={self.c2_frame} U2={self.u2_frame}\n"
            f"Targets: L1={self.target_len_1_m*1000:.1f}mm, L2={self.target_len_2_m*1000:.1f}mm, "
            f"tol={self.len_tol_m*1000:.1f}mm, ang=[{self.ang_min_deg:.1f},{self.ang_max_deg:.1f}]deg (ALL 4)"
        )

    @property
    def control_mode(self) -> CONTROL_MODE:
        return self._control_mode

    @control_mode.setter
    def control_mode(self, _):
        raise AttributeError("control_mode is read-only (set at init).")

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self._last_stamp
        msg.name = [
            self.joint_ankle_pitch,
            self.joint_ankle_roll,
            self.joint_upper_crank,
            self.joint_lower_crank,
        ]
        msg.position = [
            float(self.pitch),   # ankle_pitch
            float(self.roll),    # ankle_roll
            float(self.alpha1),  # upper_crank
            float(self.alpha2),  # lower_crank
        ]
        self.pub_joint_state.publish(msg)

        # 1Hz 디버그
        now = time.time()
        if now - self.last_debug_t > 1.0:
            self.last_debug_t = now
            self.get_logger().info(
                f"RP=(roll={self.roll:+.3f}, pitch={self.pitch:+.3f}) rad | "
                f"alpha=({self.alpha1:+.3f}, {self.alpha2:+.3f}) rad | "
                f"TF_OK={self._tf_check_ok}"
            )

    def _on_motor_state(self, msg: MotorStateArray):
        # 1) 최신 모터 상태 캐시 갱신
        for st in msg.states:
            motor_id = int(st.motor_id)

            if motor_id == self.motor_state["left_ac1"]["id"]:
                self.motor_state["left_ac1"]["pos"] = float(st.position)
                self.motor_state["left_ac1"]["vel"] = float(st.velocity)

            elif motor_id == self.motor_state["left_ac2"]["id"]:
                self.motor_state["left_ac2"]["pos"] = float(st.position)
                self.motor_state["left_ac2"]["vel"] = float(st.velocity)

            elif motor_id == self.motor_state["right_ac1"]["id"]:
                self.motor_state["right_ac1"]["pos"] = float(st.position)
                self.motor_state["right_ac1"]["vel"] = float(st.velocity)

            elif motor_id == self.motor_state["right_ac2"]["id"]:
                self.motor_state["right_ac2"]["pos"] = float(st.position)
                self.motor_state["right_ac2"]["vel"] = float(st.velocity)

        # 2) 4개 모터 상태가 모두 준비될 때까지 대기
        required_keys = ["left_ac1", "left_ac2", "right_ac1", "right_ac2"]
        for key in required_keys:
            if self.motor_state[key]["pos"] is None or self.motor_state[key]["vel"] is None:
                return

        # 3) dt 계산
        stamp_ns = stamp_to_ns(msg.header.stamp)

        if self._last_motor_state_stamp_ns is None:
            self._last_motor_state_stamp_ns = stamp_ns
            return

        dt = (stamp_ns - self._last_motor_state_stamp_ns) * 1e-9
        self._last_motor_state_stamp_ns = stamp_ns

        if not np.isfinite(dt) or dt <= 0.0:
            self.get_logger().warn(
                f"Invalid dt from /hardware_interface/state: dt={dt}. Ignoring message."
            )
            return

        # 4) 좌/우 발 입력 벡터 구성 (raw hardware convention)
        left_motor_pos = np.array([
            self.motor_state["left_ac1"]["pos"],
            self.motor_state["left_ac2"]["pos"],
        ], dtype=np.float64)

        left_motor_vel = np.array([
            self.motor_state["left_ac1"]["vel"],
            self.motor_state["left_ac2"]["vel"],
        ], dtype=np.float64)

        right_motor_pos = np.array([
            self.motor_state["right_ac1"]["pos"],
            self.motor_state["right_ac2"]["pos"],
        ], dtype=np.float64)

        right_motor_vel = np.array([
            self.motor_state["right_ac1"]["vel"],
            self.motor_state["right_ac2"]["vel"],
        ], dtype=np.float64)

        # 5) estimator 호출
        left_state, left_q, left_qd = self._estimate_one_foot(
            estimator=self.left_estimator,
            motor_pos=left_motor_pos,
            motor_vel=left_motor_vel,
            dt=dt,
            mirror=False,
        )

        right_state, right_q, right_qd = self._estimate_one_foot(
            estimator=self.right_estimator,
            motor_pos=right_motor_pos,
            motor_vel=right_motor_vel,
            dt=dt,
            mirror=True,
        )

        # 6) 출력 메시지 구성
        out = RsuStateArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.rsu_state_frame_id
        out.seq = self._rsu_state_seq
        self._rsu_state_seq += 1

        out.q_dot.left_rsu_roll = float(left_qd[0])
        out.q_dot.left_rsu_pitch = float(left_qd[1])
        out.q_dot.right_rsu_roll = float(right_qd[0])
        out.q_dot.right_rsu_pitch = float(right_qd[1])

        out.q.left_rsu_roll = float(left_q[0])
        out.q.left_rsu_pitch = float(left_q[1])
        out.q.right_rsu_roll = float(right_q[0])
        out.q.right_rsu_pitch = float(right_q[1])

        out.feasible = bool(
            left_state.feasible and right_state.feasible and
            left_state.valid and right_state.valid
        )

        self.pub_rsu_state.publish(out)

        # 7) degraded / invalid 상태 로깅
        if (not left_state.valid) or (not right_state.valid):
            self.get_logger().warn(
                "[RSU estimator] invalid state | "
                f"L(feasible={left_state.feasible}, valid={left_state.valid}, "
                f"res={left_state.residual_norm:.3e}, cond={left_state.condJ:.3f}, sigma_min={left_state.sigma_min:.3e}) | "
                f"R(feasible={right_state.feasible}, valid={right_state.valid}, "
                f"res={right_state.residual_norm:.3e}, cond={right_state.condJ:.3f}, sigma_min={right_state.sigma_min:.3e})"
            )
        elif left_state.degraded or right_state.degraded:
            self.get_logger().warn(
                "[RSU estimator] degraded state | "
                f"L(res={left_state.residual_norm:.3e}, cond={left_state.condJ:.3f}, sigma_min={left_state.sigma_min:.3e}) | "
                f"R(res={right_state.residual_norm:.3e}, cond={right_state.condJ:.3f}, sigma_min={right_state.sigma_min:.3e})"
            )

    def _estimate_one_foot(self, estimator, motor_pos, motor_vel, dt, mirror=False):
        motor_pos = np.array(motor_pos, dtype=np.float64)
        motor_vel = np.array(motor_vel, dtype=np.float64)

        if mirror:
            motor_pos = -motor_pos
            motor_vel = -motor_vel

        state = estimator.update(motor_pos, motor_vel, dt)

        q = np.array(state.q_rel, dtype=np.float64)
        qd = np.array(state.qd_rel, dtype=np.float64)

        if mirror:
            q = -q
            qd = -qd

        return state, q, qd

    def publish_solver_respond(self, feasible: bool):
        msg = Vector3Stamped()
        msg.header.stamp = self._last_stamp
        if feasible:
            msg.vector.x = self.alpha1
            msg.vector.y = self.alpha2
            msg.vector.z = 1.0
        else:
            msg.vector.x = self.alpha1  # or 0.0, depending on your preference
            msg.vector.y = self.alpha2  # or 0.0, depending on your preference
            msg.vector.z = 0.0
        self.pub_solver_respond.publish(msg)

    # ---------- TF check helpers ----------
    def _lookup_tf(self, target_frame: str):
        try:
            # latest available
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
            return tf
        except Exception:
            return None

    def _tf_to_p_R(self, tfmsg):
        t = tfmsg.transform.translation
        q = tfmsg.transform.rotation  # x,y,z,w
        p = np.array([t.x, t.y, t.z], dtype=np.float64)
        R = quat_to_R(q.x, q.y, q.z, q.w)
        return p, R

    def _in_ang_range(self, ang_deg: float) -> bool:
        return (self.ang_min_deg <= ang_deg <= self.ang_max_deg)

    def tf_hardware_safetycheck(self):
        tf_c1 = self._lookup_tf(self.c1_frame)
        tf_u1 = self._lookup_tf(self.u1_frame)
        tf_c2 = self._lookup_tf(self.c2_frame)
        tf_u2 = self._lookup_tf(self.u2_frame)

        if tf_c1 is None or tf_u1 is None or tf_c2 is None or tf_u2 is None:
            self._tf_check_ok = False
            self._tf_check_diag = "TF missing"
            raise RuntimeError(f"TF lookup failed for frames: "
                f"C1={'OK' if tf_c1 else 'MISSING'} "
                f"U1={'OK' if tf_u1 else 'MISSING'} "
                f"C2={'OK' if tf_c2 else 'MISSING'} "
                f"U2={'OK' if tf_u2 else 'MISSING'}"
            )

        pc1, Rc1 = self._tf_to_p_R(tf_c1)
        pu1, Ru1 = self._tf_to_p_R(tf_u1)
        pc2, Rc2 = self._tf_to_p_R(tf_c2)
        pu2, Ru2 = self._tf_to_p_R(tf_u2)

        # (1) Length check
        L1 = norm3(pc1 - pu1)
        L2 = norm3(pc2 - pu2)
        err1 = abs(L1 - self.target_len_1_m)
        err2 = abs(L2 - self.target_len_2_m)
        ok_len = (err1 <= self.len_tol_m) and (err2 <= self.len_tol_m)

        # (2) Ball Joint Angle check (ALL 4)
        y_c1 = Rc1 @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        y_u1 = Ru1 @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        y_c2 = Rc2 @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        y_u2 = Ru2 @ np.array([0.0, 1.0, 0.0], dtype=np.float64)

        ang_c1_deg = rad2deg(angle_between(y_c1, (pu1 - pc1)))
        ang_u1_deg = rad2deg(angle_between(y_u1, (pc1 - pu1)))
        ang_c2_deg = rad2deg(angle_between(y_c2, (pu2 - pc2)))
        ang_u2_deg = rad2deg(angle_between(y_u2, (pc2 - pu2)))

        ok_ang = (
            self._in_ang_range(ang_c1_deg) and
            self._in_ang_range(ang_u1_deg) and
            self._in_ang_range(ang_c2_deg) and
            self._in_ang_range(ang_u2_deg)
        )

        # now = time.time()
        # if now - self.last_debug_t > 1.0:
        #     self.get_logger().info(
        #     f"  L1 = {L1:.4f} m (target={self.target_len_1_m:.4f} m, err={err1*1000:.1f} mm)\n"
        #     f"  L2 = {L2:.4f} m (target={self.target_len_2_m:.4f} m, err={err2*1000:.1f} mm)\n"
        #     f"  pc1 = {pc1.tolist()}\n"
        #     f"  pu1 = {pu1.tolist()}\n"
        #     f"  pu1-pc1 = {(pu1-pc1).tolist()}\n"
        #     f"  pc2 = {pc2.tolist()}\n"
        #     f"  pu2 = {pu2.tolist()}\n"
        #     f"  pu2-pc2 = {(pu2-pc2).tolist()}\n"
        #     f"  angles (deg): C1={ang_c1_deg:.1f}, U1={ang_u1_deg:.1f}, C2={ang_c2_deg:.1f}, U2={ang_u2_deg:.1f}"
        #     )

        if not ok_len:
            raise RuntimeError(
                f"TF HW safety check failed: SHAFT LENGTH OUT OF RANGE\n"
                f"  L1 = {L1:.4f} m (target={self.target_len_1_m:.4f} m, err={err1*1000:.1f} mm)\n"
                f"  L2 = {L2:.4f} m (target={self.target_len_2_m:.4f} m, err={err2*1000:.1f} mm)\n"
                f"  pc1 = {pc1.tolist()}\n"
                f"  pu1 = {pu1.tolist()}\n"
                f"  pu1-pc1 = {(pu1-pc1).tolist()}\n"
                f"  pc2 = {pc2.tolist()}\n"
                f"  pu2 = {pu2.tolist()}\n"
                f"  pu2-pc2 = {(pu2-pc2).tolist()}\n"
                f"  angles (deg): C1={ang_c1_deg:.1f}, U1={ang_u1_deg:.1f}, C2={ang_c2_deg:.1f}, U2={ang_u2_deg:.1f}"
            )
        elif not ok_ang:
            raise RuntimeError(
                f"TF HW safety check failed: BALL JOINT OUT OF SAFTY RANGE "
                f"(C1={ang_c1_deg:.1f}deg, U1={ang_u1_deg:.1f}deg, "
                f"C2={ang_c2_deg:.1f}deg, U2={ang_u2_deg:.1f}deg, "
                f"allowed=[{self.ang_min_deg:.1f}, {self.ang_max_deg:.1f}]deg)"
            )
        else:
            return True
    # ---------- main callback ----------

    def _accept_target_order(self, seq: int, stamp) -> bool:
        # 1) seq 우선
        if seq != 0:
            if self._last_seq is None:
                self._last_seq = int(seq)
                return True
            if int(seq) <= int(self._last_seq):
                return False
            self._last_seq = int(seq)
            return True

        # 2) seq가 0이면 stamp로
        if self._last_stamp is None:
            self._last_stamp = stamp
            return True
        if stamp_to_ns(stamp) <= stamp_to_ns(self._last_stamp):
            return False
        self._last_stamp = stamp
        return True

    def _on_both_foot_request(self, msg: RsuTarget):
        # monotonic check
        if not self._accept_target_order(msg.seq, msg.header.stamp):
            self.get_logger().warn("Received /rsu/target with non-increasing seq/stamp. Ignoring.")
            return

        # --- read inputs ---
        l_roll  = float(msg.left_roll)
        l_pitch = float(msg.left_pitch)
        r_roll  = float(msg.right_roll) * -1.0
        r_pitch = float(msg.right_pitch) * -1.0

        # --- solve left/right with continuity ---
        l_prev = self.prev_alpha_2D[0, :].copy()
        r_prev = self.prev_alpha_2D[1, :].copy()

        l_res = self.solver.solve(l_roll, l_pitch, l_prev)
        r_res = self.solver.solve(r_roll, r_pitch, r_prev)

        left_ok  = bool(l_res.feasible)
        right_ok = bool(r_res.feasible)

        # --- update or hold ---
        if left_ok:
            self.prev_alpha_2D[0, :] = np.array(l_res.alpha, dtype=np.float64).reshape(2,)
        else:
            if not self.hold_alpha_on_infeasible:
                self.prev_alpha_2D[0, :] = 0.0

        if right_ok:
            self.prev_alpha_2D[1, :] = np.array(r_res.alpha, dtype=np.float64).reshape(2,)
        else:
            if not self.hold_alpha_on_infeasible:
                self.prev_alpha_2D[1, :] = 0.0

        # --- publish solution ---
        out = RsuSolution()
        out.header.stamp = msg.header.stamp
        out.seq = msg.seq

        out.left_actuator_1  = float(self.prev_alpha_2D[0, 0])
        out.left_actuator_2  = float(self.prev_alpha_2D[0, 1])
        out.right_actuator_1 = float(self.prev_alpha_2D[1, 0]) * -1.0
        out.right_actuator_2 = float(self.prev_alpha_2D[1, 1]) * -1.0

        # feasible 의미: "이번 요청에 대해 좌/우 모두 유효해"
        out.feasible = bool(left_ok and right_ok)

        self.pub_both_foot_solution.publish(out)

    def _on_rpy(self, msg: Vector3Stamped):
        # NOTE
        # 상위 컨트롤러에서 cmd msg 발생시 마다 timestamp 작성 -> solver에서는 request msg의 타임스탬프와 출력값을 연동하여 관리
        if self._control_mode == CONTROL_MODE.DEBUG:
            if self._last_stamp is None:
                self._last_stamp = msg.header.stamp
            elif stamp_to_ns(msg.header.stamp) <= stamp_to_ns(self._last_stamp):
                self.get_logger().warn(
                    "Received /request_to_solver with timestamp older than or equal to last processed command. Ignoring."
                )
                return
            self._last_stamp = msg.header.stamp

        # RSU solve -> alpha
        res = self.solver.solve(float(msg.vector.x), float(msg.vector.y), self.prev_alpha_1D)

        if bool(res.feasible):
            a_solver = np.array(res.alpha, dtype=np.float64).reshape(2,)
            self.alpha1 = float(a_solver[0])
            self.alpha2 = float(a_solver[1])
            self.prev_alpha_1D[:] = a_solver
            self.roll = float(msg.vector.x)
            self.pitch = float(msg.vector.y)
        else:
            if self.hold_alpha_on_infeasible:
                self.get_logger().warn("[IK] infeasible -> hold previous state")
                # self.get_logger().debug(
                #     f"Previous alpha held: alpha1={self.alpha1:.3f}, alpha2={self.alpha2:.3f}"
                # )
            else:
                # optional: reset or other behavior
                self.get_logger().warn("[IK] infeasible -> no update (policy decided)")
        self.publish_joint_states()

        # TF gating: if crank state violates constraints, hold state and block publish
        if self.gate_publish_by_tf_check:
            try:
                self._tf_check_ok = self.tf_hardware_safetycheck()
                if not self._tf_check_ok:
                    self._tf_check_diag = "TF hardware safety check failed (length/angle out of range)"
            except Exception as e:
                self._tf_check_ok = False
                self.get_logger().error(f"TF hardware safety check FAILED: {e}")
        if self._control_mode == CONTROL_MODE.DEBUG:
            self.publish_solver_respond(res.feasible and self._tf_check_ok)


def main():
    rclpy.init()
    node = RSUSolverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()