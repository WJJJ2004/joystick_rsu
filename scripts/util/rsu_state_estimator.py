# rsu_state_estimator.py
# RSU ankle state estimator
# Uses existing closed-form IK solver as internal measurement model:
#   x = [roll, pitch]  ->  alpha = [alpha1, alpha2]
#
# Estimation target:
#   motor_pos, motor_vel -> q_rel=[roll, pitch], qd_rel=[roll_rate, pitch_rate]
#
# Design:
# - Position: damped Gauss-Newton on residual r = IK(x) - alpha_meas
# - Jacobian: numerical differentiation of IK with same-branch continuity seed
# - Velocity: damped pseudo-inverse of J
# - Stabilization: blend Jacobian velocity with finite-difference velocity + LPF
#
# Depends on rsu_solver.py containing:
#   RSUParams, RSUSolver, wrap_to_pi

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Tuple
import math
import numpy as np

from rsu_solver import RSUSolver, wrap_to_pi


def wrap_vec(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    out = np.zeros_like(v, dtype=float)
    it = np.nditer(v, flags=["multi_index"])
    for x in it:
        out[it.multi_index] = wrap_to_pi(float(x))
    return out


def clamp_norm(v: np.ndarray, max_norm: float) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n <= max_norm or n <= 1e-15:
        return v
    return v * (max_norm / n)


def sanitize_vec2(v: np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(2,)
    out = v.copy()
    for i in range(2):
        if not np.isfinite(out[i]):
            out[i] = 0.0
    return out


@dataclass
class RSUStateEstimatorConfig:
    # Position solve
    max_iter: int = 4
    lambda_pos: float = 1e-6
    tol_dx: float = 1e-8
    tol_res: float = 1e-8
    dx_max: float = math.radians(2.0)     # max correction norm per iteration
    q_init: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))

    # Jacobian
    jac_h: float = 1e-5
    jac_lambda: float = 1e-5              # damping for pseudo-inverse

    # Velocity
    beta_jac: float = 0.9                 # blend ratio for Jacobian velocity
    vel_lpf_tau: float = 0.02             # seconds
    motor_vel_lpf_tau: float = 0.01       # seconds

    # Validity / conditioning
    residual_thresh: float = 1e-3
    cond_warn: float = 50.0
    cond_fail: float = 200.0
    sigma_min_thresh: float = 1e-5

    # Timing / safety
    dt_min: float = 1e-5
    dt_max: float = 0.1

    # Fallback behavior
    hold_last_on_invalid: bool = True
    zero_vel_on_invalid: bool = True

    # Optional output saturation
    qd_limit: Optional[np.ndarray] = None  # shape (2,), rad/s

    def __post_init__(self):
        self.q_init = np.asarray(self.q_init, dtype=float).reshape(2,)
        if self.qd_limit is not None:
            self.qd_limit = np.asarray(self.qd_limit, dtype=float).reshape(2,)


@dataclass
class AnkleState:
    q_rel: np.ndarray           # [roll, pitch]
    qd_rel: np.ndarray          # [roll_rate, pitch_rate]
    q_motor: np.ndarray         # measured [alpha1, alpha2]
    qd_motor: np.ndarray        # measured [alpha_dot1, alpha_dot2]

    alpha_pred: np.ndarray      # IK(x_est) predicted motor angle
    residual: np.ndarray        # alpha_pred - alpha_meas (wrapped)
    residual_norm: float

    J_qx: np.ndarray            # d alpha / d x, shape (2,2)
    condJ: float
    sigma_min: float

    valid: bool
    degraded: bool
    feasible: bool
    iter_used: int
    branch: np.ndarray          # chosen branch of final IK solve

    debug_msg: str = ""


class RSUStateEstimator:
    def __init__(self, solver: RSUSolver, cfg: Optional[RSUStateEstimatorConfig] = None):
        self.solver = solver
        self.cfg = cfg if cfg is not None else RSUStateEstimatorConfig()

        self.initialized_ = False

        self.x_prev_ = self.cfg.q_init.copy()               # previous q_rel
        self.xd_prev_ = np.zeros(2, dtype=float)            # previous qd_rel
        self.alpha_prev_seed_ = np.zeros(2, dtype=float)   # for IK continuity
        self.motor_vel_prev_filt_ = np.zeros(2, dtype=float)

        self.last_valid_ = False
        self.last_branch_ = np.full(2, -1, dtype=int)

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------
    # def reset(self, q_init: Optional[np.ndarray] = None):
    #     self.initialized_ = False
    #     self.x_prev_ = self.cfg.q_init.copy() if q_init is None else np.asarray(q_init, dtype=float).reshape(2,)
    #     self.xd_prev_ = np.zeros(2, dtype=float)
    #     self.alpha_prev_seed_ = np.zeros(2, dtype=float)
    #     self.motor_vel_prev_filt_ = np.zeros(2, dtype=float)
    #     self.last_valid_ = False
    #     self.last_branch_ = np.full(2, -1, dtype=int)

    def reset(
        self,
        q_init: Optional[np.ndarray] = None,
        alpha_seed: Optional[np.ndarray] = None,
        qd_init: Optional[np.ndarray] = None,
        initialized: bool = False,
    ):
        self.initialized_ = initialized

        self.x_prev_ = (
            self.cfg.q_init.copy()
            if q_init is None
            else np.asarray(q_init, dtype=float).reshape(2,)
        )

        self.xd_prev_ = (
            np.zeros(2, dtype=float)
            if qd_init is None
            else np.asarray(qd_init, dtype=float).reshape(2,)
        )

        self.alpha_prev_seed_ = (
            np.zeros(2, dtype=float)
            if alpha_seed is None
            else np.asarray(alpha_seed, dtype=float).reshape(2,)
        )

        self.motor_vel_prev_filt_ = np.zeros(2, dtype=float)
        self.last_valid_ = False
        self.last_branch_ = np.full(2, -1, dtype=int)

    def update(
        self,
        motor_pos: np.ndarray,
        motor_vel: np.ndarray,
        dt: float,
    ) -> AnkleState:
        motor_pos = sanitize_vec2(motor_pos)
        motor_vel = sanitize_vec2(motor_vel)
        dt_ok = np.isfinite(dt) and (self.cfg.dt_min <= dt <= self.cfg.dt_max)

        if not dt_ok:
            return self._fallback_state(
                motor_pos=motor_pos,
                motor_vel=motor_vel,
                debug_msg=f"invalid dt: {dt}",
            )

        motor_vel_filt = self._lowpass_vec(
            raw=motor_vel,
            prev=self.motor_vel_prev_filt_,
            tau=self.cfg.motor_vel_lpf_tau,
            dt=dt,
        )
        self.motor_vel_prev_filt_ = motor_vel_filt

        x_init = self.x_prev_.copy() if self.initialized_ else self.cfg.q_init.copy()

        ok_pos, x_est, alpha_pred, residual, residual_norm, iter_used, branch, debug_pos = \
            self.estimate_position(alpha_meas=motor_pos, x_init=x_init)

        if not ok_pos:
            return self._fallback_state(
                motor_pos=motor_pos,
                motor_vel=motor_vel_filt,
                debug_msg=f"position estimation failed: {debug_pos}",
            )

        try:
            J = self.compute_jacobian_numerical(x=x_est, alpha_seed=alpha_pred)
        except Exception as e:
            return self._fallback_state(
                motor_pos=motor_pos,
                motor_vel=motor_vel_filt,
                debug_msg=f"jacobian failed: {e}",
            )

        if not np.all(np.isfinite(J)):
            return self._fallback_state(
                motor_pos=motor_pos,
                motor_vel=motor_vel_filt,
                debug_msg="jacobian has NaN/Inf",
            )

        xdot_jac, condJ, sigma_min = self.motorVelToAnkleVel(J_qx=J, alpha_dot=motor_vel_filt)

        if not np.all(np.isfinite(xdot_jac)):
            return self._fallback_state(
                motor_pos=motor_pos,
                motor_vel=motor_vel_filt,
                debug_msg="velocity solve produced NaN/Inf",
            )

        if self.initialized_:
            xdot_fd = wrap_vec(x_est - self.x_prev_) / dt
        else:
            xdot_fd = np.zeros(2, dtype=float)

        beta = float(self.cfg.beta_jac)
        xdot_raw = beta * xdot_jac + (1.0 - beta) * xdot_fd

        xdot_est = self._lowpass_vec(
            raw=xdot_raw,
            prev=self.xd_prev_,
            tau=self.cfg.vel_lpf_tau,
            dt=dt,
        )

        if self.cfg.qd_limit is not None:
            xdot_est = np.clip(xdot_est, -self.cfg.qd_limit, self.cfg.qd_limit)

        valid = True
        degraded = False
        debug_flags = []

        if residual_norm > self.cfg.residual_thresh:
            valid = False
            debug_flags.append(f"residual_norm={residual_norm:.3e} > thresh={self.cfg.residual_thresh:.3e}")

        if condJ > self.cfg.cond_fail:
            valid = False
            debug_flags.append(f"condJ={condJ:.3f} > fail={self.cfg.cond_fail:.3f}")
        elif condJ > self.cfg.cond_warn:
            degraded = True
            debug_flags.append(f"condJ={condJ:.3f} > warn={self.cfg.cond_warn:.3f}")

        if sigma_min < self.cfg.sigma_min_thresh:
            degraded = True
            debug_flags.append(f"sigma_min={sigma_min:.3e} < thresh={self.cfg.sigma_min_thresh:.3e}")

        if not np.all(np.isfinite(x_est)) or not np.all(np.isfinite(xdot_est)):
            valid = False
            debug_flags.append("state contains NaN/Inf")

        if not valid:
            return self._fallback_state(
                motor_pos=motor_pos,
                motor_vel=motor_vel_filt,
                debug_msg="; ".join(debug_flags) if debug_flags else "invalid state",
                J_qx=J,
                alpha_pred=alpha_pred,
                residual=residual,
                residual_norm=residual_norm,
                condJ=condJ,
                sigma_min=sigma_min,
                branch=branch,
                iter_used=iter_used,
            )

        self.x_prev_ = x_est.copy()
        self.xd_prev_ = xdot_est.copy()
        self.alpha_prev_seed_ = alpha_pred.copy()
        self.initialized_ = True
        self.last_valid_ = True
        self.last_branch_ = np.asarray(branch, dtype=int).copy()

        return AnkleState(
            q_rel=x_est.copy(),
            qd_rel=xdot_est.copy(),
            q_motor=motor_pos.copy(),
            qd_motor=motor_vel_filt.copy(),
            alpha_pred=alpha_pred.copy(),
            residual=residual.copy(),
            residual_norm=float(residual_norm),
            J_qx=J.copy(),
            condJ=float(condJ),
            sigma_min=float(sigma_min),
            valid=True,
            degraded=degraded,
            feasible=True,
            iter_used=int(iter_used),
            branch=np.asarray(branch, dtype=int).copy(),
            debug_msg="; ".join(debug_flags),
        )

    # -------------------------------------------------------------------------
    # Core position estimation
    # -------------------------------------------------------------------------
    def estimate_position(
        self,
        alpha_meas: np.ndarray,
        x_init: np.ndarray,
    ) -> Tuple[bool, np.ndarray, np.ndarray, np.ndarray, float, int, np.ndarray, str]:
        alpha_meas = np.asarray(alpha_meas, dtype=float).reshape(2,)
        x = np.asarray(x_init, dtype=float).reshape(2,).copy()

        final_alpha_pred = np.zeros(2, dtype=float)
        final_residual = np.zeros(2, dtype=float)
        final_branch = np.full(2, -1, dtype=int)

        for it in range(self.cfg.max_iter):
            sol = self.solver.solve(float(x[0]), float(x[1]), prev_alpha=self.alpha_prev_seed_)

            if not sol.feasible or np.any(sol.branch < 0):
                return False, x, final_alpha_pred, final_residual, float("inf"), it, final_branch, "IK infeasible during GN"

            alpha_pred = np.asarray(sol.alpha, dtype=float).reshape(2,)
            residual = wrap_vec(alpha_pred - alpha_meas)
            residual_norm = float(np.linalg.norm(residual))

            try:
                J = self.compute_jacobian_numerical(x=x, alpha_seed=alpha_pred)
            except Exception as e:
                return False, x, alpha_pred, residual, residual_norm, it, np.asarray(sol.branch, dtype=int), f"jacobian in GN failed: {e}"

            H = J.T @ J + self.cfg.lambda_pos * np.eye(2, dtype=float)
            g = J.T @ residual

            try:
                dx = -np.linalg.solve(H, g)
            except np.linalg.LinAlgError:
                return False, x, alpha_pred, residual, residual_norm, it, np.asarray(sol.branch, dtype=int), "normal equation solve failed"

            dx = clamp_norm(dx, self.cfg.dx_max)
            x_new = x + dx

            final_alpha_pred = alpha_pred
            final_residual = residual
            final_branch = np.asarray(sol.branch, dtype=int)

            if float(np.linalg.norm(dx)) < self.cfg.tol_dx and residual_norm < self.cfg.tol_res:
                x = x_new
                break

            x = x_new

        sol_final = self.solver.solve(float(x[0]), float(x[1]), prev_alpha=self.alpha_prev_seed_)
        if not sol_final.feasible or np.any(sol_final.branch < 0):
            return False, x, final_alpha_pred, final_residual, float("inf"), self.cfg.max_iter, final_branch, "final IK infeasible"

        alpha_pred = np.asarray(sol_final.alpha, dtype=float).reshape(2,)
        residual = wrap_vec(alpha_pred - alpha_meas)
        residual_norm = float(np.linalg.norm(residual))
        branch = np.asarray(sol_final.branch, dtype=int)

        return True, x, alpha_pred, residual, residual_norm, self.cfg.max_iter, branch, "ok"

    # -------------------------------------------------------------------------
    # Numerical Jacobian with same-branch continuity
    # -------------------------------------------------------------------------
    def compute_jacobian_numerical(
        self,
        x: np.ndarray,
        alpha_seed: np.ndarray,
    ) -> np.ndarray:
        x = np.asarray(x, dtype=float).reshape(2,)
        alpha_seed = np.asarray(alpha_seed, dtype=float).reshape(2,)
        h = float(self.cfg.jac_h)

        phi = float(x[0])
        pitch = float(x[1])

        # Center solve with local branch seed
        sol_c = self.solver.solve(phi, pitch, prev_alpha=alpha_seed)
        if not sol_c.feasible or np.any(sol_c.branch < 0):
            raise RuntimeError("center IK infeasible")

        alpha0 = np.asarray(sol_c.alpha, dtype=float).reshape(2,)

        sol_pp = self.solver.solve(phi + h, pitch, prev_alpha=alpha0)
        sol_pm = self.solver.solve(phi - h, pitch, prev_alpha=alpha0)
        sol_tp = self.solver.solve(phi, pitch + h, prev_alpha=alpha0)
        sol_tm = self.solver.solve(phi, pitch - h, prev_alpha=alpha0)

        if not (sol_pp.feasible and sol_pm.feasible and sol_tp.feasible and sol_tm.feasible):
            raise RuntimeError("finite difference neighborhood infeasible")

        a_pp = np.asarray(sol_pp.alpha, dtype=float).reshape(2,)
        a_pm = np.asarray(sol_pm.alpha, dtype=float).reshape(2,)
        a_tp = np.asarray(sol_tp.alpha, dtype=float).reshape(2,)
        a_tm = np.asarray(sol_tm.alpha, dtype=float).reshape(2,)

        d_dphi = wrap_vec(a_pp - a_pm) / (2.0 * h)
        d_dth = wrap_vec(a_tp - a_tm) / (2.0 * h)

        J = np.column_stack([d_dphi, d_dth])
        return J

    # -------------------------------------------------------------------------
    # Velocity mapping by damped pseudo-inverse
    # -------------------------------------------------------------------------
    def motorVelToAnkleVel(
        self,
        J_qx: np.ndarray,
        alpha_dot: np.ndarray,
    ) -> Tuple[np.ndarray, float, float]:
        J_qx = np.asarray(J_qx, dtype=float).reshape(2, 2)
        alpha_dot = np.asarray(alpha_dot, dtype=float).reshape(2,)

        U, S, Vt = np.linalg.svd(J_qx, full_matrices=False)

        damped_inv = np.zeros_like(S)
        lam = float(self.cfg.jac_lambda)
        for i, s in enumerate(S):
            damped_inv[i] = s / (s * s + lam)

        J_pinv = Vt.T @ np.diag(damped_inv) @ U.T
        xdot = J_pinv @ alpha_dot

        sigma_min = float(np.min(S)) if S.size > 0 else 0.0
        sigma_max = float(np.max(S)) if S.size > 0 else 0.0
        condJ = float("inf") if sigma_min <= 1e-15 else sigma_max / sigma_min

        return xdot, condJ, sigma_min

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------
    def _lowpass_vec(self, raw: np.ndarray, prev: np.ndarray, tau: float, dt: float) -> np.ndarray:
        raw = np.asarray(raw, dtype=float).reshape(2,)
        prev = np.asarray(prev, dtype=float).reshape(2,)
        if tau <= 0.0:
            return raw.copy()
        gamma = dt / (tau + dt)
        gamma = max(0.0, min(1.0, gamma))
        return (1.0 - gamma) * prev + gamma * raw

    def _fallback_state(
        self,
        motor_pos: np.ndarray,
        motor_vel: np.ndarray,
        debug_msg: str,
        J_qx: Optional[np.ndarray] = None,
        alpha_pred: Optional[np.ndarray] = None,
        residual: Optional[np.ndarray] = None,
        residual_norm: Optional[float] = None,
        condJ: Optional[float] = None,
        sigma_min: Optional[float] = None,
        branch: Optional[np.ndarray] = None,
        iter_used: int = 0,
    ) -> AnkleState:
        if self.cfg.hold_last_on_invalid and self.last_valid_:
            q_rel = self.x_prev_.copy()
        else:
            q_rel = self.cfg.q_init.copy()

        if self.cfg.zero_vel_on_invalid:
            qd_rel = np.zeros(2, dtype=float)
        else:
            qd_rel = self.xd_prev_.copy()

        self.initialized_ = self.initialized_ or self.last_valid_

        return AnkleState(
            q_rel=q_rel,
            qd_rel=qd_rel,
            q_motor=np.asarray(motor_pos, dtype=float).reshape(2,).copy(),
            qd_motor=np.asarray(motor_vel, dtype=float).reshape(2,).copy(),
            alpha_pred=np.zeros(2, dtype=float) if alpha_pred is None else np.asarray(alpha_pred, dtype=float).reshape(2,).copy(),
            residual=np.zeros(2, dtype=float) if residual is None else np.asarray(residual, dtype=float).reshape(2,).copy(),
            residual_norm=float("inf") if residual_norm is None else float(residual_norm),
            J_qx=np.zeros((2, 2), dtype=float) if J_qx is None else np.asarray(J_qx, dtype=float).reshape(2, 2).copy(),
            condJ=float("inf") if condJ is None else float(condJ),
            sigma_min=0.0 if sigma_min is None else float(sigma_min),
            valid=False,
            degraded=True,
            feasible=False,
            iter_used=int(iter_used),
            branch=np.full(2, -1, dtype=int) if branch is None else np.asarray(branch, dtype=int).reshape(2,).copy(),
            debug_msg=debug_msg,
        )


# -----------------------------------------------------------------------------
# Example usage
# -----------------------------------------------------------------------------
if __name__ == "__main__":
    from rsu_solver import RSUParams

    W_a = np.array([[-86.0, 40.0, 235.0],
                    [-86.0, -40.0, 235.0]])
    F_b = np.array([[-34.0, 36.0, 36.0],
                    [-34.0, -36.0, 36.0]])
    c = np.array([40.0, 40.0])
    r = np.array([80.0, 80.0])
    psi = np.deg2rad(np.array([-90.0, 90.0]))

    params = RSUParams(a_W=W_a, b_F=F_b, c=c, r=r, psi=psi, eps=1e-9)
    solver = RSUSolver(params)

    cfg = RSUStateEstimatorConfig(
        max_iter=4,
        lambda_pos=1e-6,
        jac_h=1e-5,
        jac_lambda=1e-5,
        residual_thresh=1e-3,
        beta_jac=0.9,
        vel_lpf_tau=0.02,
        q_init=np.array([0.0, 0.0]),
    )

    estimator = RSUStateEstimator(solver=solver, cfg=cfg)

    # truth
    roll_true = math.radians(5.0)
    pitch_true = math.radians(-10.0)
    sol = solver.solve(roll_true, pitch_true, prev_alpha=None)

    motor_pos = sol.alpha
    motor_vel = np.array([0.1, -0.05], dtype=float)

    state = estimator.update(motor_pos=motor_pos, motor_vel=motor_vel, dt=0.002)

    print("valid       :", state.valid)
    print("degraded    :", state.degraded)
    print("q_rel       :", state.q_rel)
    print("qd_rel      :", state.qd_rel)
    print("residual    :", state.residual)
    print("res_norm    :", state.residual_norm)
    print("condJ       :", state.condJ)
    print("sigma_min   :", state.sigma_min)
    print("branch      :", state.branch)
    print("debug_msg   :", state.debug_msg)