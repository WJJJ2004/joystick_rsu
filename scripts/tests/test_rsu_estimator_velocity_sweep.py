"""
scripts.tests.test_rsu_estimator_velocity_sweep의 Docstring

jac_lambda = 1e-6
jac_h = 5e-5
beta_jac = 1.0
vel_lpf_tau = 0.0
motor_vel_lpf_tau = 0.0
"""



# tests/test_rsu_estimator_velocity_sweep.py
import os
import sys
import math
import itertools
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "util"))

from rsu_solver import RSUParams, RSUSolver
from rsu_state_estimator import RSUStateEstimator, RSUStateEstimatorConfig


def rad2deg(x):
    return np.asarray(x) * 180.0 / math.pi


def make_solver():
    W_a = np.array([[0.0,  36.0, 170.0],
                    [0.0, -36.0,  82.0]])
    F_b = np.array([[-20.0,  36.0, 16.0],
                    [-20.0, -36.0, 16.0]])
    c = np.array([30.0, -30.0])
    r = np.array([154.0, 66.0])
    psi = np.array([1.5707963267948966, -1.5707963267948966])
    # =========================

    params = RSUParams(a_W=W_a, b_F=F_b, c=c, r=r, psi=psi, eps=1e-9)
    return RSUSolver(params)


def make_estimator(solver, jac_lambda, jac_h, beta_jac, vel_lpf_tau, motor_vel_lpf_tau):
    cfg = RSUStateEstimatorConfig(
        max_iter=5,
        lambda_pos=1e-6,
        jac_h=jac_h,
        jac_lambda=jac_lambda,
        residual_thresh=1e-4,
        beta_jac=beta_jac,
        vel_lpf_tau=vel_lpf_tau,
        motor_vel_lpf_tau=motor_vel_lpf_tau,
        cond_warn=50.0,
        cond_fail=200.0,
        sigma_min_thresh=1e-5,
        q_init=np.array([0.0, 0.0]),
    )
    return RSUStateEstimator(solver=solver, cfg=cfg)


def build_trajectory():
    dt = 0.002
    T = 8.0
    t = np.arange(0.0, T, dt)

    roll = np.deg2rad(6.0) * np.sin(2.0 * math.pi * 0.5 * t)
    pitch = np.deg2rad(8.0) * np.sin(2.0 * math.pi * 0.35 * t + 0.4)

    roll_dot = np.deg2rad(6.0) * (2.0 * math.pi * 0.5) * np.cos(2.0 * math.pi * 0.5 * t)
    pitch_dot = np.deg2rad(8.0) * (2.0 * math.pi * 0.35) * np.cos(2.0 * math.pi * 0.35 * t + 0.4)

    q_true = np.column_stack([roll, pitch])
    qd_true = np.column_stack([roll_dot, pitch_dot])
    return t, dt, q_true, qd_true


def generate_motor_data(solver, t, q_true, dt):
    alpha = np.zeros((len(t), 2), dtype=float)
    feasible_mask = np.zeros(len(t), dtype=bool)

    prev_alpha = None
    for k in range(len(t)):
        sol = solver.solve(float(q_true[k, 0]), float(q_true[k, 1]), prev_alpha=prev_alpha)
        if sol.feasible:
            alpha[k, :] = sol.alpha
            feasible_mask[k] = True
            prev_alpha = sol.alpha.copy()
        else:
            alpha[k, :] = np.nan
            feasible_mask[k] = False
            prev_alpha = None

    alpha_dot = np.zeros_like(alpha)
    for k in range(1, len(t) - 1):
        if feasible_mask[k - 1] and feasible_mask[k + 1]:
            alpha_dot[k, :] = (alpha[k + 1, :] - alpha[k - 1, :]) / (2.0 * dt)
        else:
            alpha_dot[k, :] = 0.0

    alpha_dot[0, :] = alpha_dot[1, :]
    alpha_dot[-1, :] = alpha_dot[-2, :]

    return alpha, alpha_dot, feasible_mask


def run_one_case(
    solver,
    q_true,
    qd_true,
    alpha,
    alpha_dot,
    feasible_mask,
    dt,
    jac_lambda,
    jac_h,
    beta_jac,
    vel_lpf_tau,
    motor_vel_lpf_tau,
    warmup_sec=0.3,
):
    estimator = make_estimator(
        solver=solver,
        jac_lambda=jac_lambda,
        jac_h=jac_h,
        beta_jac=beta_jac,
        vel_lpf_tau=vel_lpf_tau,
        motor_vel_lpf_tau=motor_vel_lpf_tau,
    )

    N = len(q_true)

    q_hat = np.full_like(q_true, np.nan, dtype=float)
    qd_hat = np.full_like(qd_true, np.nan, dtype=float)
    residual_norm = np.full(N, np.nan, dtype=float)
    condJ = np.full(N, np.nan, dtype=float)
    sigma_min = np.full(N, np.nan, dtype=float)
    valid_mask = np.zeros(N, dtype=bool)
    degraded_mask = np.zeros(N, dtype=bool)

    for k in range(N):
        if not feasible_mask[k]:
            continue

        state = estimator.update(alpha[k, :], alpha_dot[k, :], dt)

        q_hat[k, :] = state.q_rel
        qd_hat[k, :] = state.qd_rel
        residual_norm[k] = state.residual_norm
        condJ[k] = state.condJ
        sigma_min[k] = state.sigma_min
        valid_mask[k] = state.valid
        degraded_mask[k] = state.degraded

    warmup_n = int(round(warmup_sec / dt))
    eval_mask = feasible_mask & valid_mask
    if warmup_n < N:
        eval_mask[:warmup_n] = False
    if np.sum(eval_mask) == 0:
        return {
            "success": False,
            "reason": "no valid evaluation samples",
            "jac_lambda": jac_lambda,
            "jac_h": jac_h,
            "beta_jac": beta_jac,
            "vel_lpf_tau": vel_lpf_tau,
            "motor_vel_lpf_tau": motor_vel_lpf_tau,
        }

    q_err = q_hat[eval_mask] - q_true[eval_mask]
    qd_err = qd_hat[eval_mask] - qd_true[eval_mask]

    q_err_norm = np.linalg.norm(q_err, axis=1)
    qd_err_norm = np.linalg.norm(qd_err, axis=1)

    qd_rms_each = np.sqrt(np.mean(qd_err ** 2, axis=0))
    qd_rms_norm = math.sqrt(np.mean(np.sum(qd_err ** 2, axis=1)))

    result = {
        "success": True,
        "jac_lambda": jac_lambda,
        "jac_h": jac_h,
        "beta_jac": beta_jac,
        "vel_lpf_tau": vel_lpf_tau,
        "motor_vel_lpf_tau": motor_vel_lpf_tau,

        "samples_total": N,
        "samples_feasible": int(np.sum(feasible_mask)),
        "samples_valid": int(np.sum(valid_mask)),
        "samples_eval": int(np.sum(eval_mask)),
        "degraded_count": int(np.sum(degraded_mask)),

        "q_err_mean_deg": float(np.mean(rad2deg(q_err_norm))),
        "q_err_max_deg": float(np.max(rad2deg(q_err_norm))),

        "qd_err_mean_deg_s": float(np.mean(rad2deg(qd_err_norm))),
        "qd_err_max_deg_s": float(np.max(rad2deg(qd_err_norm))),
        "qd_rms_roll_deg_s": float(rad2deg(qd_rms_each[0])),
        "qd_rms_pitch_deg_s": float(rad2deg(qd_rms_each[1])),
        "qd_rms_norm_deg_s": float(rad2deg(qd_rms_norm)),

        "residual_mean": float(np.nanmean(residual_norm[eval_mask])),
        "residual_max": float(np.nanmax(residual_norm[eval_mask])),

        "condJ_mean": float(np.nanmean(condJ[eval_mask])),
        "condJ_max": float(np.nanmax(condJ[eval_mask])),

        "sigma_min_mean": float(np.nanmean(sigma_min[eval_mask])),
        "sigma_min_min": float(np.nanmin(sigma_min[eval_mask])),
    }
    return result


def print_result(res, rank=None):
    head = f"[Rank {rank}] " if rank is not None else ""
    if not res["success"]:
        print(head + f"FAILED | "
              f"jac_lambda={res['jac_lambda']:.1e}, "
              f"jac_h={res['jac_h']:.1e}, "
              f"beta={res['beta_jac']:.2f}, "
              f"vel_tau={res['vel_lpf_tau']:.3f}, "
              f"motor_tau={res['motor_vel_lpf_tau']:.3f} | "
              f"reason={res['reason']}")
        return

    print(
        head +
        f"jac_lambda={res['jac_lambda']:.1e}, "
        f"jac_h={res['jac_h']:.1e}, "
        f"beta={res['beta_jac']:.2f}, "
        f"vel_tau={res['vel_lpf_tau']:.3f}, "
        f"motor_tau={res['motor_vel_lpf_tau']:.3f} | "
        f"q_mean={res['q_err_mean_deg']:.4f} deg, "
        f"q_max={res['q_err_max_deg']:.4f} deg | "
        f"qd_mean={res['qd_err_mean_deg_s']:.4f} deg/s, "
        f"qd_max={res['qd_err_max_deg_s']:.4f} deg/s, "
        f"qd_rms={res['qd_rms_norm_deg_s']:.4f} deg/s | "
        f"res_mean={res['residual_mean']:.3e}, "
        f"cond_max={res['condJ_max']:.3f}"
    )


def main():
    solver = make_solver()
    t, dt, q_true, qd_true = build_trajectory()
    alpha, alpha_dot, feasible_mask = generate_motor_data(solver, t, q_true, dt)

    if np.sum(feasible_mask) == 0:
        print("No feasible motor trajectory samples.")
        return

    # ======================
    # Sweep candidates
    # ======================
    jac_lambda_list = [1e-7, 1e-6, 1e-5, 1e-4]
    jac_h_list = [1e-6, 1e-5, 5e-5]
    beta_jac_list = [1.0, 0.95, 0.9, 0.8]
    vel_lpf_tau_list = [0.0, 0.003, 0.005, 0.01]
    motor_vel_lpf_tau_list = [0.0, 0.002, 0.005]

    combos = list(itertools.product(
        jac_lambda_list,
        jac_h_list,
        beta_jac_list,
        vel_lpf_tau_list,
        motor_vel_lpf_tau_list,
    ))

    print("==== RSU VELOCITY SWEEP TEST ====")
    print(f"total combos: {len(combos)}")
    print(f"trajectory samples: {len(t)}")
    print(f"feasible samples: {np.sum(feasible_mask)}")
    print("")

    results = []

    for idx, (jac_lambda, jac_h, beta_jac, vel_tau, motor_tau) in enumerate(combos, start=1):
        res = run_one_case(
            solver=solver,
            q_true=q_true,
            qd_true=qd_true,
            alpha=alpha,
            alpha_dot=alpha_dot,
            feasible_mask=feasible_mask,
            dt=dt,
            jac_lambda=jac_lambda,
            jac_h=jac_h,
            beta_jac=beta_jac,
            vel_lpf_tau=vel_tau,
            motor_vel_lpf_tau=motor_tau,
            warmup_sec=0.3,
        )
        results.append(res)

        if idx % 20 == 0 or idx == len(combos):
            print(f"progress: {idx}/{len(combos)}")

    success_results = [r for r in results if r["success"]]
    if len(success_results) == 0:
        print("No successful parameter set.")
        return

    # 정렬 기준:
    # 1) qd mean error
    # 2) qd RMS
    # 3) qd max error
    # 4) q mean error
    ranked = sorted(
        success_results,
        key=lambda r: (
            r["qd_err_mean_deg_s"],
            r["qd_rms_norm_deg_s"],
            r["qd_err_max_deg_s"],
            r["q_err_mean_deg"],
        )
    )

    print("\n==== TOP 15 PARAMETER SETS ====\n")
    for i, r in enumerate(ranked[:15], start=1):
        print_result(r, rank=i)

    best = ranked[0]
    print("\n==== BEST SET DETAIL ====")
    for k, v in best.items():
        if isinstance(v, float):
            print(f"{k:24s}: {v:.8e}")
        else:
            print(f"{k:24s}: {v}")

    # 참고용: 검증모드 후보만 따로 보기
    verify_mode = [
        r for r in success_results
        if abs(r["beta_jac"] - 1.0) < 1e-12
        and abs(r["vel_lpf_tau"]) < 1e-12
        and abs(r["motor_vel_lpf_tau"]) < 1e-12
    ]
    if len(verify_mode) > 0:
        verify_best = sorted(
            verify_mode,
            key=lambda r: (
                r["qd_err_mean_deg_s"],
                r["qd_rms_norm_deg_s"],
                r["qd_err_max_deg_s"],
            )
        )[0]
        print("\n==== BEST VERIFY-MODE SET (beta=1, no LPF) ====")
        print_result(verify_best)

    # 참고용: 실사용 모드 후보만 따로 보기
    runtime_mode = [
        r for r in success_results
        if r["beta_jac"] <= 0.95
    ]
    if len(runtime_mode) > 0:
        runtime_best = sorted(
            runtime_mode,
            key=lambda r: (
                r["qd_err_mean_deg_s"],
                r["qd_rms_norm_deg_s"],
                r["qd_err_max_deg_s"],
            )
        )[0]
        print("\n==== BEST RUNTIME-MODE SET ====")
        print_result(runtime_best)


if __name__ == "__main__":
    main()