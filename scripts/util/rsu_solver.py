# rsu_solver.py
# Closed-form RSU ankle IK (paper-style, with mount angle psi about +Z)
# Foot orientation: R = Ry(pitch) * Rx(roll), yaw = 0
# After mount alignment: dy*cos(alpha) + dz*sin(alpha) = k

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional, Tuple, List
import math
import numpy as np


@dataclass
class RSUParams:
    # All positions in consistent length units (e.g., mm)
    # Angles in radians
    a_W: np.ndarray   # shape (2,3)
    b_F: np.ndarray   # shape (2,3)
    c: np.ndarray     # shape (2,)
    r: np.ndarray     # shape (2,)
    psi: np.ndarray   # shape (2,) mount yaw about +Z (rad)
    eps: float = 1e-9

    def __post_init__(self):
        self.a_W = np.asarray(self.a_W, dtype=float).reshape(2, 3)
        self.b_F = np.asarray(self.b_F, dtype=float).reshape(2, 3)
        self.c = np.asarray(self.c, dtype=float).reshape(2,)
        self.r = np.asarray(self.r, dtype=float).reshape(2,)
        self.psi = np.asarray(self.psi, dtype=float).reshape(2,)


@dataclass
class SolveResult:
    feasible: bool
    alpha: np.ndarray         # shape (2,)
    branch: np.ndarray        # shape (2,) int
    k: np.ndarray             # shape (2,)
    rho: np.ndarray           # shape (2,)
    asin_arg: np.ndarray      # shape (2,)
    residual: np.ndarray      # shape (2,)
    d: np.ndarray             # shape (2,3)
    d_hat: np.ndarray         # shape (2,3)
    d_tilde: np.ndarray       # shape (2,3)


def Rx(a: float) -> np.ndarray:
    c = math.cos(a); s = math.sin(a)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]], dtype=float)

def Ry(a: float) -> np.ndarray:
    c = math.cos(a); s = math.sin(a)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=float)

def Rz(a: float) -> np.ndarray:
    c = math.cos(a); s = math.sin(a)
    return np.array([[c,-s, 0],
                     [s, c, 0],
                     [0, 0, 1]], dtype=float)

def wrap_to_pi(x: float) -> float:
    # (-pi, pi]
    two_pi = 2.0 * math.pi
    x = math.fmod(x + math.pi, two_pi)
    if x < 0:
        x += two_pi
    return x - math.pi

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class RSUSolver:
    def __init__(self, params: RSUParams):
        self.p = params

    @staticmethod
    def _alpha_candidates_paper_yz(dy: float, dz: float, k: float, eps: float) -> Tuple[List[float], List[bool], float, float]:
        """
        Solve: dy*cos(a) + dz*sin(a) = k
        rho = sqrt(dy^2 + dz^2), varphi = atan2(dy, dz)
        a = -varphi + asin(k/rho)
        a = -varphi + pi - asin(k/rho)
        """
        rho = math.sqrt(dy*dy + dz*dz)
        if rho <= eps:
            return [0.0, 0.0], [False, False], rho, float("nan")

        arg = k / rho
        arg_c = clamp(arg, -1.0, 1.0)

        # strict feasibility check with tiny tolerance
        if abs(arg) > 1.0 + 1e-12:
            varphi = math.atan2(dy, dz)
            asv = math.asin(arg_c)
            a0 = wrap_to_pi(-varphi + asv)
            a1 = wrap_to_pi(-varphi + math.pi - asv)
            return [a0, a1], [False, False], rho, arg_c

        varphi = math.atan2(dy, dz)
        asv = math.asin(arg_c)
        a0 = wrap_to_pi(-varphi + asv)
        a1 = wrap_to_pi(-varphi + math.pi - asv)
        return [a0, a1], [True, True], rho, arg_c

    def solve(self, roll: float, pitch: float, prev_alpha: Optional[np.ndarray] = None) -> SolveResult:
        """
        roll=phi (rad), pitch=theta (rad), yaw=0
        prev_alpha: shape (2,) in radians; used for continuity-based branch selection
        """
        p = self.p

        # R_WF = Ry(pitch) Rx(roll)
        R_WF = Ry(pitch) @ Rx(roll)

        d = np.zeros((2,3), dtype=float)
        d_hat = np.zeros((2,3), dtype=float)
        d_tilde = np.zeros((2,3), dtype=float)

        k = np.zeros(2, dtype=float)
        rho = np.zeros(2, dtype=float)
        asin_arg = np.zeros(2, dtype=float)

        alpha_cands = [[0.0, 0.0], [0.0, 0.0]]
        valid_cands = [[False, False], [False, False]]

        feasible = True

        for i in range(2):
            if p.c[i] <= p.eps or p.r[i] <= p.eps:
                feasible = False
                continue

            # d_i = a_i - R b_i
            d[i,:] = p.a_W[i,:] - (R_WF @ p.b_F[i,:])

            dn = float(np.linalg.norm(d[i,:]))
            if dn <= p.eps:
                feasible = False
                continue

            d_hat[i,:] = d[i,:] / dn

            # k_i
            k[i] = (p.r[i]**2 - p.c[i]**2 - dn**2) / (2.0 * p.c[i] * dn)

            # mount alignment
            d_tilde[i,:] = (Rz(p.psi[i]).T @ d_hat[i,:])

            dy = float(d_tilde[i,1])
            dz = float(d_tilde[i,2])

            cands, valids, rho_i, arg_i = self._alpha_candidates_paper_yz(dy, dz, float(k[i]), p.eps)
            alpha_cands[i] = cands
            valid_cands[i] = valids
            rho[i] = rho_i
            asin_arg[i] = arg_i

            if not (valids[0] and valids[1]):
                feasible = False

        if not feasible:
            return SolveResult(
                feasible=False,
                alpha=np.array([0.0, 0.0], dtype=float),
                branch=np.array([0, 0], dtype=int),
                k=k, rho=rho, asin_arg=asin_arg,
                residual=np.array([float("nan"), float("nan")], dtype=float),
                d=d, d_hat=d_hat, d_tilde=d_tilde
            )

        # Branch selection among 4 combos
        best_cost = float("inf")
        best_alpha = np.array([0.0, 0.0], dtype=float)
        best_branch = np.array([0, 0], dtype=int)

        prev = None
        if prev_alpha is not None:
            prev = np.asarray(prev_alpha, dtype=float).reshape(2,)

        for b0 in [0, 1]:
            for b1 in [0, 1]:
                a_try = np.array([alpha_cands[0][b0], alpha_cands[1][b1]], dtype=float)

                if prev is not None:
                    d0 = wrap_to_pi(float(a_try[0] - prev[0]))
                    d1 = wrap_to_pi(float(a_try[1] - prev[1]))
                    cost = d0*d0 + d1*d1
                else:
                    cost = float(a_try[0]*a_try[0] + a_try[1]*a_try[1])

                if cost < best_cost:
                    best_cost = cost
                    best_alpha = a_try
                    best_branch = np.array([b0, b1], dtype=int)

        # Residual (paper equation)
        residual = np.zeros(2, dtype=float)
        for i in range(2):
            dy = float(d_tilde[i,1])
            dz = float(d_tilde[i,2])
            a = float(best_alpha[i])
            residual[i] = dy*math.cos(a) + dz*math.sin(a) - float(k[i])

        return SolveResult(
            feasible=True,
            alpha=best_alpha,
            branch=best_branch,
            k=k, rho=rho, asin_arg=asin_arg,
            residual=residual,
            d=d, d_hat=d_hat, d_tilde=d_tilde
        )
