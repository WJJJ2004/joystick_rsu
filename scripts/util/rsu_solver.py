# rsu_solver.py
# RSU closed-form IK, paper-faithful implementation with previous variable naming
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
    branch: np.ndarray        # shape (2,) int (0 or 1)
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


class RSUSolver:
    def __init__(self, params: RSUParams):
        self.p = params

    @staticmethod
    def _alpha_candidates_paper_yz(dy: float, dz: float, k: float, eps: float) -> Tuple[List[float], List[bool], float, float]:
        """
        Compute two alpha candidates following the paper derivation for
        the scalar equation: dy*cos(alpha) + dz*sin(alpha) = k.
        Returns:
          (alphas_list, valids_list, rho, raw_arg)
        where raw_arg is k/rho (before clipping).
        """
        rho = math.hypot(dy, dz)
        if rho <= eps:
            # degenerate: dy,dz nearly zero -> equation reduces to 0 = k
            if abs(k) <= eps:
                # effectively infinite solutions, choose canonical 0 for both branches
                return [0.0, 0.0], [True, True], rho, 0.0
            else:
                return [], [False, False], rho, float('nan')

        raw_arg = k / rho
        if abs(raw_arg) > 1.0 + 1e-12:
            return [], [False, False], rho, raw_arg

        arg_clipped = max(-1.0, min(1.0, raw_arg))
        asin_val = math.asin(arg_clipped)
        phi_tilde = math.atan2(dy, dz)

        alpha_a = -phi_tilde + asin_val
        alpha_b = -phi_tilde + math.pi - asin_val

        return [alpha_a, alpha_b], [True, True], rho, raw_arg

    def solve(self, roll: float, pitch: float, prev_alpha: Optional[np.ndarray] = None) -> SolveResult:
        """
        Solve RSU IK for both legs given foot roll and pitch.
        prev_alpha: optional shape (2,) previous alpha to select continuous branch
        """

        p = self.p

        # outputs initialization (previous variable names preserved)
        alpha = np.zeros(2, dtype=float)
        branch = np.full(2, -1, dtype=int)
        k = np.full(2, float('nan'), dtype=float)
        rho = np.zeros(2, dtype=float)
        asin_arg = np.full(2, float('nan'), dtype=float)
        residual = np.full(2, float('nan'), dtype=float)
        d = np.zeros((2,3), dtype=float)
        d_hat = np.zeros((2,3), dtype=float)
        d_tilde = np.zeros((2,3), dtype=float)

        feasible = True

        # compute foot rotation as in paper: W R_F = Ry(pitch) * Rx(roll)
        Rf = Ry(pitch) @ Rx(roll)

        for i in range(2):
            aW = p.a_W[i, :]
            bF = p.b_F[i, :]
            ci = float(p.c[i])
            ri = float(p.r[i])
            psi_i = float(p.psi[i])

            # d_i = a_W - Rf * b_F
            di = aW - (Rf @ bF)
            d[i, :] = di
            di_norm = np.linalg.norm(di)
            if di_norm <= p.eps:
                feasible = False
                alpha[i] = 0.0
                branch[i] = -1
                k[i] = float('nan')
                rho[i] = 0.0
                asin_arg[i] = float('nan')
                residual[i] = float('nan')
                d_hat[i, :] = np.zeros(3)
                d_tilde[i, :] = np.zeros(3)
                continue

            dhi = di / di_norm
            d_hat[i, :] = dhi

            # k_i per paper
            ki = (ri*ri - ci*ci - di_norm*di_norm) / (2.0 * ci * di_norm)
            k[i] = ki

            # tilde = Rz(-psi) @ d_hat  (paper defines tilde^T = (W hat d)^T Rz(psi))
            til = Rz(-psi_i) @ dhi
            d_tilde[i, :] = til
            dy = float(til[1])
            dz = float(til[2])

            # candidate alphas
            alphas, valids, rho_i, raw_arg = self._alpha_candidates_paper_yz(dy, dz, ki, p.eps)
            rho[i] = rho_i
            asin_arg[i] = raw_arg

            if not alphas:
                feasible = False
                alpha[i] = 0.0
                branch[i] = -1
                residual[i] = float('nan')
                continue

            # evaluate residuals of the two candidates and choose
            cand_wrapped = []
            cand_res = []
            for a in alphas:
                a_wr = wrap_to_pi(a)
                cand_wrapped.append(a_wr)
                res = dy * math.cos(a_wr) + dz * math.sin(a_wr) - ki
                cand_res.append(res)

            # branch selection
            chosen_idx = 0
            if prev_alpha is not None and np.isfinite(prev_alpha[i]):
                prev = float(prev_alpha[i])
                diffs = [abs(wrap_to_pi(a_wr - prev)) for a_wr in cand_wrapped]
                chosen_idx = int(np.argmin(diffs))
            else:
                abs_res = [abs(rv) for rv in cand_res]
                chosen_idx = int(np.argmin(abs_res))

            alpha[i] = cand_wrapped[chosen_idx]
            branch[i] = chosen_idx
            residual[i] = cand_res[chosen_idx]

        return SolveResult(
            feasible=feasible,
            alpha=alpha,
            branch=branch,
            k=k,
            rho=rho,
            asin_arg=asin_arg,
            residual=residual,
            d=d,
            d_hat=d_hat,
            d_tilde=d_tilde
        )


# Example usage
if __name__ == "__main__":
    W_a = np.array([[-86.0, 40.0, 235.0],
                    [-86.0, -40.0, 235.0]])
    F_b = np.array([[-34.0, 36.0, 36.0],
                    [-34.0, -36.0, 36.0]])
    c = np.array([40.0, 40.0])
    r = np.array([80.0, 80.0])
    psi = np.deg2rad(np.array([-90.0, 90.0]))

    params = RSUParams(a_W=W_a, b_F=F_b, c=c, r=r, psi=psi, eps=1e-9)
    solver = RSUSolver(params)

    roll = math.radians(5.0)
    pitch = math.radians(-10.0)

    res = solver.solve(roll, pitch)
    print("feasible:", res.feasible)
    print("alpha (rad):", res.alpha)
    print("branch:", res.branch)
    print("residual:", res.residual)
    print("k:", res.k)
    print("rho:", res.rho)