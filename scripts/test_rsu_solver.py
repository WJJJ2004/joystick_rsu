import numpy as np
import math
from rsu_solver import RSUParams, RSUSolver

def deg2rad(d): return d * math.pi / 180.0

p = RSUParams(
    a_W=np.array([[0,  36, 169],
                  [0, -36,  81]]),
    b_F=np.array([[-30,  36, 0],
                  [-30, -36, 0]]),
    c=np.array([30, 30]),
    r=np.array([169.5, 81.0]),
    psi=np.array([deg2rad(-90), deg2rad(90)]),
)

solver = RSUSolver(p)

roll = 0.10
pitch = -0.05
prev = np.array([0.0, 0.0])

res = solver.solve(roll, pitch, prev)

print("feasible:", res.feasible)
print("alpha:", res.alpha, "rad")
print("branch:", res.branch)
print("residual:", res.residual)
print("k:", res.k)
print("rho:", res.rho)
print("asin_arg:", res.asin_arg)
