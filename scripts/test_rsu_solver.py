"""
RSU 파라메터 테이블 버전 > prototype 버전으로 업데이트 완료
"""




import numpy as np
import math
from util.rsu_solver import RSUParams, RSUSolver

def deg2rad(d): return d * math.pi / 180.0

p = RSUParams(
    a_W=np.array([[0.0,  36.0, 170.0],
                  [0.0, -36.0,  82.0]]),
    b_F=np.array([[-20.0,  36.0, 16.0],
                  [-20.0, -36.0, 16.0]]),
    c=np.array([30.0, -30.0]),
    r=np.array([154.0, 66.0]),
    psi=np.array([deg2rad(90), deg2rad(-90)]),
)

# p = RSUParams(
#     a_W=np.array([[-30,  36, 170],
#                   [-30, -36,  82]]),
#     b_F=np.array([[-30,  36, 0],    # x = -30 으로 맞춤
#                   [-30, -36, 0]]),
#     c=np.array([30, 30]),
#     r=np.array([170.0, 82.0]),
#     psi=np.array([deg2rad(90), deg2rad(90)]),
# )

solver = RSUSolver(p)

roll = 0.0
pitch = 0.05
prev = np.array([0.0, 0.0])

for i in range(100):
    res = solver.solve(roll, pitch, prev)
    
    err0 = abs(min((pitch - res.alpha[0]),(pitch + res.alpha[0])))
    err1 = abs(min((pitch - res.alpha[1]),(pitch + res.alpha[1])))
    
    print(f"=== iter {i:3d} | pitch={pitch:.4f} rad ===")
    print(f"  alpha[0]={res.alpha[0]:.6f}  err={err0:.6f}  branch={res.branch[0]}")
    print(f"  alpha[1]={res.alpha[1]:.6f}  err={err1:.6f}  branch={res.branch[1]}")
    print(f"  k={res.k}  rho={res.rho}  asin_arg={res.asin_arg}")
    print(f"  residual={res.residual}")
    
    if err0 > 0.01:
        print("ERROR IN CRANK 0 SOLUTION!")
    if err1 > 0.01:
        print("ERROR IN CRANK 1 SOLUTION!")
    pitch += 0.01
    prev = res.alpha.copy()  # 이전 해로 업데이트
