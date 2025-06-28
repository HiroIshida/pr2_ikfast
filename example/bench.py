import time
import numpy as np
from pr2_ikfast import solve_ik

torso_angle = 0.1
lb = np.array([-2.28539816, -0.5236, -3.9, -2.3213, -6.28318531, -2.18, -6.28318531])
ub = np.array([0.71460184, 1.3963, 0.8, 0.0, 6.28318531, 0.0, 6.28318531])

elapsed_list = []
for _ in range(1000):
    ts = time.time()
    ret = solve_ik([0.5, -0.2, 0.8], np.eye(3).tolist(), torso_angle, True, lb, ub)
    elapsed = time.time() - ts
    elapsed_list.append(elapsed)

print(f"mean elapsed time: {np.mean(elapsed_list) * 1000} ms")
