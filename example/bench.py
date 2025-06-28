import time
import numpy as np
from pr2_ikfast import sample_ik_solution

torso_angle = 0.1
lb = np.array([-2.28539816, -0.5236, -3.9, -2.3213, -6.28318531, -2.18, -6.28318531])
ub = np.array([0.71460184, 1.3963, 0.8, 0.0, 6.28318531, 0.0, 6.28318531])

ts_list = []
for _ in range(10000):
    ts = time.time()
    for sol in  sample_ik_solution([0.5, -0.2, 0.8], np.eye(3).tolist(), torso_angle, True):
        if np.all(sol >= lb) and np.all(sol <= ub):
            break
    ts_list.append(time.time() - ts)
print(f"mean: {np.mean(ts_list)}")
