import time
import numpy as np
from pr2_ikfast import solve_ik
from skrobot.models.pr2 import PR2
from skrobot.viewers import TrimeshSceneViewer

torso_angle = 0.1
lb = np.array([-2.28539816, -0.5236, -3.9, -2.3213, -6.28318531, -2.18, -6.28318531])
ub = np.array([0.71460184, 1.3963, 0.8, 0.0, 6.28318531, 0.0, 6.28318531])

pr2 = PR2()
joints = [
    pr2.r_shoulder_pan_joint,
    pr2.r_shoulder_lift_joint,
    pr2.r_upper_arm_roll_joint,
    pr2.r_elbow_flex_joint,
    pr2.r_forearm_roll_joint,
    pr2.r_wrist_flex_joint,
    pr2.r_wrist_roll_joint,
]
pr2.torso_lift_joint.joint_angle(torso_angle)
v = TrimeshSceneViewer()
v.add(pr2)
v.show()


for _ in range(1000):
    ts = time.time()
    ret = solve_ik([0.5, -0.2, 0.8], np.eye(3).tolist(), torso_angle, True, lb, ub)
    print(f"ikfast took {time.time() - ts:.4f} seconds")
    for joint, angle in zip(joints, ret):
        joint.joint_angle(angle)
    v.redraw()
    input("Press Enter to continue...")
