from plainmp.robot_spec import PR2RarmSpec
from plainmp.utils import primitive_to_plainmp_sdf
import time
import numpy as np
from pr2_ikfast import solve_ik
from skrobot.models.pr2 import PR2
from skrobot.model.primitives import Box
from skrobot.viewers import TrimeshSceneViewer

torso_angle = 0.1
pr2 = PR2()
pr2.reset_pose()
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

obstacle = Box([0.3, 0.3, 0.3])
obstacle.translate([0.4, -0.3, 0.5])

spec = PR2RarmSpec()
lb, ub = spec.angle_bounds()
ineq_cst = spec.create_collision_const()
ineq_cst.set_sdf(primitive_to_plainmp_sdf(obstacle))
spec.reflect_skrobot_model_to_kin(pr2)

v = TrimeshSceneViewer()
v.add(pr2)
v.add(obstacle)
v.show()


for _ in range(1000):
    ts = time.time()
    ret = solve_ik([0.6, -0.2, 0.8], np.eye(3).tolist(), torso_angle, True, lb, ub, predicate = lambda x: ineq_cst.is_valid(x))
    print(f"ikfast with plainmp collision avoidance took {time.time() - ts:.4f} seconds")
    for joint, angle in zip(joints, ret):
        joint.joint_angle(angle)
    v.redraw()
    input("Press Enter to continue...")
