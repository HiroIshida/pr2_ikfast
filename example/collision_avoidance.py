from plainmp.robot_spec import PR2RarmSpec
from plainmp.utils import primitive_to_plainmp_sdf
import time
import numpy as np
from pr2_ikfast import sample_ik_solution
from skrobot.models.pr2 import PR2
from skrobot.model.primitives import Box
from skrobot.viewers import TrimeshSceneViewer

torso_angle = 0.1
pr2 = PR2(use_tight_joint_limit=False)
pr2.reset_pose()
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
    for sol in sample_ik_solution([0.6, -0.2, 0.8], np.eye(3).tolist(), torso_angle, True):
        if not (np.all(sol >= lb) and np.all(sol <= ub)):
            continue
        if ineq_cst.is_valid(sol):
            print("found solution")
            break
    print(f"ikfast with plainmp collision avoidance took {time.time() - ts:.4f} seconds")
    spec.set_skrobot_model_state(pr2, sol)
    v.redraw()
    input("Press Enter to continue...")
