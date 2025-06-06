import numpy as np
from pr2_ikfast import solve_left_ik, solve_right_ik

# torso_lift_joint and {}_upper_arm_roll_joint are free joints
# and we set them to [0.1, 0.2] for the example.
sol = solve_left_ik([0.5, -0.2, 0.8], np.eye(3).tolist(), [0.1, 0.2])
print(f"{len(sol)} solutions found for left arm")
sol = solve_right_ik([0.5, -0.2, 0.8], np.eye(3).tolist(), [0.1, 0.2])
print(f"{len(sol)} solutions found for right arm")


