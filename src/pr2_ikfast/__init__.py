from pr2_ikfast.ikLeft import leftIK
from pr2_ikfast.ikRight import rightIK
from typing import List, Callable, Optional
import numpy as np
from dataclasses import dataclass


# lowest level IK solver for the left arm


def solve_left_ik(trans: List[float], rot: List[float], free_vals: List[float]):
    return leftIK(rot, trans, free_vals)


def solve_right_ik(trans: List[float], rot: List[float], free_vals: List[float]):
    return rightIK(rot, trans, free_vals)


# More convenient interface for the left arm IK solver


@dataclass
class UnoformSampler:
    n_size: int = 72
    lb: float = 0.5 * -np.pi
    ub: float = 0.5 * np.pi

    def __call__(self) -> np.ndarray:
        return np.random.uniform(self.lb, self.ub, size=(self.n_size,))


def solve_ik(
    trans: List[float],
    rot: List[List[float]],
    torso_value: float,
    is_rarm: bool,
    lb: Optional[np.ndarray],
    ub: Optional[np.ndarray],
    sampler: Optional[Callable[[], np.ndarray]] = None,
) -> List[float]:
    if sampler is None:
        sampler = UnoformSampler()
    fn = solve_right_ik if is_rarm else solve_left_ik
    upper_arm_roll_joint_vals = sampler()
    for val2 in upper_arm_roll_joint_vals:
        free_vals = [torso_value, val2]
        retall = fn(trans, rot, free_vals)
        if lb is not None and ub is not None:
            for ret in retall:
                if np.all(ret[1:] >= lb) and np.all(ret[1:] <= ub):
                    return ret[1:]
