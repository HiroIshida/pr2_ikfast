from pr2_ikfast.ikLeft import leftIK
from pr2_ikfast.ikRight import rightIK
from typing import List


def solve_left_ik(trans: List[float], rot: List[List[float]], free_vals: List[float]):
    return leftIK(rot, trans, free_vals)

def solve_right_ik(trans: List[float], rot: List[List[float]], free_vals: List[float]):
    return rightIK(rot, trans, free_vals)
