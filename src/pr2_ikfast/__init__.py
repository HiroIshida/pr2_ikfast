from pr2_ikfast.ikLeft import leftIK
from pr2_ikfast.ikRight import rightIK
from typing import List, Callable, Optional, Iterator, Generator
import numpy as np
from dataclasses import dataclass


# lowest level IK solver for the left arm


def solve_left_ik(trans: List[float], rot: List[float], free_vals: List[float]):
    return leftIK(rot, trans, free_vals)


def solve_right_ik(trans: List[float], rot: List[float], free_vals: List[float]):
    return rightIK(rot, trans, free_vals)


# More convenient interface for the left arm IK solver


@dataclass
class UniformSampler:
    n_size: int = 72
    lb: float = -np.pi
    ub: float = np.pi

    def __call__(self) -> Iterator[float]:
        for _ in range(self.n_size):
            yield np.random.uniform(self.lb, self.ub)


def sample_ik_solution(
    trans: List[float],
    rot: List[List[float]],
    torso_value: float,
    is_rarm: bool,
    sampler: Optional[Callable[[], Iterator[float]]] = None,
    batch: bool = False
    ) -> Generator[np.ndarray, None, None]:

    # use -0.051, because original cpp code is compiled wrt baes_footprint
    trans_modif = [trans[0], trans[1], trans[2] - 0.051]

    if sampler is None:
        sampler = UniformSampler()
    fn = solve_right_ik if is_rarm else solve_left_ik
    for val2 in sampler():
        free_vals = [torso_value, val2]
        retall = fn(trans_modif, rot, free_vals)
        if retall is None:
            continue
        if batch:
            yield np.array([ret[1:] for ret in retall])
        else:
            for ret in retall:
                yield ret[1:]
