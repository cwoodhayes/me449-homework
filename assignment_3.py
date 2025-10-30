from dataclasses import dataclass
from pathlib import Path
from typing import Generator

from modern_robotics import IKinBody
import numpy as np

from lib.printers import print_readable_and_answer, print_readable


@dataclass
class IKIterT:
    idx: int
    theta_i: np.ndarray  # joint vector theta_i
    T_sb: np.ndarray  # EE config T_sb(theta_i)
    V_b: np.ndarray  # error twist V_b
    err_wb: float  # angular error magnitude ||w_b||
    err_vb: float  # linear error magnitude ||v_b||


def ik_in_body_iterator(
    x_dest: np.ndarray, theta0: np.ndarray
) -> Generator[IKIterT, None, None]:
    """Helper iterator for numerical inverse kinematics.

    This generator function yields an intermediate solution
    for each iteration of the newton-raphson method.

    Args:
        x_dest (np.ndarray): destination EE position
        theta0 (np.ndarray): initial guess joint vector

    Yields:
        IKIterT: iteration return type (defined above)
    """
    IKinBody()

    idx = 0
    while True:
        idx += 1


def display_iter(iter: IKIterT) -> None:
    """Prettyprint the per-iteration bookkeeping values"""
    print(f"Iteration {iter.idx}:\n")
    print_readable(iter.theta_i, "joint vector:")
    print()
    print_readable(iter.T_sb, "SE(3) end-effector config:")
    print_readable(iter.V_b, "error twist V_b")
    print(f"angular error: {iter.err_wb}")
    print(f"linear error: {iter.err_vb}")
    print()


def IKinBodyIterates(
    x_dest: np.ndarray,
    theta0: np.ndarray,
    epsilon_w: float,
    epsilon_v: float,
) -> np.ndarray:
    """Geometric iterative numerical inverse kinematics solver.

    Given an EE destination pose and an initial guess joint vector
    theta0, iteratively find the joint vector which results in
    x_dest.

    Args:
        x_dest (np.ndarray): _description_
        theta0 (np.ndarray): _description_
        epsilon_w (float): _description_
        epsilon_v (float): _description_

    Returns:
        np.ndarray: _description_
    """
    max_iter = 10
    thetas = []
    for iter in ik_in_body_iterator(x_dest, theta0):
        display_iter(iter)
        thetas.append(iter.theta_i)
        if iter.idx >= max_iter:
            print("Maximum iteration reached. Finishing...")
            break

    return np.array(thetas)


def main():
    T_sd = np.array(
        [
            [0, 0, -1, 0],
            [1, 0, 0, 0.6],
            [0, -1, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    IKinBodyIterates()


if __name__ == "__main__":
    main()
