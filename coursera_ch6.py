from modern_robotics import IKinSpace

import numpy as np

from lib.printers import print_readable_and_answer, print_readable


def newton_raphson(g, theta_k):
    """takes in g(theta) and theta_k, and returns theta_k+1.

    Converges on theta s.t. g(theta) = 0 after many calls

    Args:
        g: function g(theta)
        theta_k: k'th guess for theta
    """
    pass
    # theta_1 = theta_k - np.linalg.


def q1():
    print("################## Q1")

    theta_2 = np.array(
        [
            5 + 8 / 5.0,
            5 / 2.0 + 9.0 / 20,
        ]
    )
    print_readable_and_answer(theta_2, "theta_2")


def q2():
    print("################## Q2")

    T_sd = np.array(
        [
            [-0.585, -0.811, 0, 0.076],
            [0.811, -0.585, 0, 2.608],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    theta_0 = np.array([np.pi / 4, np.pi / 4, np.pi / 4])
    Slist = np.array(
        [
            [0, 0, 1, 0, -1, 0],
            [0, 0, 1, 0, -2, 0],
            [0, 0, 1, 0, -3, 0],
        ]
    ).T
    M = np.array(
        [
            [1, 0, 0, 3],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )

    theta, success = IKinSpace(Slist, M, T_sd, theta_0, 0.001, 0.0001)
    if type(theta) is np.ndarray:
        print_readable_and_answer(theta, "theta (solution)")
    if not success:
        print("FAILED TO CONVERGE")


if __name__ == "__main__":
    # q1()
    q2()
