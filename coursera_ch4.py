"""
coursera ch4 work
"""

import numpy as np
import modern_robotics


np.set_printoptions(precision=2, suppress=True)


def print_readable_and_answer(arr: np.ndarray) -> None:
    print("READABLE:")
    print(arr)
    print("COPYABLE:")
    print(repr(arr))


def q1():
    print("#################### Q1 ##################")
    L = 1
    T = np.array(
        [
            [1, 0, 0, (2 + np.sqrt(3)) * L],
            [0, 1, 0, 0],
            [0, 0, 1, (1 + np.sqrt(3)) * L],
            [0, 0, 0, 1],
        ]
    )
    print_readable_and_answer(T)


def q2():
    print("#################### Q2 ##################")
    L = 1
    T = np.array(
        [
            [0, 0, 1, 0, -L, 0],
            [0, 1, 0, 0, 0, L],
            [0, 1, 0, L, 0, (1 + np.sqrt(3)) * L],
            [0, 1, 0, -(np.sqrt(3) - 1) * L, 0, (2 + np.sqrt(3)) * L],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 1, 0, -(2 + np.sqrt(3)) * L, 0],
        ]
    ).T

    print_readable_and_answer(T)


def q3():
    print("#################### Q3 ##################")
    T = np.array(
        [
            [0, 0, 1, 0, (np.sqrt(3) + 1), 0],
            [0, 1, 0, (np.sqrt(3) + 1), 0, np.sqrt(3) + 1],
            [0, 1, 0, (np.sqrt(3) + 2), 0, -1],
            [0, 1, 0, 2, 0, 0],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 1, 0, 0, 0],
        ]
    ).T
    print_readable_and_answer(T)


def q4():
    print("#################### Q4 ##################")
    L = 1
    M = np.array(
        [
            [1, 0, 0, (2 + np.sqrt(3)) * L],
            [0, 1, 0, 0],
            [0, 0, 1, (1 + np.sqrt(3)) * L],
            [0, 0, 0, 1],
        ]
    )
    S = np.array(
        [
            [0, 0, 1, 0, -L, 0],
            [0, 1, 0, 0, 0, L],
            [0, 1, 0, L, 0, (1 + np.sqrt(3)) * L],
            [0, 1, 0, -(np.sqrt(3) - 1) * L, 0, (2 + np.sqrt(3)) * L],
            [0, 0, 0, 0, 0, 1],
            [0, 0, 1, 0, -(2 + np.sqrt(3)) * L, 0],
        ]
    ).T

    theta = [-np.pi / 2, np.pi / 2, np.pi / 3, -np.pi / 4, 1, np.pi / 6]
    out = modern_robotics.FKinSpace(M, S, theta)
    print_readable_and_answer(out)


if __name__ == "__main__":
    q1()
    q2()
    q3()
    q4()
