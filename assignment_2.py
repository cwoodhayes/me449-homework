from modern_robotics import FKinBody, FKinSpace
import numpy as np

from lib.printers import print_readable_and_answer


def main():
    ex_4_2()


def ex_4_2():
    print("##################4.2")
    M = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 2],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
        ]
    )
    S_space = np.array(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1, 1, 1, 0],
            [0, 1, 2, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    S_body = np.array(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1, 1, 1, 0],
            [-2, -1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    theta = np.array([0, np.pi / 2, -np.pi / 2, 1])

    T_space = FKinSpace(M, S_space, theta)
    T_body = FKinBody(M, S_body, theta)
    print_readable_and_answer(T_space, "T_space")
    print_readable_and_answer(T_body, "T_body")
    pass


if __name__ == "__main__":
    main()
