from modern_robotics import (
    InverseDynamics,
    QuinticTimeScaling,
    ScrewTrajectory,
    CartesianTrajectory,
)

import numpy as np

from lib.printers import print_readable_and_answer, print_readable


def q5():
    out = QuinticTimeScaling(5, 3)
    print(out)


def q6():
    Xs = np.eye(4)
    Xe = np.array(
        [
            [0, 0, 1, 1],
            [1, 0, 0, 2],
            [0, 1, 0, 3],
            [0, 0, 0, 1],
        ]
    )
    out = ScrewTrajectory(Xs, Xe, 10, 10, 3)
    print(len(out))
    print_readable_and_answer(out[-2])  # pyright: ignore[reportArgumentType]


def q7():
    Xs = np.eye(4)
    Xe = np.array(
        [
            [0, 0, 1, 1],
            [1, 0, 0, 2],
            [0, 1, 0, 3],
            [0, 0, 0, 1],
        ]
    )
    out = CartesianTrajectory(Xs, Xe, 10, 10, 5)
    print_readable_and_answer(out[-2])  # pyright: ignore[reportArgumentType]


if __name__ == "__main__":
    # q2:
    print("################### q2:")
    print("10 * (1 * t / T) ** 3 - 15 * (1 * t / T) ** 4 + 6 * (t / T) ** 5")

    q5()
    q6()
    q7()
