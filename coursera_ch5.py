from modern_robotics import JacobianSpace, JacobianBody
import numpy as np

from lib.printers import print_readable_and_answer, print_readable


def q1():
    print("################## Q1")
    r = np.array([1 + np.cos(np.pi / 4), np.sin(np.pi / 4), 0])
    f = np.array([2, 0, 0])
    p = np.cross(r, f)

    F_s = np.concat([p, f])
    F_s_col = F_s.reshape(-1, 1)
    print_readable(F_s_col, "F_s")

    J_s = np.array(
        [
            [0, 0, 0],
            [0, 0, 0],
            [1, 1, 1],
            [0, 0, np.sin(np.pi / 4)],
            [0, -1, -(1 + np.cos(np.pi / 4))],
            [0, 0, 0],
        ]
    )

    tau = J_s.T @ F_s_col

    print_readable_and_answer(tau.flatten(), "tau")


def q2():
    print("################## Q2")

    F_b = np.array([0, 0, 10, 10, 10, 0])
    F_b_col = F_b.reshape(-1, 1)
    print_readable(F_b_col, "F_b")

    J_b = np.array(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1, 1, 1, 1],
            [-1, -1, -1, 0],
            [3, 2, 1, 1],
            [0, 0, 0, 0],
        ]
    )

    tau = J_b.T @ F_b_col

    print_readable_and_answer(tau.flatten(), "tau")


def q3():
    print("################## Q3")

    Slist = np.array(
        [
            [0, 1, 0],
            [0, 0, 0],
            [1, 0, 0],
            [0, 0, 0],
            [0, 2, 1],
            [0, 0, 0],
        ]
    )
    theta = np.array([np.pi / 2, np.pi / 2, 1])

    J = JacobianSpace(Slist, theta)

    print_readable_and_answer(J, "Jacobian")


def q4():
    print("################## Q4")

    Slist = np.array(
        [
            [0, -1, 0],
            [1, 0, 0],
            [0, 0, 0],
            [3, 0, 0],
            [0, 3, 0],
            [0, 0, 1],
        ]
    )
    theta = np.array([np.pi / 2, np.pi / 2, 1])

    J = JacobianBody(Slist, theta)

    print_readable_and_answer(J, "Jacobian")


if __name__ == "__main__":
    # q1()
    # q2()
    # q3()
    q4()
