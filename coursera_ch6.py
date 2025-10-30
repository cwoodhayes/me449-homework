from modern_robotics import IKinSpace

import numpy as np
import sympy as sym

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

    x, y = sym.symbols("x,y")
    theta = sym.Matrix([[x], [y]])
    f_xy = sym.Matrix(
        [
            [x**2 - 9],
            [y**2 - 4],
        ]
    )

    jac = f_xy.jacobian(theta)
    jac_inv = jac.inv()
    print_readable(jac_inv, "J^-1")

    print_readable(jac_inv @ f_xy, "J^-1 @ f(x,y)")
    print_readable(theta, "theta_k")
    theta_next = theta - (jac_inv @ f_xy)
    print_readable(theta_next, "theta_k+1")

    func = sym.lambdify([x, y], theta_next)
    theta1 = func(1, 1)
    print_readable(theta1)
    theta2 = func(*theta1.flatten())
    print_readable_and_answer(theta2, "theta_2")


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
            [0, 0, 1, 0, 0, 0],
            [0, 0, 1, 0, -1, 0],
            [0, 0, 1, 0, -2, 0],
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
    q1()
    q2()
