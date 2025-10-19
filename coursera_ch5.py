from modern_robotics import JacobianSpace, JacobianBody, Adjoint
import numpy as np

from lib.printers import print_readable_and_answer, print_readable


def q1():
    print("################## Q1")
    r = np.array([0, 0, 0])
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

    # return

    # for my own curiosity, i want to figure out what this wrench is when expressed in the body frame
    # like should it go up and to the right, so that the line of force goes thru the origin? so no moment,
    # but an angled force?
    Tsb = np.array(
        [
            [1, 0, 0, 2 + np.cos(np.pi / 4)],
            [0, 1, 0, np.sin(np.pi / 4)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    Ad_Tbs = Adjoint(np.linalg.inv(Tsb))
    F_b = Ad_Tbs.T @ F_s
    print_readable(F_b.reshape(-1, 1), "F_b (body wrench)")


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


def q5q6():
    print("################## Q5")

    J_bv = np.array(
        [
            [-0.104, 0, 0.006, -0.045, 0, 0.006, 0],
            [-0.889, 0.006, 0, -0.844, 0.006, 0, 0],
            [0, -0.105, 0.889, 0, 0, 0, 0],
        ]
    )
    A = J_bv @ J_bv.T
    eigs = np.linalg.eig(A)

    print(eigs.eigenvalues)
    print(eigs.eigenvectors)

    # eigenvalue idx 1 is the largest (by inspection of the above)
    print_readable_and_answer(eigs.eigenvectors[1], "principal axis unit vec")

    print_readable_and_answer(np.sqrt(eigs.eigenvalues[1]), "principal axis length")


if __name__ == "__main__":
    q1()
    # q2()
    # q3()
    # q4()
    # q5q6()
