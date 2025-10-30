from dataclasses import dataclass
from pathlib import Path

from modern_robotics import se3ToVec, MatrixLog6, TransInv, FKinBody, JacobianBody
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


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot

    Based on modern_robotics.IKinBody()

    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.

    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Output:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    """
    thetalist: np.ndarray = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    T_i = TransInv(FKinBody(M, Blist, thetalist))
    Vb = se3ToVec(MatrixLog6(np.dot(T_i, T)))
    err = (
        np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg
        or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    )
    while err and i < maxiterations:
        thetalist = thetalist + np.dot(
            np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb
        )
        T_i = TransInv(FKinBody(M, Blist, thetalist))
        Vb = se3ToVec(MatrixLog6(np.dot(T_i, T)))
        err_wb = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        err_vb = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err = err_wb > eomg or err_vb > ev

        iter = IKIterT(i, thetalist, T_i, Vb, float(err_wb), float(err_vb))
        display_iter(iter)

        i = i + 1
    return (thetalist, not err)


def main():
    T_sd = np.array(
        [
            [0, 0, -1, 0],
            [1, 0, 0, 0.6],
            [0, -1, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    e_w = 0.001
    e_v = 0.0001

    theta_short_iterates = np.array([0, 0, 0, 0, 0, 0])

    # UR5 constants:
    W1 = 0.109
    W2 = 0.082
    L1 = 0.425
    L2 = 0.392
    H1 = 0.089
    H2 = 0.095
    Blist = np.array(
        [
            [0, 0, 1, 0, 0, 0],
            [0, 1, 0, -H1, 0, 0],
            [0, 1, 0, -H1, 0, L1],
            [0, 1, 0, -H1, 0, L1 + L2],
            [0, 0, -1, -W1, L1 + L2, 0],
            [0, 1, 0, H2 - H1, 0, L1 + L2],
        ]
    ).T
    M = np.array(
        [
            [-1, 0, 0, L1 + L2],
            [0, 0, 1, W1 + W2],
            [0, 1, 0, H1 - H2],
            [0, 0, 0, 1],
        ]
    )

    thetalist, success = IKinBodyIterates(
        Blist, M, T_sd, theta_short_iterates, e_w, e_v
    )
    print_readable(
        thetalist, f"theta_d ({'CONVERGED' if success else 'NO CONVERGENCE'})"
    )


if __name__ == "__main__":
    main()
