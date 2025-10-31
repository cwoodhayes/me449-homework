from dataclasses import dataclass
from pathlib import Path
import signal

from modern_robotics import se3ToVec, MatrixLog6, TransInv, FKinBody, JacobianBody
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

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
    print("\njoint vector:")
    print_readable(iter.theta_i, ndigits=2)
    print("\nSE(3) end-effector config:")
    print_readable(iter.T_sb, ndigits=2)
    print("\nerror twist V_b:")
    print_readable(iter.V_b, ndigits=2)
    print(f"\nangular error:   {round(iter.err_wb, 3)}")
    print(f" linear error:   {round(iter.err_vb, 3)}")
    print()


def IKinBodyIterates(
    Blist, M, T, thetalist0, eomg, ev, maxiterations: int = 20
) -> tuple[np.ndarray, bool, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    T_i = FKinBody(M, Blist, thetalist)
    Vb = se3ToVec(MatrixLog6(np.dot(TransInv(T_i), T)))
    werr = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    verr = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    err = werr > eomg or verr > ev

    all_thetas = [thetalist]
    all_Ts = [T_i]
    werrs = [werr]
    verrs = [verr]

    while err and i < maxiterations:
        thetalist = thetalist + np.dot(
            np.linalg.pinv(JacobianBody(Blist, thetalist)), Vb
        )
        T_i = FKinBody(M, Blist, thetalist)
        Vb = se3ToVec(MatrixLog6(np.dot(TransInv(T_i), T)))
        werr = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        verr = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err = werr > eomg or verr > ev

        all_thetas.append(thetalist)
        all_Ts.append(T_i)
        werrs.append(werr)
        verrs.append(verr)

        iter = IKIterT(i, thetalist, T_i, Vb, float(werr), float(verr))
        display_iter(iter)

        i = i + 1
    return (
        thetalist,
        not err,
        np.array(all_thetas),
        np.array(all_Ts),
        np.array(werrs),
        np.array(verrs),
    )


def plot_3d_traj(
    Ts: np.ndarray,
    ax: Axes,
    label: str,
) -> None:
    ps = Ts[:, 0:4, 3]
    ax.plot(ps[:, 0], ps[:, 1], zs=ps[:, 2], zdir="z", label=label)
    ax.legend()
    ax.scatter(
        [ps[0, 0]],
        [ps[0, 1]],
        [ps[0, 2]],
        c="orange",
        label="initial guess",
    )
    ax.scatter(
        [ps[-1, 0]],
        [ps[-1, 1]],
        [ps[-1, 2]],
        c="black",
        marker="x",
        label="final position",
    )
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")  # type: ignore


def play_and_plot(theta_guess, Blist, M, T_sd, e_w, e_v, ax_3d, axw, axv, name: str):
    thetalist, success, all_thetas, all_Ts, all_errw, all_errv = IKinBodyIterates(
        Blist, M, T_sd, theta_guess, e_w, e_v
    )
    print_readable(
        thetalist,
        f"theta_d {name} ({'CONVERGED' if success else 'NO CONVERGENCE'})",
        ndigits=4,
    )

    plot_3d_traj(all_Ts, ax_3d, name)

    axw.plot(range(len(all_errw)), all_errw, label=name)
    axw.set_xlabel("iteration")
    axw.set_ylabel("Angular error (rads)")

    axv.plot(range(len(all_errv)), all_errv, label=name)
    axv.set_xlabel("iteration")
    axv.set_ylabel("Linear error (m)")

    axw.legend()
    axv.legend()


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

    # UR5 constants:
    W1 = 0.109
    W2 = 0.082
    L1 = 0.425
    L2 = 0.392
    H1 = 0.089
    H2 = 0.095
    Blist = np.array(
        [
            [0, 1, 0, W1 + W2, 0, L1 + L2],
            [0, 0, 1, H2, -L1 - L2, 0],
            [0, 0, 1, H2, -L2, 0],
            [0, 0, 1, H2, 0, 0],
            [0, -1, 0, -W2, 0, 0],
            [0, 0, 1, 0, 0, 0],
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

    ## Actual runs
    fig = plt.figure("3d_traj_plot")
    ax_3d = fig.add_subplot(projection="3d")

    fig2 = plt.figure("errs")
    axw, axv = fig2.subplots(2, 1)

    theta_short = np.array([1.5, -1.2, 1.7, -1.2, -0.3, -3.1])
    theta_long = np.array([4.9, 4.6, 2.1, 0.0, 2.8, -3.6])
    play_and_plot(theta_short, Blist, M, T_sd, e_w, e_v, ax_3d, axw, axv, "short")
    play_and_plot(theta_long, Blist, M, T_sd, e_w, e_v, ax_3d, axw, axv, "long")

    plt.show()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    main()
