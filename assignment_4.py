"""ME449 assignment 4."""

from enum import Enum, auto
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from modern_robotics import ForwardDynamics, EulerStep, FKinSpace
import numpy as np

from lib.printers import print_readable


@dataclass
class UR5Params:
    """
    UR5 arm dynamics parameters.

    Sourced from:
    https://hades.mech.northwestern.edu/images/d/d9/UR5-parameters-py.txt
    """

    M01 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]])
    M12 = np.array([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]])
    M23 = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]])
    M34 = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
    M45 = np.array([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]])
    M56 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]])
    M67 = np.array([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]])
    M = M01 @ M12 @ M23 @ M34 @ M45 @ M56 @ M67
    G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
    Glist = np.array([G1, G2, G3, G4, G5, G6])
    Mlist = np.array([M01, M12, M23, M34, M45, M56, M67])
    Slist = np.array(
        [
            [0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, -1, 0],
            [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
            [0, 0, 0, 0, 0.81725, 0],
            [0, 0, 0.425, 0.81725, 0, 0.81725],
        ]
    )


ur5 = UR5Params()


class PuppetConfig(Enum):
    """Configures behavior of puppet() function."""

    P1_FREEFALL = auto()
    P2_DAMPING = auto()
    P3_STATIONARY_SPRING = auto()
    P4_MOVING_SPRING = auto()


def puppet(
    thetalist: np.ndarray,
    dthetalist: np.ndarray,
    g: np.ndarray,
    Mlist: np.ndarray,
    Slist: np.ndarray,
    Glist: np.ndarray,
    t: float,
    dt: float,
    damping: float,
    stiffness: float,
    restLength: float,
    reference_pos: Callable[[float], np.ndarray],
    cfg: PuppetConfig = PuppetConfig.P4_MOVING_SPRING,
) -> tuple[np.ndarray, np.ndarray]:
    """Allows puppeteering of a UR5 robot arm.

    Args:
        thetalist (np.ndarray): n-vector of initial joint angles (units: rad)
        dthetalist (np.ndarray): n-vector of initial joint rates (units: rad/s)
        g (np.ndarray): the gravity 3-vector in the {s} frame (units: m/s2)
        Mlist (np.ndarray): the configurations of the link frames relative
            to each other at the home configuration. (There are eight frames
            total: {0} or {s} at the base of the robot, {1} . . .{6} at the
            centers of mass of the links, and {7} or {b} at the end-effector.)
        Slist (np.ndarray): The screw axes Si in the space frame when the robot
            is at its home configuration
        Glist (np.ndarray): spatial inertia matrices Gi of the links (units: kg and kg m2)
        t (float): total simulation time (units: s)
        dt (float): simulation timestep (units: s)
        damping (float): scalar indicating the viscous damping at each joint (units: Nms/rad)
        stiffness (str): scalar indicating stiffness of the springy string (units: N/m)
        restLength (float): scalar indicating the length of the spring at rest (units: m)

    Returns:
        tuple[np.ndarray, np.ndarray]:
            thetamat: an Nxn matrix where row i is the set of joint
                values after simulation step i - 1
            dthetmat: an N x n matrix where row i is the set of joint
                rates after simulation step i − 1
    """
    N = int(np.ceil(t / dt))
    thetamat = np.empty(shape=(N, 6))
    dthetamat = np.empty(shape=(N, 6))
    print(f"thetamat shape={thetamat.shape}")

    for i in range(N):
        thetamat[i] = thetalist
        dthetamat[i] = dthetalist

        # figure out tau and F_tip
        match cfg:
            case PuppetConfig.P1_FREEFALL:
                # no torques on joints; only g applied to the links
                taulist = np.zeros(6)

                # no force applied to EE
                Ftip = np.zeros(6, dtype="float")
            case PuppetConfig.P2_DAMPING:
                # joint torques caused by damping
                taulist = -dthetalist * damping

                # no force applied to EE
                Ftip = np.zeros(6, dtype="float")
            case PuppetConfig.P3_STATIONARY_SPRING | PuppetConfig.P4_MOVING_SPRING:
                # joint torques caused by damping
                taulist = -dthetalist * damping

                # get spring position in {s}
                spring_s = reference_pos(dt * i)
                spring_s = np.array([*spring_s, 1.0]).reshape(4, 1)

                # forward kinematics: T_s_b (space -> body mapping T_sb)
                T_sb = FKinSpace(ur5.M, Slist, thetalist)
                T_bs = np.linalg.inv(T_sb)

                # spring point and EE origin in body frame {b}
                spring_b = (T_bs @ spring_s)[:3, 0]
                ee_s = np.array([0.0, 0.0, 0.0, 1.0]).reshape(4, 1)
                ee_b = (T_bs @ ee_s)[:3, 0]

                # vector from spring -> EE (so r points from anchor to EE)
                r = ee_b - spring_b
                dist_mag = np.linalg.norm(r)

                # no force if zero distance or if spring is slack (optional, for cable)
                if dist_mag <= 1e-12:
                    F_on_EE = np.zeros(3)
                else:
                    extension = dist_mag - restLength
                    if extension <= 0.0:
                        force_mag = 0.0
                    else:
                        force_mag = (
                            -stiffness * extension
                        )  # negative => force on EE points toward spring
                    F_on_EE = force_mag * (r / dist_mag)

                # Ftip is the inverse of the force applied to the EE
                Ftip = np.hstack([np.zeros(3, dtype=float), -F_on_EE])

            case _:
                raise NotImplementedError("case fallthrough.")

        # forward dynamics given the above
        ddthetalist = ForwardDynamics(
            thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist
        )

        th_next, dth_next = EulerStep(thetalist, dthetalist, ddthetalist, dt)
        thetalist = np.mod(th_next, 2 * np.pi)
        dthetalist = dth_next

    return thetamat, dthetamat


def thetamat_to_csv(thetamat: np.ndarray, stem: str) -> None:
    """Write thetamat values to a csv file suitable for CoppelliaSim."""
    output_dir = Path(__file__).parent / "output"
    path = output_dir / f"{stem}.csv"
    if path.exists():
        path.unlink()

    print(f"- Writing to {path.name}")
    np.savetxt(path, thetamat, delimiter=",")


def part1() -> None:
    print("####### PART 1 ###############")

    g = np.array([0, 0, -9.81])
    home_thetas = np.zeros(6, dtype="float")
    rest_dthetas = np.zeros(6, dtype="float")
    t = 10.0
    dt_good = 0.001
    dt_explode = 0.02

    # where energy appears nearly constant
    thetamat, dthetamat = puppet(
        home_thetas,
        rest_dthetas,
        g,
        ur5.Mlist,
        ur5.Slist,
        ur5.Glist,
        t,
        dt_good,
        0.0,
        0.0,
        0.0,
        reference_pos=lambda t_curr: np.array([0.0, 0.0, 0.0]),
        cfg=PuppetConfig.P1_FREEFALL,
    )
    thetamat_to_csv(thetamat, "part1a")

    # where energy does not appear constant
    thetamat, dthetamat = puppet(
        home_thetas,
        rest_dthetas,
        g,
        ur5.Mlist,
        ur5.Slist,
        ur5.Glist,
        t,
        dt_explode,
        0.0,
        0.0,
        0.0,
        reference_pos=lambda t_curr: np.array([0.0, 0.0, 0.0]),
        cfg=PuppetConfig.P1_FREEFALL,
    )
    thetamat_to_csv(thetamat, "part1b")


def part2() -> None:
    print("####### PART 2 ###############")

    g = np.array([0, 0, -9.81])
    home_thetas = np.zeros(6, dtype="float")
    rest_dthetas = np.zeros(6, dtype="float")
    t = 10.0
    dt = 0.01

    names = ["part2a", "part2b"]
    damps = [1.0, -0.01]

    # names = ["part2-test"]
    # damps = [3.0]

    for name, damping in zip(names, damps):
        thetamat, dthetamat = puppet(
            home_thetas,
            rest_dthetas,
            g,
            ur5.Mlist,
            ur5.Slist,
            ur5.Glist,
            t,
            dt,
            damping,
            0.0,
            0.0,
            reference_pos=lambda t_curr: np.array([0.0, 0.0, 0.0]),
            cfg=PuppetConfig.P2_DAMPING,
        )
        thetamat_to_csv(thetamat, name)


def part3() -> None:
    print("####### PART 3 ###############")

    g = np.zeros(3)
    home_thetas = np.zeros(6, dtype="float")
    rest_dthetas = np.zeros(6, dtype="float")
    t = 10.0
    dt = 0.005
    rest_length = 0.0

    rpos = lambda t_curr: np.array([1.0, -1.0, 1.0])

    # create some small oscillations with the spring
    stiffness = 2.0
    damping = 0.001
    thetamat, dthetamat = puppet(
        home_thetas,
        rest_dthetas,
        g,
        ur5.Mlist,
        ur5.Slist,
        ur5.Glist,
        t,
        dt,
        damping,
        stiffness,
        rest_length,
        reference_pos=rpos,
        cfg=PuppetConfig.P3_STATIONARY_SPRING,
    )
    thetamat_to_csv(thetamat, "part3a")

    # add damping for part b
    stiffness = 100.0
    damping = 4.0
    thetamat, dthetamat = puppet(
        home_thetas,
        rest_dthetas,
        g,
        ur5.Mlist,
        ur5.Slist,
        ur5.Glist,
        t,
        dt,
        damping,
        stiffness,
        rest_length,
        reference_pos=rpos,
        cfg=PuppetConfig.P3_STATIONARY_SPRING,
    )
    thetamat_to_csv(thetamat, "part3b")


def part4() -> None:
    print("####### PART 4 ###############")

    g = np.zeros(3)
    home_thetas = np.zeros(6, dtype="float")
    rest_dthetas = np.zeros(6, dtype="float")
    t = 10.0
    dt = 0.005
    rest_length = 0.0

    def rpos(t_curr: float) -> np.ndarray:
        period = 10 / 2
        yval = np.cos(t_curr * (2 * np.pi / period))
        return np.array([1.0, yval, 1.0])

    # use aggressive damping like part 3b
    stiffness = 100.0
    damping = 4.0
    thetamat, dthetamat = puppet(
        home_thetas,
        rest_dthetas,
        g,
        ur5.Mlist,
        ur5.Slist,
        ur5.Glist,
        t,
        dt,
        damping,
        stiffness,
        rest_length,
        reference_pos=rpos,
        cfg=PuppetConfig.P4_MOVING_SPRING,
    )
    thetamat_to_csv(thetamat, "part4")


def main() -> None:
    part1()
    part2()
    part3()
    part4()


if __name__ == "__main__":
    sys.exit(main())
