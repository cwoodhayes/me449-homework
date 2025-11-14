"""ME449 assignment 4."""

import sys
from dataclasses import dataclass
from pathlib import Path
import signal

from modern_robotics import se3ToVec, MatrixLog6, TransInv, FKinBody, JacobianBody
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.axes import Axes

from lib.printers import print_readable


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
    stiffness: str,
    restLength: float,
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
    pass


def main() -> None:
    pass


if __name__ == "__main__":
    sys.exit(main())
