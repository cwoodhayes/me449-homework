"""
Data & file io helpers for assignment 6

Author: Conor Hayes
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from pathlib import Path

import numpy as np
import pandas as pd
import toml

from modern_robotics import SimulateControl


@dataclass
class UR5Params:
    """
    UR5 6dof arm dynamics parameters.

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

    joint_damping_coeff: float = 0.5
    # joint torque limits in N*m
    torque_limits = 100.0


class TrajectoryType(Enum):
    """Trajectory type for motion planning."""

    SCREW_CUBIC = auto()
    SCREW_QUINTIC = auto()
    CARTESIAN_CUBIC = auto()
    CARTESIAN_QUINTIC = auto()


@dataclass
class UR5PlanningRequest:
    """
    Planning parameters for controlled motion of the UR5 arm.

    User input representation for a6.
    """

    traj_type: TrajectoryType
    """Type of trajectory the robot should follow."""

    duration_s: float
    """Duration of planned trajectory in seconds"""

    actual_init_config: np.ndarray
    """Robot actual initial configuration. 6-vector of joint angles."""

    Ts_start: np.ndarray
    """Start configuration of planned trajectory (EE pose) as an SE(3) matrix."""

    Ts_end: np.ndarray
    """End configuration of planned trajectory (EE pose) as an SE(3) matrix."""

    sim_dt: float = 0.01
    """timestep for simulation"""

    @classmethod
    def from_file(cls, p: Path) -> UR5PlanningRequest:
        """Load in this data from a config file."""
        with open(p, "r") as fp:
            config = toml.load(fp)

            return UR5PlanningRequest(
                Ts_start=np.array(config["Ts_start"]["SE3"], dtype=float),
                Ts_end=np.array(config["Ts_end"]["SE3"], dtype=float),
                traj_type=TrajectoryType[config["traj_type"]],
                duration_s=float(config["duration_s"]),
                actual_init_config=np.array(config["actual_init_config"], dtype=float),
            )


JOINT_COLS = [f"joint_{i}" for i in range(6)]


@dataclass
class UR5TrajectoryOutput:
    """Data outputted by planning + simulation."""

    joint_angles: pd.DataFrame = field(
        default_factory=lambda: pd.DataFrame(columns=["time_s", *JOINT_COLS])
    )
    joint_torques: pd.DataFrame = field(
        default_factory=lambda: pd.DataFrame(columns=["time_s", *JOINT_COLS])
    )
    errors: pd.DataFrame = field(
        default_factory=lambda: pd.DataFrame(
            columns=["time_s", "angular_err", "linear_err"]
        )
    )

    @classmethod
    def empty_from_config(cls, cfg: UR5PlanningRequest) -> UR5TrajectoryOutput:
        out = UR5TrajectoryOutput()

        ts = np.arange(0, cfg.duration_s, cfg.sim_dt)
        N = ts.shape[0]

        for df in [out.joint_angles, out.joint_torques, out.errors]:
            df["time_s"] = ts
            cols = df.columns[1:]
            df[cols] = np.zeros(shape=(N, len(cols)))

        return out

    @property
    def N(self) -> int:
        """Number of steps in the trajectory"""
        return self.joint_angles.shape[0]

    def export(self, dirpath: Path) -> None:
        """Export these to files in a directory"""
        dirpath.mkdir(exist_ok=True, parents=True)

        self.joint_angles.to_csv(dirpath / "joint_angles.csv", index=False, header=True)
        self.joint_torques.to_csv(
            dirpath / "joint_torques.csv", index=False, header=True
        )
        self.errors.to_csv(dirpath / "errors.csv", index=False, header=True)

        # generate coppeliasim-compatible output
        cop = self.joint_angles.copy()
        del cop["time_s"]
        cop.to_csv(dirpath / "coppellia.csv", index=False, header=False)
