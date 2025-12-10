"""
Motion planning & control of UR5 robot.

Final assignment for Prof. Lynch's class.

Author: Conor Hayes
"""

from pathlib import Path

from matplotlib.axes import Axes
from matplotlib.figure import Figure
from lib import a6data

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import modern_robotics as mr

CONFIG_DIR_PATH = Path(__file__).parent / "config"
OUTPUT_DIR_PATH = Path(__file__).parent / "a6-output"
UR5 = a6data.UR5Params()


def main() -> None:
    req = a6data.UR5PlanningRequest.from_file(CONFIG_DIR_PATH / "a6_demo1.toml")

    traj = plan_and_simulate(req)

    out_dir = OUTPUT_DIR_PATH / req.traj_type.name
    plot_output(req, traj, out_dir)
    traj.export(out_dir)

    plt.show()


def plan_and_simulate(req: a6data.UR5PlanningRequest) -> a6data.UR5TrajectoryOutput:
    """Plan a trajectory and simulate it under control."""
    out = a6data.UR5TrajectoryOutput.empty_from_config(req)
    eomg = 0.01
    ev = 0.01
    N = out.N
    dt = req.sim_dt
    g = np.array([0, 0, -9.81])

    # start from rest
    theta_start, success = mr.IKinSpace(
        UR5.Slist, UR5.M, req.Ts_start, np.zeros(6), eomg, ev
    )
    if not success:
        raise RuntimeError("Failed IKinSpace!")
    dtheta_start = np.zeros(6)

    # generate a EE trajectory (Tlist) according to the request.
    print("Generating timescaled EE trajectory...")
    method = (
        3
        if (a6data.TrajectoryType.SCREW_CUBIC or a6data.TrajectoryType.CARTESIAN_CUBIC)
        else 5
    )
    if a6data.TrajectoryType.SCREW_CUBIC or a6data.TrajectoryType.SCREW_QUINTIC:
        Tlist = mr.ScrewTrajectory(req.Ts_start, req.Ts_end, req.duration_s, N, method)
    else:
        Tlist = mr.CartesianTrajectory(
            req.Ts_start, req.Ts_end, req.duration_s, N, method
        )

    # generate a joint space trajectories, velocities, and accelerations
    print("Generating desired joint trajectory...")
    thetamat = np.empty(shape=(N, 6))
    dthetamat = np.empty(shape=(N, 6))
    ddthetamat = np.empty(shape=(N, 6))
    thetamat[0, :] = theta_start
    for i in range(N):
        thetamat[i], success = mr.IKinSpace(
            UR5.Slist, UR5.M, Tlist[i], thetamat[i - 1], eomg, ev
        )
        if not success:
            print(f"Failed IKinSpace at index {i}.")
            thetamat[i] = [np.nan] * 6

    # calculate joint velocities and accelerations for the trajectory
    for i in range(N - 1):
        dthetamat[i + 1, :] = (thetamat[i + 1, :] - thetamat[i, :]) / dt
        ddthetamat[i + 1, :] = (dthetamat[i + 1, :] - dthetamat[i, :]) / dt

    # use actual given values for G and M tilde
    # then apply no force to the EE
    Ftipmat = np.zeros((N, 6))

    # tuning params
    Kp = 20
    Ki = 10
    Kd = 18
    intRes = 8

    print("Simulating with computed torque control...")
    taumat, thetamat = simulate_control(
        req.actual_init_config,
        dtheta_start,
        g,
        Ftipmat,
        UR5.Mlist,
        UR5.Glist,
        UR5.Slist,
        thetamat,
        dthetamat,
        ddthetamat,
        g,
        UR5.Mlist,
        UR5.Glist,
        Kp,
        Ki,
        Kd,
        dt,
        intRes,
        UR5.joint_damping_coeff,
        UR5.torque_limits,
    )
    jcols = out.joint_angles.columns[1:]
    out.joint_angles[jcols] = thetamat
    out.joint_torques[jcols] = taumat

    # calculate error in EE pose over time as normed linear & angular error
    Tlist = np.array(Tlist)
    Tlist_actual = np.empty_like(Tlist)
    for i in range(N):
        Tlist_actual[i] = mr.FKinSpace(UR5.M, UR5.Slist, thetamat[i])

        out.errors.loc[i, "angular_err"] = angle_err(
            Tlist[i, :3, :3], Tlist_actual[i, :3, :3]
        )

    ee_position_errs = np.squeeze(Tlist_actual[:, 0:3, 3] - Tlist[:, 0:3, 3])
    out.errors["linear_err"] = np.linalg.norm(ee_position_errs, axis=1)

    print("Done.")
    return out


def angle_err(R1: np.ndarray, R2: np.ndarray) -> float:
    """
    returns the angle along the geodesic between R1 and R2

    Args:
        R1 (np.ndarray): SO rotation matrix
        R2 (np.ndarray): SO rotation matrix

    Returns:
        float: angle between R1 and R2 in radians
    """

    # R1 @ R = R2 -- R is rotation between them.
    R = R1.T @ R2

    # use Rodrigues's formula to get the angle-axis
    # rep for this matrix, and get the angle theta
    # cite: https://www.egr.msu.edu/~vaibhav/teaching/robotics/lectures/lec13.pdf?utm_source=chatgpt.com
    theta = np.arccos((np.trace(R) - 1) / 2)
    return theta


def simulate_control(
    thetalist,
    dthetalist,
    g,
    Ftipmat,
    Mlist,
    Glist,
    Slist,
    thetamatd,
    dthetamatd,
    ddthetamatd,
    gtilde,
    Mtildelist,
    Gtildelist,
    Kp,
    Ki,
    Kd,
    dt,
    intRes,
    damping_coeff: float,
    torque_limits: float | np.ndarray,
):
    """lightly edited version of Modern Robotic's SimulateControl function."""
    Ftipmat = np.array(Ftipmat).T
    thetamatd = np.array(thetamatd).T
    dthetamatd = np.array(dthetamatd).T
    ddthetamatd = np.array(ddthetamatd).T
    m, n = np.array(thetamatd).shape
    thetacurrent = np.array(thetalist).copy()
    dthetacurrent = np.array(dthetalist).copy()
    eint = np.zeros((m, 1)).reshape(
        m,
    )
    taumat = np.zeros(np.array(thetamatd).shape)
    thetamat = np.zeros(np.array(thetamatd).shape)
    for i in range(n):
        taulist = mr.ComputedTorque(
            thetacurrent,
            dthetacurrent,
            eint,
            gtilde,
            Mtildelist,
            Gtildelist,
            Slist,
            thetamatd[:, i],
            dthetamatd[:, i],
            ddthetamatd[:, i],
            Kp,
            Ki,
            Kd,
        )
        # apply damping to each joint
        taulist -= dthetamatd[:, 1] * damping_coeff
        # apply torque limits
        taulist = np.clip(taulist, a_min=-torque_limits, a_max=torque_limits)
        for j in range(intRes):
            ddthetalist = mr.ForwardDynamics(
                thetacurrent,
                dthetacurrent,
                taulist,
                g,
                Ftipmat[:, i],
                Mlist,
                Glist,
                Slist,
            )
            thetacurrent, dthetacurrent = mr.EulerStep(
                thetacurrent, dthetacurrent, ddthetalist, 1.0 * dt / intRes
            )
        taumat[:, i] = taulist
        thetamat[:, i] = thetacurrent
        eint = np.add(eint, dt * np.subtract(thetamatd[:, i], thetacurrent))

    # Output using matplotlib to plot
    links = np.array(thetamat).shape[0]
    N = np.array(thetamat).shape[1]
    Tf = N * dt
    timestamp = np.linspace(0, Tf, N)
    fig = plt.figure("a6_actual_vs_desired")
    ax = fig.add_subplot()
    for i in range(links):
        col = [
            np.random.uniform(0, 1),
            np.random.uniform(0, 1),
            np.random.uniform(0, 1),
        ]
        ax.plot(
            timestamp,
            thetamat[i, :],
            "-",
            color=col,
            label=("ActualTheta" + str(i + 1)),
        )
        ax.plot(
            timestamp,
            thetamatd[i, :],
            ".",
            color=col,
            label=("DesiredTheta" + str(i + 1)),
        )
    ax.legend(loc="upper left")
    ax.set_xlabel("Time")
    ax.set_ylabel("Joint Angles")
    ax.set_title("Plot of Actual and Desired Joint Angles")
    taumat = np.array(taumat).T
    thetamat = np.array(thetamat).T
    return (taumat, thetamat)


def plot_output(
    cfg: a6data.UR5PlanningRequest,
    traj: a6data.UR5TrajectoryOutput,
    output_dir: Path,
) -> None:
    """Make plots of the output data & write them to output_dir."""
    output_dir.mkdir(exist_ok=True, parents=True)

    fig = plt.figure("A6_output", figsize=(6, 10))
    axes = fig.subplots(3, 1)
    plot_df(traj.joint_angles, "Joint Angles (rad)", axes[0])
    plot_df(traj.joint_torques, "Joint Torques (N*m)", axes[1])
    plot_df(traj.errors, "error", axes[2])

    fig.tight_layout()

    print("Saving figures...")
    for num in plt.get_fignums():
        fig = plt.figure(num)
        name = fig.get_label() or f"figure_{num}"
        fig.savefig(str(output_dir / f"{name}.png"))


def plot_df(df: pd.DataFrame, ylabel: str, ax: Axes) -> None:
    """Plot dataframe in the format i'm currently using."""
    # plot time vs all othe columns
    for col in df.columns:
        if col != "time_s":
            ax.plot(df["time_s"], df[col], label=col)

    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    ax.legend()


if __name__ == "__main__":
    main()
