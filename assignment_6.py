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

CONFIG_DIR_PATH = Path(__file__).parent / "config"
OUTPUT_DIR_PATH = Path(__file__).parent / "a6-output"
UR5 = a6data.UR5Params()


def main() -> None:
    cfg = a6data.UR5PlanningConfig.from_file(CONFIG_DIR_PATH / "a6_demo1.toml")

    traj = plan_and_simulate(cfg)

    out_dir = OUTPUT_DIR_PATH
    plot_output(cfg, traj, out_dir)
    traj.export(out_dir)


def plan_and_simulate(cfg: a6data.UR5PlanningConfig) -> a6data.UR5TrajectoryOutput:
    """Plan a trajectory and simulate it under control."""
    traj = a6data.UR5TrajectoryOutput.empty_from_config(cfg)

    # TODO

    return traj


def plot_output(
    cfg: a6data.UR5PlanningConfig,
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
