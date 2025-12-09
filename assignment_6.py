"""
Motion planning & control of UR5 robot.

Final assignment for Prof. Lynch's class.

Author: Conor Hayes
"""

from pathlib import Path
from lib import a6data

CONFIG_DIR_PATH = Path(__file__).parent / "config"
UR5 = a6data.UR5Params()


def main() -> None:
    plan = a6data.UR5PlanningConfig.from_file(CONFIG_DIR_PATH / "a6_demo1.toml")
    print(plan)
    print(UR5)


if __name__ == "__main__":
    main()
