"""
Convenience functions for printing stuff
"""

import numpy as np


def print_readable_and_answer(arr: np.ndarray, name: str = "", round: int = 2) -> None:
    if name:
        print(f"//////{name}: ")
    print("READABLE:")
    print(arr)
    print("COPYABLE:")
    rep = repr(arr.round(2))
    out = rep.removeprefix("array(").removesuffix(")")
    print(out)


def print_readable(arr: np.ndarray, name: str = "") -> None:
    if name:
        print(f"//////{name}: ")
    print(arr)
