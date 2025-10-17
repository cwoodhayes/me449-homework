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
    print(repr(arr.round(2)))
    print()


def print_readable(arr: np.ndarray, name: str = "") -> None:
    if name:
        print(f"//////{name}: ")
    print(arr)
