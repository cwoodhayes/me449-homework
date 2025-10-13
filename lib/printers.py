"""
Convenience functions for printing stuff
"""

import numpy as np


def print_readable_and_answer(arr: np.ndarray, name: str = "") -> None:
    if name:
        print(f"//////{name}: ")
    print("READABLE:")
    print(arr)
    print("COPYABLE:")
    print(repr(arr))
    print()
