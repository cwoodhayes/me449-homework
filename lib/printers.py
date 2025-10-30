"""
Convenience functions for printing stuff
"""

import typing
import numpy as np


def print_readable_and_answer(
    arr: np.ndarray, name: str = "", ndigits: int = 2, use_round: bool = True
) -> None:
    if name:
        print(f"//////{name}: ")
    print("READABLE:")
    print(np.round(arr, ndigits) if use_round else sigfig(arr, ndigits))
    print("COPYABLE:")
    rep = repr(arr)
    out = rep.removeprefix("array(").removesuffix(")")
    print(out)


def print_readable(
    arr: typing.Any, name: str = "", ndigits: int = 3, use_round: bool = True
) -> None:
    if name:
        print(f"//////{name}: ")
    if ndigits is not None:
        arr = np.round(arr, ndigits) if use_round else sigfig(arr, ndigits)
    print(arr)


def sigfig(x, p):
    x = np.asarray(x)
    x_positive = np.where(np.isfinite(x) & (x != 0), np.abs(x), 10 ** (p - 1))
    mags = 10 ** (p - 1 - np.floor(np.log10(x_positive)))
    return np.round(x * mags) / mags
