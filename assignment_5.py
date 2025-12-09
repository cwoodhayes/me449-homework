from lib.printers import print_readable, print_readable_and_answer

import modern_robotics as mr

import sympy as sym
import numpy as np


def ex6_8():
    x, y = sym.symbols("x, y")
    g = sym.Matrix([x**2 - 4, y**2 - 9])
    theta = sym.Matrix([x, y])

    J = g.jacobian(theta)
    print(J)
    print(J.inv())

    # doing the rest by hand


def ex8_2():
    print("################## Q1")

    # find the center of mass
    rho = 7500
    r_sphere = 0.1
    r_cyl = 0.02
    l_cyl = 0.2
    sphere_vol_m3 = 4 / 3 * np.pi * (r_sphere**3)
    cylinder_vol_m3 = l_cyl * (np.pi * r_cyl**2)
    cylinder_mass = cylinder_vol_m3 * rho
    sphere_mass = sphere_vol_m3 * rho
    print_readable(cylinder_mass, "cylinder mass")
    print_readable(sphere_mass, "sphere mass")

    I_cyl = I_cylinder(0.02, 0.2, cylinder_mass)
    I_sph = I_sphere(0.1, sphere_mass)
    print_readable(I_sph, "I sphere (no translation)")

    I_leftsph = steiners_formula(I_sph, sphere_mass, q=np.array([0, 0, -0.2]))
    I_rightsph = steiners_formula(I_sph, sphere_mass, q=np.array([0, 0, 0.2]))
    print_readable(I_leftsph, "I left sphere")
    print_readable(I_leftsph, "I right sphere")
    print_readable(I_cyl, "I cylinder")

    I = I_leftsph + I_rightsph + I_cyl
    print_readable_and_answer(I, "I_total")

    m_total = 2 * sphere_mass + cylinder_mass
    print_readable_and_answer(m_total, "m_total")


def steiners_formula(Ib: np.ndarray, m: float, q: np.ndarray) -> np.ndarray:
    q = q.reshape(3, 1)  # ensure column vector
    return Ib + m * ((q.T @ q).item() * np.eye(3) - q @ q.T)


def I_sphere(r: float, m: float) -> np.ndarray:
    return np.eye(3) * (m * (2 * r**2) / 5)


def I_cylinder(r: float, h: float, m: float) -> np.ndarray:
    return np.array(
        [
            [m * (3 * r**2 + h**2) / 12, 0, 0],
            [0, m * (3 * r**2 + h**2) / 12, 0],
            [0, 0, m * r**2 / 2],
        ]
    )


def ex11_6():
    w_n = (1 / 40) ** 0.5
    zeta = (10**0.5) / 2

    s1 = -zeta * w_n + w_n * (zeta**2 - 1)
    print_readable_and_answer(s1, "s1")
    print_readable_and_answer(-1 / s1, "time constant")

    print("part c")
    K_d = 4 * 10**0.5 - 2
    print(K_d)

    print("part d")
    K_p = 4 * 0.04**2
    print(K_p)
    K_d = 2 * np.sqrt(4 * 0.0004) - 2
    print(K_d)


if __name__ == "__main__":
    # ex6_8()

    # ex8_2()
    ex11_6()
