from modern_robotics import InverseDynamics

import numpy as np

from lib.printers import print_readable_and_answer, print_readable


def q1():
    print("################## Q1")

    # find the center of mass
    rho = 5600
    sphere_vol_m3 = 4 / 3 * np.pi * (0.1**3)
    cylinder_vol_m3 = 0.2 * (np.pi * 0.02**2)
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


def q5() -> None:
    print("############### Q5")
    M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
    M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
    M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
    M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
    M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
    M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
    M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
    G1 = np.diag([0.010267495893, 0.010267495893, 0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
    Glist = [G1, G2, G3, G4, G5, G6]
    Mlist = [M01, M12, M23, M34, M45, M56, M67]
    Slist = [
        [0, 0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0, 1],
        [1, 0, 0, 0, -1, 0],
        [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
        [0, 0, 0, 0, 0.81725, 0],
        [0, 0, 0.425, 0.81725, 0, 0.81725],
    ]

    theta = np.array([0, np.pi / 6, np.pi / 4, np.pi / 3, np.pi / 2, 2 * np.pi / 3])
    thetadot = np.array(6 * [0.2])
    thetaddot = np.array(6 * [0.1])
    g = np.array([0, 0, -9.81])
    F_tip = np.array(6 * [0.1])

    out = InverseDynamics(theta, thetadot, thetaddot, g, F_tip, Mlist, Glist, Slist)
    print_readable_and_answer(out, "Joint forces and torques")


if __name__ == "__main__":
    q1()

    # q5()
