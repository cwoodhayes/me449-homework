from modern_robotics import (
    FKinBody,
    FKinSpace,
    TransInv,
    Adjoint,
    MatrixLog6,
    se3ToVec,
    MatrixExp6,
    VecTose3,
)
import numpy as np

from lib.printers import print_readable


def ex_4_2():
    print("##################4.2")
    M = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, 2],
            [0, 0, 1, 1],
            [0, 0, 0, 1],
        ]
    )
    S_space = np.array(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1, 1, 1, 0],
            [0, 1, 2, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    S_body = np.array(
        [
            [0, 0, 0, 0],
            [0, 0, 0, 0],
            [1, 1, 1, 0],
            [-2, -1, 0, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    theta = np.array([0, np.pi / 2, -np.pi / 2, 1])

    T_space = FKinSpace(M, S_space, theta)
    T_body = FKinBody(M, S_body, theta)
    print_readable(T_space, "T_space")
    print_readable(T_body, "T_body")
    pass


def ex_5_2():
    print("##################5.2 A")
    r = np.array([1 + np.cos(np.pi / 4), np.sin(np.pi / 4), 0])
    f = np.array([5, 0, 0])
    p = np.cross(r, f)
    print_readable(p, "p")

    F_s = np.concat([p, [5, 0, 0]])

    J_s = np.array(
        [
            [0, 0, 0],
            [0, 0, 0],
            [1, 1, 1],
            [0, 0, np.sin(np.pi / 4)],
            [0, -1, -(1 + np.cos(np.pi / 4))],
            [0, 0, 0],
        ]
    )

    F_s_col = F_s.reshape(-1, 1)

    tau = J_s.T @ F_s_col

    print_readable(tau, "tau_a")

    print("##################5.2 5")
    r = np.array([1 + np.cos(np.pi / 4), np.sin(np.pi / 4), 0])
    f = np.array([0, 5, 0])
    p = np.cross(r, f)
    print_readable(p, "p")

    F_s = np.concat([p, [0, 5, 0]])
    F_s_col = F_s.reshape(-1, 1)
    tau = J_s.T @ F_s_col

    print_readable(tau, "tau_b")


def ex_3_16():
    print("################# 3.16 C")
    T_sb = np.array(
        [
            [1, 0, 0, 0],
            [0, 0, 1, 2],
            [0, -1, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    T_bs = TransInv(T_sb)
    print_readable(T_bs, "T_sb inverse")

    print("################# 3.16 G")
    p_s = np.array([1, 2, 3, 1])
    p_primeprime = T_bs @ p_s
    print_readable(p_primeprime, "p''")

    print("################# 3.16 H")
    T_sa = np.array(
        [
            [1, -1, 0, 3],
            [0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1],
        ]
    )
    T_as = TransInv(T_sa)

    Ad_Tas = Adjoint(T_as)
    V_s = np.array([3, 2, 1, -1, 2, 3])
    V_a = Ad_Tas @ V_s
    print_readable(V_a, "V_a")

    print("################# 3.16 I")
    Sbracket_theta = MatrixLog6(T_sa)
    print_readable(Sbracket_theta, "[S]th")
    twist_sa = se3ToVec(Sbracket_theta)
    print_readable(twist_sa, "Vector form of [S]th")

    # normalize the twist. Do this by normalizing the w component
    screw_sa = twist_sa / np.linalg.norm(twist_sa[:3,])
    print_readable(screw_sa, "Screw S")


def ex_3_16_J():
    print("################# 3.16 J")
    Stheta = np.array([0, 1, 2, 3, 0, 0])
    Stheta_bracket = VecTose3(Stheta)
    T = MatrixExp6(Stheta_bracket)
    print_readable(T, "T")


if __name__ == "__main__":
    # ex_4_2()
    # ex_5_2()
    ex_3_16()
    ex_3_16_J()
