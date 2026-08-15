"""
CR5 / UR-style Analytical Forward and Inverse Kinematics Core.
Faithfully ported from cr5_ur_kin.cpp.
"""

import math
import numpy as np

ZERO_THRESH = 1e-8

def _sign(x: float | np.number) -> int:
    if x > 0:
        return 1
    elif x < 0:
        return -1
    return 0

# CR5 Robot DH Parameters (in meters)
D1 = 0.147
A2 = -0.427
A3 = -0.357
D4 = 0.141
D5 = 0.116
D6 = 0.105

PI = math.pi


def forward(q: list[float] | np.ndarray) -> np.ndarray:
    """
    Computes 4x4 Homogeneous Transformation Matrix for given DH joint angles.
    Matches ur_kinematics::forward in cr5_ur_kin.cpp.
    
    :param q: 6 DH joint angles in radians
    :return: 4x4 Transformation Matrix (np.ndarray float64)
    """
    q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
    
    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s3, c3 = math.sin(q3), math.cos(q3)
    s4, c4 = math.sin(q4), math.cos(q4)
    s5, c5 = math.sin(q5), math.cos(q5)
    s6, c6 = math.sin(q6), math.cos(q6)
    
    q23 = q2 + q3
    q234 = q2 + q3 + q4
    
    s23, c23 = math.sin(q23), math.cos(q23)
    s234, c234 = math.sin(q234), math.cos(q234)
    
    T = np.zeros((4, 4), dtype=np.float64)
    
    T[0, 0] = c234 * c1 * s5 - c5 * s1
    T[0, 1] = c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6
    T[0, 2] = -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6
    T[0, 3] = D6 * c234 * c1 * s5 - A3 * c23 * c1 - A2 * c1 * c2 - D6 * c5 * s1 - D5 * s234 * c1 - D4 * s1

    T[1, 0] = c1 * c5 + c234 * s1 * s5
    T[1, 1] = -c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6
    T[1, 2] = s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1
    T[1, 3] = D6 * (c1 * c5 + c234 * s1 * s5) + D4 * c1 - A3 * c23 * s1 - A2 * c2 * s1 - D5 * s234 * s1

    T[2, 0] = -s234 * s5
    T[2, 1] = -c234 * s6 - s234 * c5 * c6
    T[2, 2] = s234 * c5 * s6 - c234 * c6
    T[2, 3] = D1 + A3 * s23 + A2 * s2 - D5 * (c23 * c4 - s23 * s4) - D6 * s5 * (c23 * s4 + s23 * c4)

    T[3, 0] = 0.0
    T[3, 1] = 0.0
    T[3, 2] = 0.0
    T[3, 3] = 1.0
    
    return T


def forward_all(q: list[float] | np.ndarray) -> list[np.ndarray]:
    """
    Computes all intermediate transformation matrices T1 to T6.
    Matches ur_kinematics::forward_all in cr5_ur_kin.cpp.
    
    :param q: 6 DH joint angles in radians
    :return: list of 6 4x4 Transformation Matrices [T1, T2, T3, T4, T5, T6]
    """
    q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
    
    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s3, c3 = math.sin(q3), math.cos(q3)
    s4, c4 = math.sin(q4), math.cos(q4)
    s5, c5 = math.sin(q5), math.cos(q5)
    s6, c6 = math.sin(q6), math.cos(q6)
    
    q23 = q2 + q3
    q234 = q2 + q3 + q4
    
    s23, c23 = math.sin(q23), math.cos(q23)
    s234, c234 = math.sin(q234), math.cos(q234)

    T1 = np.array([
        [c1, 0, s1, 0],
        [s1, 0, -c1, 0],
        [0, 1, 0, D1],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    T2 = np.array([
        [c1 * c2, -c1 * s2, s1, A2 * c1 * c2],
        [c2 * s1, -s1 * s2, -c1, A2 * c2 * s1],
        [s2, c2, 0, D1 + A2 * s2],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    T3 = np.array([
        [c23 * c1, -s23 * c1, s1, c1 * (A3 * c23 + A2 * c2)],
        [c23 * s1, -s23 * s1, -c1, s1 * (A3 * c23 + A2 * c2)],
        [s23, c23, 0, D1 + A3 * s23 + A2 * s2],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    T4 = np.array([
        [c234 * c1, s1, s234 * c1, c1 * (A3 * c23 + A2 * c2) + D4 * s1],
        [c234 * s1, -c1, s234 * s1, s1 * (A3 * c23 + A2 * c2) - D4 * c1],
        [s234, 0, -c234, D1 + A3 * s23 + A2 * s2],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    T5 = np.array([
        [s1 * s5 + c234 * c1 * c5, -s234 * c1, c5 * s1 - c234 * c1 * s5, c1 * (A3 * c23 + A2 * c2) + D4 * s1 + D5 * s234 * c1],
        [c234 * c5 * s1 - c1 * s5, -s234 * s1, -c1 * c5 - c234 * s1 * s5, s1 * (A3 * c23 + A2 * c2) - D4 * c1 + D5 * s234 * s1],
        [s234 * c5, c234, -s234 * s5, D1 + A3 * s23 + A2 * s2 - D5 * c234],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    T6 = np.array([
        [c6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * s6, -s6 * (s1 * s5 + c234 * c1 * c5) - s234 * c1 * c6, c5 * s1 - c234 * c1 * s5, D6 * (c5 * s1 - c234 * c1 * s5) + c1 * (A3 * c23 + A2 * c2) + D4 * s1 + D5 * s234 * c1],
        [-c6 * (c1 * s5 - c234 * c5 * s1) - s234 * s1 * s6, s6 * (c1 * s5 - c234 * c5 * s1) - s234 * c6 * s1, -c1 * c5 - c234 * s1 * s5, s1 * (A3 * c23 + A2 * c2) - D4 * c1 - D6 * (c1 * c5 + c234 * s1 * s5) + D5 * s234 * s1],
        [c234 * s6 + s234 * c5 * c6, c234 * c6 - s234 * c5 * s6, -s234 * s5, D1 + A3 * s23 + A2 * s2 - D5 * c234 - D6 * s234 * s5],
        [0, 0, 0, 1]
    ], dtype=np.float64)

    return [T1, T2, T3, T4, T5, T6]


def inverse(T: np.ndarray, q6_des: float = 0.0) -> list[list[float]]:
    """
    Computes all analytical inverse kinematics solutions (up to 8 solutions).
    Matches ur_kinematics::inverse in cr5_ur_kin.cpp.
    
    :param T: 4x4 Transformation Matrix
    :param q6_des: desired q6 angle when wrist is singular (q5 ~ 0)
    :return: list of joint angle solutions, each solution is a list of 6 floats in radians
    """
    sols = []
    
    T_flat = T.flatten()
    T02 = -T_flat[0]
    T00 =  T_flat[1]
    T01 =  T_flat[2]
    T03 = -T_flat[3]
    
    T12 = -T_flat[4]
    T10 =  T_flat[5]
    T11 =  T_flat[6]
    T13 = -T_flat[7]
    
    T22 =  T_flat[8]
    T20 = -T_flat[9]
    T21 = -T_flat[10]
    T23 =  T_flat[11]

    # --- 1. Shoulder Rotate Joint (q1) ---
    q1 = [0.0, 0.0]
    A = D6 * T12 - T13
    B = D6 * T02 - T03
    R = A * A + B * B
    
    if abs(A) < ZERO_THRESH:
        if abs(abs(D4) - abs(B)) < ZERO_THRESH:
            div = -_sign(D4) * _sign(B)
        else:
            div = -D4 / B
        if abs(div) > 1.0:
            return sols
        arcsin = math.asin(div)
        if abs(arcsin) < ZERO_THRESH:
            arcsin = 0.0
        q1[0] = arcsin + 2.0 * PI if arcsin < 0.0 else arcsin
        q1[1] = PI - arcsin
    elif abs(B) < ZERO_THRESH:
        if abs(abs(D4) - abs(A)) < ZERO_THRESH:
            div = _sign(D4) * _sign(A)
        else:
            div = D4 / A
        if abs(div) > 1.0:
            return sols
        arccos = math.acos(div)
        q1[0] = arccos
        q1[1] = 2.0 * PI - arccos
    elif D4 * D4 > R:
        return sols
    else:
        div = D4 / math.sqrt(R)
        div = max(-1.0, min(1.0, div))
        arccos = math.acos(div)
        arctan = math.atan2(-B, A)
        pos = arccos + arctan
        neg = -arccos + arctan
        if abs(pos) < ZERO_THRESH:
            pos = 0.0
        if abs(neg) < ZERO_THRESH:
            neg = 0.0
        q1[0] = pos if pos >= 0.0 else 2.0 * PI + pos
        q1[1] = neg if neg >= 0.0 else 2.0 * PI + neg

    # --- 2. Wrist 2 Joint (q5) ---
    q5 = [[0.0, 0.0], [0.0, 0.0]]
    for i in range(2):
        numer = (T03 * math.sin(q1[i]) - T13 * math.cos(q1[i]) - D4)
        if abs(abs(numer) - abs(D6)) < ZERO_THRESH:
            div = _sign(numer) * _sign(D6)
        else:
            div = numer / D6
        div = max(-1.0, min(1.0, div))
        arccos = math.acos(div)
        q5[i][0] = arccos
        q5[i][1] = 2.0 * PI - arccos

    # --- 3. Wrist 3 Joint (q6) & RRR Joints (q2, q3, q4) ---
    for i in range(2):
        for j in range(2):
            c1 = math.cos(q1[i])
            s1 = math.sin(q1[i])
            c5 = math.cos(q5[i][j])
            s5 = math.sin(q5[i][j])
            
            # Wrist 3 (q6)
            if abs(s5) < ZERO_THRESH:
                q6 = q6_des
            else:
                q6 = math.atan2(_sign(s5) * -(T01 * s1 - T11 * c1),
                                _sign(s5) *  (T00 * s1 - T10 * c1))
                if abs(q6) < ZERO_THRESH:
                    q6 = 0.0
                if q6 < 0.0:
                    q6 += 2.0 * PI

            c6 = math.cos(q6)
            s6 = math.sin(q6)
            x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1))
            x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5
            p13x = D5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - D6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1
            p13y = T23 - D1 - D6 * T22 + D5 * (T21 * c6 + T20 * s6)

            c3 = (p13x * p13x + p13y * p13y - A2 * A2 - A3 * A3) / (2.0 * A2 * A3)
            if abs(abs(c3) - 1.0) < ZERO_THRESH:
                c3 = float(_sign(c3))
            elif abs(c3) > 1.0:
                continue

            arccos3 = math.acos(max(-1.0, min(1.0, c3)))
            q3 = [arccos3, 2.0 * PI - arccos3]
            denom = A2 * A2 + A3 * A3 + 2.0 * A2 * A3 * c3
            s3 = math.sin(arccos3)
            A_term = (A2 + A3 * c3)
            B_term = A3 * s3
            
            q2 = [
                math.atan2((A_term * p13y - B_term * p13x) / denom, (A_term * p13x + B_term * p13y) / denom),
                math.atan2((A_term * p13y + B_term * p13x) / denom, (A_term * p13x - B_term * p13y) / denom)
            ]
            
            c23_0 = math.cos(q2[0] + q3[0])
            s23_0 = math.sin(q2[0] + q3[0])
            c23_1 = math.cos(q2[1] + q3[1])
            s23_1 = math.sin(q2[1] + q3[1])
            
            q4 = [
                math.atan2(c23_0 * x04y - s23_0 * x04x, x04x * c23_0 + x04y * s23_0),
                math.atan2(c23_1 * x04y - s23_1 * x04x, x04x * c23_1 + x04y * s23_1)
            ]

            for k in range(2):
                q2_k = q2[k]
                q4_k = q4[k]
                if abs(q2_k) < ZERO_THRESH:
                    q2_k = 0.0
                elif q2_k < 0.0:
                    q2_k += 2.0 * PI
                if abs(q4_k) < ZERO_THRESH:
                    q4_k = 0.0
                elif q4_k < 0.0:
                    q4_k += 2.0 * PI
                
                sols.append([q1[i], q2_k, q3[k], q4_k, q5[i][j], q6])
                
    return sols
