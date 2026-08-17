"""
CR5 / UR-style analytical FK/IK core (DH frame, not URDF).

This is the math used by `CR5Kinematics(backend="python")`. The C++ twin lives in
`cr5_kinematics_cpp/cr5_ur_kin.cpp` and must stay numerically equivalent.

DH vs URDF
----------
These functions take/return DH joint angles. CR5 URDF q2/q4 differ by ±π/2:
    q_dh[1] = q_urdf[1] - π/2
    q_dh[3] = q_urdf[3] - π/2
`CR5Kinematics` applies that offset; do not pass URDF angles in here directly.

IK
--
`inverse` returns up to 8 DH solutions in [0, 2π): 2 q1 (shoulder) × 2 q5 (wrist)
× 2 q3 (elbow). Wrist-singular poses (sin(q5)≈0) use `q6_des` as the free q6 hint.
Unreachable poses (D4² > R, or |c3| > 1) return [].

`forward_all` is still a stub (identity frames) — Jacobian/manipulability is unused.
"""

import math
import numpy as np

PI = math.pi
ZERO_THRESH = 1e-8

# Modified DH (metres), same numbers as cr5_ur_kin.cpp / CR5 URDF.
D1 = 0.147   # base height
A2 = -0.427  # upper arm (negative DH convention)
A3 = -0.357  # forearm
D4 = 0.141   # shoulder offset (wrist cylinder radius about J1)
D5 = 0.116   # wrist 1
D6 = 0.105   # flange / wrist 2

def _sign(x: float) -> float:
    if x > 0: return 1.0
    if x < 0: return -1.0
    return 0.0

def forward(q: list[float] | np.ndarray) -> np.ndarray:
    q1, q2, q3, q4, q5, q6 = q[0], q[1], q[2], q[3], q[4], q[5]
    
    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s3, c3 = math.sin(q3), math.cos(q3)
    s4, c4 = math.sin(q4), math.cos(q4)
    s5, c5 = math.sin(q5), math.cos(q5)
    s6, c6 = math.sin(q6), math.cos(q6)
    
    q234 = q2 + q3 + q4
    s23, c23 = math.sin(q2 + q3), math.cos(q2 + q3)
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

def inverse(T: np.ndarray, q6_des: float = 0.0) -> list[list[float]]:
    # ur_kinematics indexes T with a column permutation (T02=-T[0,0], …).
    T_flat = T.flatten()
    T02 = -T_flat[0]; T00 =  T_flat[1]; T01 =  T_flat[2]; T03 = -T_flat[3]
    T12 = -T_flat[4]; T10 =  T_flat[5]; T11 =  T_flat[6]; T13 = -T_flat[7]
    T22 =  T_flat[8]; T20 = -T_flat[9]; T21 = -T_flat[10]; T23 =  T_flat[11]

    q_sols = []
    
    # q1: two shoulders from the wrist-centre projection onto the XY plane.
    # Discriminant R = A²+B²; unreachable if |d4| > sqrt(R) (inside J1 cylinder).
    q1 = [0.0, 0.0]
    A = D6 * T12 - T13
    B = D6 * T02 - T03
    R = A * A + B * B
    if abs(A) < ZERO_THRESH:
        div = -_sign(D4) * _sign(B) if abs(abs(D4) - abs(B)) < ZERO_THRESH else -D4 / B
        arcsin = math.asin(div)
        if abs(arcsin) < ZERO_THRESH: arcsin = 0.0
        q1[0] = arcsin + 2.0 * PI if arcsin < 0.0 else arcsin
        q1[1] = PI - arcsin
    elif abs(B) < ZERO_THRESH:
        div = _sign(D4) * _sign(A) if abs(abs(D4) - abs(A)) < ZERO_THRESH else D4 / A
        arccos = math.acos(div)
        q1[0] = arccos
        q1[1] = 2.0 * PI - arccos
    elif D4 * D4 > R:
        return []
    else:
        arccos = math.acos(D4 / math.sqrt(R))
        arctan = math.atan2(-B, A)
        pos = arccos + arctan
        neg = -arccos + arctan
        if abs(pos) < ZERO_THRESH: pos = 0.0
        if abs(neg) < ZERO_THRESH: neg = 0.0
        q1[0] = pos if pos >= 0.0 else 2.0 * PI + pos
        q1[1] = neg if neg >= 0.0 else 2.0 * PI + neg

    # q5: two wrist pitches per q1 (acos of the J4/J6 alignment residual).
    q5 = [[0.0, 0.0], [0.0, 0.0]]
    for i in range(2):
        numer = (T03 * math.sin(q1[i]) - T13 * math.cos(q1[i]) - D4)
        div = _sign(numer) * _sign(D6) if abs(abs(numer) - abs(D6)) < ZERO_THRESH else numer / D6
        arccos = math.acos(div)
        q5[i][0] = arccos
        q5[i][1] = 2.0 * PI - arccos

    for i in range(2):
        for j in range(2):
            c1, s1 = math.cos(q1[i]), math.sin(q1[i])
            c5, s5 = math.cos(q5[i][j]), math.sin(q5[i][j])
            
            # q6: from the wrist-2 rotation; free (q6_des) when sin(q5)≈0 (wrist singular).
            if abs(s5) < ZERO_THRESH:
                q6 = q6_des
            else:
                q6 = math.atan2(_sign(s5) * -(T01 * s1 - T11 * c1), 
                                _sign(s5) * (T00 * s1 - T10 * c1))
                if abs(q6) < ZERO_THRESH: q6 = 0.0
                if q6 < 0.0: q6 += 2.0 * PI

            # q2/q3/q4: planar 3R (elbow up/down). Skip if |c3|>1 (reach exceeded).
            c6, s6 = math.cos(q6), math.sin(q6)
            x04x = -s5 * (T02 * c1 + T12 * s1) - c5 * (s6 * (T01 * c1 + T11 * s1) - c6 * (T00 * c1 + T10 * s1))
            x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5
            p13x = D5 * (s6 * (T00 * c1 + T10 * s1) + c6 * (T01 * c1 + T11 * s1)) - D6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1
            p13y = T23 - D1 - D6 * T22 + D5 * (T21 * c6 + T20 * s6)

            c3 = (p13x * p13x + p13y * p13y - A2 * A2 - A3 * A3) / (2.0 * A2 * A3)
            if abs(abs(c3) - 1.0) < ZERO_THRESH:
                c3 = _sign(c3)
            elif abs(c3) > 1.0:
                continue

            arccos = math.acos(c3)
            q3 = [arccos, 2.0 * PI - arccos]
            denom = A2 * A2 + A3 * A3 + 2 * A2 * A3 * c3
            s3 = math.sin(arccos)
            A = (A2 + A3 * c3)
            B = A3 * s3
            
            q2 = [
                math.atan2((A * p13y - B * p13x) / denom, (A * p13x + B * p13y) / denom),
                math.atan2((A * p13y + B * p13x) / denom, (A * p13x - B * p13y) / denom)
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
                if abs(q2[k]) < ZERO_THRESH: q2[k] = 0.0
                elif q2[k] < 0.0: q2[k] += 2.0 * PI
                
                if abs(q4[k]) < ZERO_THRESH: q4[k] = 0.0
                elif q4[k] < 0.0: q4[k] += 2.0 * PI
                
                q_sols.append([q1[i], q2[k], q3[k], q4[k], q5[i][j], q6])

    return q_sols

def forward_all(q: list[float] | np.ndarray) -> list[np.ndarray]:
    return [np.eye(4) for _ in range(6)]
