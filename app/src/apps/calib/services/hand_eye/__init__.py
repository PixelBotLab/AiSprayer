# -*- coding: utf-8 -*-
"""
手眼标定求解内核 (两种安装共用)。

对外只暴露一个入口 `solve_hand_eye(mount, samples, ...)`, 由 mount 决定走哪套数学模型,
调用方不需要知道 EyeToHandSolution / EyeInHandSolution 的差异。
"""
from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from .eye_in_hand import EyeInHandSolution
from .eye_in_hand import solve as solve_eye_in_hand
from .eye_to_hand import EyeToHandSolution
from .eye_to_hand import solve as solve_eye_to_hand
from .geometry import (
    DOBOT_EULER_SEQ, EYE_IN_HAND, EYE_TO_HAND, INTERNAL_ANGLE_UNIT, MIN_SAMPLES,
    MOUNTS, RECOMMENDED_SAMPLES, UNIT_DEG, UNIT_RAD, chessboard_object_points,
    infer_angle_unit, invert_transform, make_transform, matrix_to_pose,
    normalize_pose, pose_to_matrix, rotation_from_pose,
)
from .samples import CalibSample, clean_samples, evaluate_data_quality

__all__ = [
    "CalibSample", "DOBOT_EULER_SEQ", "EYE_IN_HAND", "EYE_TO_HAND",
    "EyeInHandSolution", "EyeToHandSolution", "INTERNAL_ANGLE_UNIT",
    "MIN_SAMPLES", "MOUNTS", "RECOMMENDED_SAMPLES", "UNIT_DEG", "UNIT_RAD",
    "chessboard_object_points", "clean_samples", "evaluate_data_quality",
    "infer_angle_unit", "invert_transform", "make_transform", "matrix_to_pose",
    "normalize_pose", "pose_to_matrix", "rotation_from_pose", "solve_hand_eye",
]


def solve_hand_eye(mount: str, samples: Sequence[CalibSample],
                   K: Optional[np.ndarray] = None,
                   D: Optional[Sequence[float]] = None):
    """按安装方式求解手眼外参, 失败返回 None。"""
    if mount == EYE_TO_HAND:
        return solve_eye_to_hand(samples, K=K, D=D)
    if mount == EYE_IN_HAND:
        return solve_eye_in_hand(samples, K=K, D=D)
    raise ValueError(f"unknown hand-eye mount '{mount}', expected one of {MOUNTS}")


def minimum_samples(mount: str) -> int:
    """该安装方式可求解的绝对下界样本数。"""
    return MIN_SAMPLES.get(mount, 3)


def recommended_samples(mount: str) -> int:
    """该安装方式达到稳定精度的建议样本数。"""
    return RECOMMENDED_SAMPLES.get(mount, 12)
