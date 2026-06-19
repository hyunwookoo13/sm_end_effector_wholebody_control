#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
import math

import numpy as np


@dataclass
class EmaFilterState:
    """grasp 안정화 필터 상태."""

    position: np.ndarray | None = None
    quaternion_xyzw: np.ndarray | None = None


def normalize_quaternion_xyzw(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm <= 1e-9:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def clamp_translation_step(prev: np.ndarray, cur: np.ndarray, max_step_m: float) -> np.ndarray:
    delta = cur - prev
    dist = np.linalg.norm(delta)
    if dist <= max_step_m or dist <= 1e-9:
        return cur
    return prev + (delta / dist) * max_step_m


def slerp_xyzw(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = normalize_quaternion_xyzw(q0)
    q1 = normalize_quaternion_xyzw(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        return normalize_quaternion_xyzw(q0 + alpha * (q1 - q0))

    theta_0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return normalize_quaternion_xyzw((s0 * q0) + (s1 * q1))


def apply_ema(
    state: EmaFilterState,
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
    position_alpha: float,
    orientation_alpha: float,
) -> tuple[np.ndarray, np.ndarray]:
    if state.position is None or state.quaternion_xyzw is None:
        state.position = position.copy()
        state.quaternion_xyzw = normalize_quaternion_xyzw(quaternion_xyzw)
        return state.position, state.quaternion_xyzw

    state.position = (1.0 - position_alpha) * state.position + position_alpha * position
    state.quaternion_xyzw = slerp_xyzw(state.quaternion_xyzw, quaternion_xyzw, orientation_alpha)
    return state.position.copy(), state.quaternion_xyzw.copy()

