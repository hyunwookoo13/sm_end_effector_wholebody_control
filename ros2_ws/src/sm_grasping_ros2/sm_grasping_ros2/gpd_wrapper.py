#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class GraspCandidate:
    """단일 grasp 후보 표현."""

    translation: np.ndarray  # shape=(3,)
    quaternion_xyzw: np.ndarray  # shape=(4,)
    score: float
    opening_width_m: float


class GpdWrapper:
    """GPD 백엔드 래퍼.

    환경마다 GPD 연동 방식(라이브러리 import/외부 노드 브리지)이 달라
    기본값은 안전한 휴리스틱 후보 생성으로 동작한다.
    """

    def __init__(
        self,
        logger,
        default_opening_m: float = 0.04,
        lateral_backoff_m: float = 0.0,
    ):
        self.logger = logger
        self.default_opening_m = float(default_opening_m)
        self.lateral_backoff_m = float(lateral_backoff_m)
        self._load_error: Optional[str] = None
        self._warned = False
        self._last_yaw: Optional[float] = None

    @property
    def is_ready(self) -> bool:
        return True

    @property
    def load_error(self) -> Optional[str]:
        return self._load_error

    def reset_tracking_state(self) -> None:
        """Forget orientation continuity when the requested object changes."""
        self._last_yaw = None

    def infer(self, points_xyz: np.ndarray, max_candidates: int, lims: list[float]) -> list[GraspCandidate]:
        if points_xyz.size == 0:
            return []

        if not self._warned:
            self.logger.warn("GPD 미사용: 캔/원통형 물체용 휴리스틱 grasp를 생성합니다.")
            self._warned = True

        points = points_xyz.astype(float)

        # 1. 중심점
        centroid = np.mean(points, axis=0)

        # 2. XY 평면 PCA로 물체 장축 추정
        xy = points[:, :2]
        xy_centered = xy - np.mean(xy, axis=0, keepdims=True)

        if points.shape[0] >= 5:
            cov = np.cov(xy_centered, rowvar=False)
            eigvals, eigvecs = np.linalg.eigh(cov)
            major_axis = eigvecs[:, int(np.argmax(eigvals))]
        else:
            major_axis = np.array([1.0, 0.0], dtype=float)

        major_axis = major_axis / (np.linalg.norm(major_axis) + 1e-9)

        # 3. 캔 장축에 수직인 방향
        closing_axis = np.array([-major_axis[1], major_axis[0]], dtype=float)
        closing_axis = closing_axis / (np.linalg.norm(closing_axis) + 1e-9)

        # 4. 로봇 base 쪽에서 접근하도록 방향 선택
        # target_frame이 base_link/chassis_link 기준이라고 가정하면,
        # centroid xy 벡터는 로봇에서 물체로 향하는 방향
        robot_to_obj = centroid[:2]
        if np.linalg.norm(robot_to_obj) > 1e-6:
            robot_to_obj = robot_to_obj / np.linalg.norm(robot_to_obj)

            # closing_axis가 로봇에서 물체 쪽으로 너무 반대면 뒤집기
            if np.dot(closing_axis, robot_to_obj) < 0.0:
                closing_axis = -closing_axis

        # 5. grasp yaw
        # gripper local X축이 접근 방향이라고 보고 yaw를 잡음
        yaw = float(np.arctan2(closing_axis[1], closing_axis[0]))
        yaw = self._stabilize_yaw(yaw)
        q = self._yaw_to_quat_xyzw(yaw)

        # 6. 선택적 횡방향 보정. Top-down grasp는 기본적으로 ROI 중심을 사용한다.
        target = centroid.copy()
        target[0] -= closing_axis[0] * self.lateral_backoff_m
        target[1] -= closing_axis[1] * self.lateral_backoff_m

        # 너무 낮게 찍히면 테이블/물체 표면에 박을 수 있어서 살짝 위로
        target[2] += 0.015

        score = 0.7

        return [
            GraspCandidate(
                translation=np.array([target[0], target[1], target[2]], dtype=float),
                quaternion_xyzw=q,
                score=score,
                opening_width_m=self.default_opening_m,
            )
        ][: max(1, max_candidates)]

    @staticmethod
    def _estimate_yaw_from_points(points_xyz: np.ndarray) -> float:
        """XY 평면 PCA로 장축 방향(yaw)을 추정한다."""
        if points_xyz.shape[0] < 5:
            return 0.0
        xy = points_xyz[:, :2].astype(float)
        mean = np.mean(xy, axis=0, keepdims=True)
        centered = xy - mean
        cov = np.cov(centered, rowvar=False)
        eigvals, eigvecs = np.linalg.eigh(cov)
        major_vec = eigvecs[:, int(np.argmax(eigvals))]
        yaw = float(np.arctan2(major_vec[1], major_vec[0]))
        return yaw

    @staticmethod
    def _yaw_to_quat_xyzw(yaw: float) -> np.ndarray:
        half = 0.5 * float(yaw)
        return np.array([0.0, 0.0, np.sin(half), np.cos(half)], dtype=float)

    def _stabilize_yaw(self, yaw: float) -> float:
        """PCA 축의 180도 모호성으로 인한 뒤집힘을 방지한다."""
        if self._last_yaw is None:
            self._last_yaw = float(yaw)
            return float(yaw)

        # 같은 축의 양방향(yaw, yaw+pi) 중 이전 yaw와 더 가까운 값을 사용한다.
        cand1 = float(yaw)
        cand2 = float(yaw + np.pi)
        d1 = abs(self._angle_diff(cand1, self._last_yaw))
        d2 = abs(self._angle_diff(cand2, self._last_yaw))
        chosen = cand1 if d1 <= d2 else cand2
        self._last_yaw = chosen
        return chosen

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        return float(np.arctan2(np.sin(a - b), np.cos(a - b)))
