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

    def __init__(self, logger, default_opening_m: float = 0.04):
        self.logger = logger
        self.default_opening_m = float(default_opening_m)
        self._load_error: Optional[str] = None
        self._warned = False
        self._last_yaw: Optional[float] = None

    @property
    def is_ready(self) -> bool:
        return True

    @property
    def load_error(self) -> Optional[str]:
        return self._load_error

    def infer(self, points_xyz: np.ndarray, max_candidates: int, lims: list[float]) -> list[GraspCandidate]:
        """ROI 포인트클라우드에서 grasp 후보를 생성한다.

        TODO:
        - GPD ROS 인터페이스(/detect_grasps 결과) 연동
        - 또는 GPD 라이브러리 직접 연동
        """
        if points_xyz.size == 0:
            return []

        # 현재 단계에서는 중단 없는 통합 검증을 위해 중심점 기반 후보를 생성한다.
        if not self._warned:
            self.logger.warn("GPD 실연동 전 단계입니다. 현재는 중심점 기반 임시 grasp를 생성합니다.")
            self._warned = True

        centroid = np.mean(points_xyz, axis=0)
        # ROI 포인트의 평면 분포(PCA)로 장축 방향을 추정하고,
        # 장축+90도(단축 파지)로 yaw를 설정한다.
        raw_yaw = self._estimate_yaw_from_points(points_xyz)
        yaw = raw_yaw + (np.pi * 0.5)
        yaw = self._stabilize_yaw(yaw)
        q = self._yaw_to_quat_xyzw(yaw)
        # 임시 휴리스틱 단계에서는 절대 z 거리로 점수를 깎지 않는다.
        # (테이블 높이가 큰 환경에서 작은 물체가 min_grasp_score에 걸리는 문제 방지)
        score = 0.6

        return [
            GraspCandidate(
                translation=np.array([centroid[0], centroid[1], centroid[2]], dtype=float),
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
