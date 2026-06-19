#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import os
import threading
import time
from copy import deepcopy
from typing import Any

import cv2
import message_filters
import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_geometry_msgs  # noqa: F401  # PointStamped TF 변환 타입 등록용
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped
from PIL import Image as PilImage
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


class Florence2Detector:
    """Florence-2 모델 로딩과 객체 검출 실행을 담당합니다."""

    def __init__(
        self,
        model_id: str,
        device: str,
        confidence_threshold: float,
        model_cache_dir: str,
        logger,
        num_beams: int = 1,
        max_new_tokens: int = 256,
    ):
        self.model_id = model_id
        self.requested_device = device
        self.confidence_threshold = confidence_threshold
        self.model_cache_dir = model_cache_dir
        self.logger = logger
        self.num_beams = max(1, num_beams)
        self.max_new_tokens = max(64, max_new_tokens)
        self.device = "cpu"
        self.model_dtype = None
        self.torch = None
        self.processor = None
        self.model = None
        self._load_model()

    def _load_model(self) -> None:
        """PyTorch CUDA 사용 가능 여부에 따라 모델 실행 장치를 결정합니다."""
        try:
            import torch
            import transformers
            from transformers import AutoModelForCausalLM, AutoProcessor
        except Exception as exc:
            raise RuntimeError(f"Florence-2 의존성 로딩 실패: {exc}") from exc

        # Florence-2 remote code는 transformers 5.x에서 호환성 이슈가 발생할 수 있습니다.
        major_version = int(str(transformers.__version__).split(".", maxsplit=1)[0])
        if major_version >= 5:
            raise RuntimeError(
                "현재 transformers 버전이 Florence-2와 호환되지 않습니다. "
                "transformers==4.49.0으로 설치해 주세요."
            )

        self.torch = torch
        cuda_available = torch.cuda.is_available()
        self.logger.info(
            f"[DIAG] torch={torch.__version__}  CUDA available={cuda_available}"
            + (f"  device_name={torch.cuda.get_device_name(0)}" if cuda_available else "")
        )
        if not cuda_available:
            self.logger.warn(
                "[DIAG] CUDA를 사용할 수 없습니다 — CPU로 실행됩니다 (매우 느림). "
                "ROS2 환경의 Python이 CUDA 빌드 torch를 사용하는지 확인하세요: "
                "python3 -c \"import torch; print(torch.cuda.is_available())\""
            )

        if self.requested_device == "auto":
            self.device = "cuda" if cuda_available else "cpu"
        else:
            self.device = self.requested_device

        dtype = torch.float16 if self.device == "cuda" else torch.float32
        self.model_dtype = dtype
        cache_dir = self.model_cache_dir or None
        if cache_dir:
            os.makedirs(cache_dir, exist_ok=True)
        self.logger.info(f"Florence-2 모델 로딩: {self.model_id}, device={self.device}, dtype={dtype}, cache_dir={cache_dir}")
        self.processor = AutoProcessor.from_pretrained(
            self.model_id,
            trust_remote_code=True,
            cache_dir=cache_dir,
        )
        self.model = AutoModelForCausalLM.from_pretrained(
            self.model_id,
            trust_remote_code=True,
            torch_dtype=dtype,
            cache_dir=cache_dir,
        ).to(self.device)
        self.model.eval()

    @staticmethod
    def build_prompt(target_objects: list[str]) -> str:
        """다중 객체 목록을 Florence-2 open vocabulary prompt로 구성합니다."""
        object_text = ", ".join(target_objects)
        return f"<OPEN_VOCABULARY_DETECTION>{object_text}"

    def detect(self, rgb_bgr: np.ndarray, target_objects: list[str]) -> list[dict[str, Any]]:
        """RGB 이미지에서 target_objects에 해당하는 bbox 목록을 반환합니다."""
        if not target_objects:
            return []

        rgb_image = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2RGB)
        pil_image = PilImage.fromarray(rgb_image)
        prompt = self.build_prompt(target_objects)

        inputs = self.processor(text=prompt, images=pil_image, return_tensors="pt")
        converted_inputs = {}
        for key, value in inputs.items():
            if hasattr(value, "dtype") and self.torch.is_floating_point(value):
                converted_inputs[key] = value.to(device=self.device, dtype=self.model_dtype)
            else:
                converted_inputs[key] = value.to(self.device)
        inputs = converted_inputs

        with self.torch.no_grad():
            generated_ids = self.model.generate(
                input_ids=inputs["input_ids"],
                pixel_values=inputs["pixel_values"],
                max_new_tokens=self.max_new_tokens,
                num_beams=self.num_beams,
            )

        generated_text = self.processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
        parsed = self.processor.post_process_generation(
            generated_text,
            task="<OPEN_VOCABULARY_DETECTION>",
            image_size=(rgb_bgr.shape[1], rgb_bgr.shape[0]),
        )
        return self._normalize_detections(parsed, target_objects)

    def _normalize_detections(self, parsed: Any, target_objects: list[str]) -> list[dict[str, Any]]:
        """Florence-2 post-process 결과 형식 차이를 흡수해 공통 dict로 정리합니다."""
        payload = parsed
        if isinstance(parsed, dict):
            payload = parsed.get("<OPEN_VOCABULARY_DETECTION>", parsed)

        bboxes = payload.get("bboxes", []) if isinstance(payload, dict) else []
        # <OPEN_VOCABULARY_DETECTION> 태스크는 'bboxes_labels' 키를 사용하고 'labels'가 없음
        labels = payload.get("bboxes_labels", payload.get("labels", [])) if isinstance(payload, dict) else []
        scores = payload.get("scores", []) if isinstance(payload, dict) else []

        results = []
        target_set = {name.lower() for name in target_objects}
        # 실사용에서 자주 바뀌는 라벨 동의어를 허용합니다.
        alias_map = {
            "mug": {"mug", "cup", "coffee cup", "coffee mug", "teacup", "tea cup"},
            "cup": {"cup", "mug", "coffee cup", "coffee mug", "teacup", "tea cup"},
            "bottle": {"bottle", "water bottle", "drink bottle"},
            "person": {"person", "human", "man", "woman"},
            "chair": {"chair", "office chair", "stool"},
            "box": {"box", "carton", "package"},
        }

        expanded_targets = set()
        for name in target_set:
            expanded_targets.add(name)
            expanded_targets.update(alias_map.get(name, set()))
        for idx, bbox in enumerate(bboxes):
            label = str(labels[idx]) if idx < len(labels) else "object"
            score = float(scores[idx]) if idx < len(scores) else 1.0
            if score < self.confidence_threshold:
                continue
            label_lower = label.lower()
            if expanded_targets and label_lower not in expanded_targets:
                # Florence 출력 label이 문장 형태일 수 있어 부분 포함도 허용합니다.
                if not any(target in label_lower for target in expanded_targets):
                    continue
            xmin, ymin, xmax, ymax = [int(round(float(v))) for v in bbox[:4]]
            results.append(
                {
                    "object_name": label,
                    "confidence": score,
                    "bbox": {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax},
                }
            )
        return results


class DepthToPointConverter:
    """Depth 이미지와 CameraInfo intrinsic으로 픽셀을 3D 점으로 변환합니다."""

    def __init__(
        self,
        depth_scale: float,
        min_depth_m: float,
        max_depth_m: float,
        roi_inner_scale: float = 0.6,
        roi_depth_percentile: float = 35.0,
        roi_depth_band_m: float = 0.03,
        roi_depth_segmentation_min_area_px: int = 10,
        roi_depth_segmentation_kernel_px: int = 1,
    ):
        self.depth_scale = depth_scale
        self.min_depth_m = min_depth_m
        self.max_depth_m = max_depth_m
        # bbox 가장자리의 배경 혼입을 줄이기 위한 내부 ROI 비율
        self.roi_inner_scale = max(0.1, min(1.0, float(roi_inner_scale)))
        # 배경보다 가까운 객체 표면을 우선 선택하기 위한 depth 백분위수
        self.roi_depth_percentile = max(1.0, min(99.0, float(roi_depth_percentile)))
        # ROI 포인트클라우드에서 대표 depth 주변만 남기는 밴드 폭 (0이면 비활성)
        self.roi_depth_band_m = max(0.0, float(roi_depth_band_m))
        # depth 세그멘테이션 후 최소 객체 픽셀 수
        self.roi_depth_segmentation_min_area_px = max(1, int(roi_depth_segmentation_min_area_px))
        # 세그멘테이션 마스크 정제를 위한 morphology kernel 크기(홀수)
        kernel_px = max(1, int(roi_depth_segmentation_kernel_px))
        self.roi_depth_segmentation_kernel_px = kernel_px if kernel_px % 2 == 1 else kernel_px + 1

    def depth_at_roi(
        self,
        depth_image: np.ndarray,
        bbox: dict[str, int],
        roi_mask: np.ndarray | None = None,
    ) -> tuple[float | None, tuple[int, int]]:
        """bbox 중심 depth가 유효하지 않으면 내부 ROI의 가까운 depth 백분위수를 사용합니다."""
        height, width = depth_image.shape[:2]
        xmin = max(0, min(width - 1, bbox["xmin"]))
        xmax = max(0, min(width - 1, bbox["xmax"]))
        ymin = max(0, min(height - 1, bbox["ymin"]))
        ymax = max(0, min(height - 1, bbox["ymax"]))
        u = int((xmin + xmax) * 0.5)
        v = int((ymin + ymax) * 0.5)
        roi_raw = depth_image[ymin : ymax + 1, xmin : xmax + 1]
        mask_ok = (
            roi_mask is not None
            and roi_mask.shape[0] == roi_raw.shape[0]
            and roi_mask.shape[1] == roi_raw.shape[1]
        )
        roi_mask_local = roi_mask.astype(bool) if mask_ok else None

        center_depth = self._raw_depth_to_meter(depth_image[v, u])
        if self._is_valid_depth(center_depth):
            if roi_mask_local is None or roi_mask_local[v - ymin, u - xmin]:
                return center_depth, (u, v)

        if roi_mask_local is not None:
            depth_m = self._array_depth_to_meter(roi_raw)
            valid = np.isfinite(depth_m) & (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m) & roi_mask_local
            if not np.any(valid):
                return None, (u, v)
            ys, xs = np.where(valid)
            dist2 = ((xs - (u - xmin)) ** 2) + ((ys - (v - ymin)) ** 2)
            idx = int(np.argmin(dist2))
            pu = int(xmin + xs[idx])
            pv = int(ymin + ys[idx])
            return float(depth_m[ys[idx], xs[idx]]), (pu, pv)

        if self._is_valid_depth(center_depth):
            return center_depth, (u, v)

        ixmin, ixmax, iymin, iymax = self._inner_bounds(xmin, xmax, ymin, ymax)
        roi = depth_image[iymin : iymax + 1, ixmin : ixmax + 1]
        depth_m = self._array_depth_to_meter(roi)
        valid = depth_m[np.isfinite(depth_m)]
        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        if valid.size == 0:
            return None, (u, v)
        return float(np.percentile(valid, self.roi_depth_percentile)), (u, v)

    def pixel_to_camera_point(
        self,
        u: int,
        v: int,
        depth_m: float,
        camera_info: CameraInfo,
        frame_id: str,
    ) -> PointStamped:
        """CameraInfo K 행렬을 이용해 optical frame 기준 3D 좌표를 계산합니다."""
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        point = PointStamped()
        point.header = camera_info.header
        point.header.frame_id = frame_id
        point.point.x = (u - cx) * depth_m / fx
        point.point.y = (v - cy) * depth_m / fy
        point.point.z = depth_m
        return point

    def roi_to_camera_points(
        self,
        depth_image: np.ndarray,
        bbox: dict[str, int],
        camera_info: CameraInfo,
        roi_mask: np.ndarray | None = None,
    ) -> list[tuple]:
        """Depth ROI를 camera optical frame의 XYZ 포인트 집합으로 변환합니다."""
        height, width = depth_image.shape[:2]
        xmin = max(0, min(width - 1, bbox["xmin"]))
        xmax = max(0, min(width - 1, bbox["xmax"]))
        ymin = max(0, min(height - 1, bbox["ymin"]))
        ymax = max(0, min(height - 1, bbox["ymax"]))
        if xmax < xmin or ymax < ymin:
            return []

        roi_raw = depth_image[ymin : ymax + 1, xmin : xmax + 1]
        depth_m = self._array_depth_to_meter(roi_raw)
        valid = np.isfinite(depth_m) & (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)
        if roi_mask is not None and roi_mask.shape == depth_m.shape:
            valid = valid & roi_mask.astype(bool)
        if not np.any(valid):
            return []
        if self.roi_depth_band_m > 0.0:
            valid_depths = depth_m[valid]
            depth_ref = float(np.percentile(valid_depths, self.roi_depth_percentile))
            valid = valid & (np.abs(depth_m - depth_ref) <= self.roi_depth_band_m)
            if not np.any(valid):
                return []

        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        u = np.arange(xmin, xmax + 1, dtype=np.float32)
        v = np.arange(ymin, ymax + 1, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        z = depth_m[valid]
        x = ((uu[valid] - cx) * z) / fx
        y = ((vv[valid] - cy) * z) / fy
        points = np.stack((x, y, z), axis=1).astype(np.float32)
        return [tuple(row) for row in points]

    def build_depth_segmentation_mask(self, depth_image: np.ndarray, bbox: dict[str, int]) -> np.ndarray | None:
        """bbox 내부 depth 분포에서 객체 마스크를 추정합니다."""
        height, width = depth_image.shape[:2]
        xmin = max(0, min(width - 1, bbox["xmin"]))
        xmax = max(0, min(width - 1, bbox["xmax"]))
        ymin = max(0, min(height - 1, bbox["ymin"]))
        ymax = max(0, min(height - 1, bbox["ymax"]))
        if xmax < xmin or ymax < ymin:
            return None

        roi_raw = depth_image[ymin : ymax + 1, xmin : xmax + 1]
        depth_m = self._array_depth_to_meter(roi_raw)
        valid = np.isfinite(depth_m) & (depth_m >= self.min_depth_m) & (depth_m <= self.max_depth_m)
        if not np.any(valid):
            return None

        valid_depths = depth_m[valid]
        segmentation_percentile = min(self.roi_depth_percentile, 15.0)
        depth_ref = float(np.percentile(valid_depths, segmentation_percentile))
        band = self.roi_depth_band_m if self.roi_depth_band_m > 0.0 else 0.03
        mask = valid & (np.abs(depth_m - depth_ref) <= band)
        if not np.any(mask):
            return None

        mask_u8 = (mask.astype(np.uint8) * 255)
        if self.roi_depth_segmentation_kernel_px > 1:
            kernel = np.ones(
                (self.roi_depth_segmentation_kernel_px, self.roi_depth_segmentation_kernel_px),
                dtype=np.uint8,
            )
            mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_OPEN, kernel, iterations=1)
            mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel, iterations=1)
        if int(np.count_nonzero(mask_u8)) == 0:
            return None

        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8, connectivity=8)
        if num_labels <= 1:
            return (mask_u8 > 0)

        cx = int((xmax - xmin) * 0.5)
        cy = int((ymax - ymin) * 0.5)
        selected_label = -1
        center_label = int(labels[cy, cx]) if 0 <= cy < labels.shape[0] and 0 <= cx < labels.shape[1] else 0
        if center_label > 0 and int(stats[center_label, cv2.CC_STAT_AREA]) >= self.roi_depth_segmentation_min_area_px:
            selected_label = center_label
        else:
            best_score = -1.0
            for label in range(1, num_labels):
                area = int(stats[label, cv2.CC_STAT_AREA])
                if area < self.roi_depth_segmentation_min_area_px:
                    continue
                comp_cx = float(stats[label, cv2.CC_STAT_LEFT] + stats[label, cv2.CC_STAT_WIDTH] * 0.5)
                comp_cy = float(stats[label, cv2.CC_STAT_TOP] + stats[label, cv2.CC_STAT_HEIGHT] * 0.5)
                dist = float(np.hypot(comp_cx - cx, comp_cy - cy))
                # 얇은 물체에서는 가장 큰 배경 조각보다 bbox 중심에 가까운 컴포넌트를 우선합니다.
                score = float(area) / (1.0 + dist)
                if score > best_score:
                    best_score = score
                    selected_label = label
        if selected_label <= 0:
            return None
        return labels == selected_label

    def _raw_depth_to_meter(self, value: Any) -> float:
        if isinstance(value, np.floating):
            return float(value)
        return float(value) * self.depth_scale

    def _array_depth_to_meter(self, depth: np.ndarray) -> np.ndarray:
        if np.issubdtype(depth.dtype, np.floating):
            return depth.astype(np.float32)
        return depth.astype(np.float32) * self.depth_scale

    def _is_valid_depth(self, depth_m: float) -> bool:
        return math.isfinite(depth_m) and self.min_depth_m <= depth_m <= self.max_depth_m

    def _inner_bounds(self, xmin: int, xmax: int, ymin: int, ymax: int) -> tuple[int, int, int, int]:
        width = max(1, xmax - xmin + 1)
        height = max(1, ymax - ymin + 1)
        cx = (xmin + xmax) * 0.5
        cy = (ymin + ymax) * 0.5
        half_w = max(1.0, 0.5 * width * self.roi_inner_scale)
        half_h = max(1.0, 0.5 * height * self.roi_inner_scale)
        ixmin = int(round(cx - half_w))
        ixmax = int(round(cx + half_w))
        iymin = int(round(cy - half_h))
        iymax = int(round(cy + half_h))
        ixmin = max(xmin, min(xmax, ixmin))
        ixmax = max(ixmin, min(xmax, ixmax))
        iymin = max(ymin, min(ymax, iymin))
        iymax = max(iymin, min(ymax, iymax))
        return ixmin, ixmax, iymin, iymax


class TFPointTransformer:
    """camera frame 점을 target frame으로 변환합니다."""

    def __init__(self, node: Node):
        self.node = node
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, node)

    def transform(
        self,
        point: PointStamped,
        target_frame: str,
        timeout_sec: float,
        use_latest: bool = False,
    ) -> tuple[PointStamped | None, bool]:
        source = point
        if use_latest:
            source = deepcopy(point)
            source.header.stamp = Time().to_msg()
        try:
            transformed = self.buffer.transform(
                source,
                target_frame,
                timeout=Duration(seconds=timeout_sec),
            )
            return transformed, True
        except Exception as exc:
            self.node.get_logger().warn(f"TF 변환 실패: {point.header.frame_id} -> {target_frame}: {exc}")
            return None, False


class PointCloudROIFilter:
    """PointCloud2에서 bbox ROI에 해당하는 포인트만 추출합니다.

    organized cloud(ordered_pc=true)는 UV 인덱스, unorganized cloud는
    카메라 내부 파라미터(K)를 이용한 frustum 투영으로 필터링합니다.
    """

    def __init__(self, logger):
        self.logger = logger
        self.xyz_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

    def filter(
        self,
        cloud_msg: PointCloud2,
        bbox: dict[str, int],
        camera_info: CameraInfo | None = None,
    ) -> tuple[PointCloud2, int]:
        points, count = self.extract_points(cloud_msg, bbox, camera_info)
        roi_cloud = self.make_cloud(cloud_msg, points)
        return roi_cloud, count

    def extract_points(
        self,
        cloud_msg: PointCloud2,
        bbox: dict[str, int],
        camera_info: CameraInfo | None = None,
    ) -> tuple[list[tuple], int]:
        if cloud_msg.height <= 1:
            if camera_info is None:
                self.logger.warn("ROI PointCloud: unorganized cloud에 camera_info 없음, 건너뜀")
                return [], 0
            return self._frustum_filter(cloud_msg, bbox, camera_info)

        xmin = max(0, min(cloud_msg.width - 1, bbox["xmin"]))
        xmax = max(0, min(cloud_msg.width - 1, bbox["xmax"]))
        ymin = max(0, min(cloud_msg.height - 1, bbox["ymin"]))
        ymax = max(0, min(cloud_msg.height - 1, bbox["ymax"]))
        uvs = [(u, v) for v in range(ymin, ymax + 1) for u in range(xmin, xmax + 1)]

        points = []
        try:
            for point in pc2.read_points(cloud_msg, skip_nans=True, uvs=uvs):
                points.append(self._point_to_tuple(point))
        except Exception as exc:
            self.logger.warn(f"ROI PointCloud 추출 실패: {exc}")
            points = []

        return points, len(points)

    def extract_points_by_mask(
        self,
        cloud_msg: PointCloud2,
        bbox: dict[str, int],
        roi_mask: np.ndarray,
        camera_info: CameraInfo | None = None,
    ) -> tuple[list[tuple], int]:
        """organized cloud에서 segmentation mask 픽셀에 해당하는 포인트만 추출합니다."""
        if cloud_msg.height <= 1:
            return self.extract_points(cloud_msg, bbox, camera_info)

        xmin = max(0, min(cloud_msg.width - 1, bbox["xmin"]))
        xmax = max(0, min(cloud_msg.width - 1, bbox["xmax"]))
        ymin = max(0, min(cloud_msg.height - 1, bbox["ymin"]))
        ymax = max(0, min(cloud_msg.height - 1, bbox["ymax"]))
        expected_shape = (ymax - ymin + 1, xmax - xmin + 1)
        if roi_mask.shape != expected_shape:
            return self.extract_points(cloud_msg, bbox, camera_info)

        ys, xs = np.where(roi_mask.astype(bool))
        if ys.size == 0:
            return [], 0
        uvs = [(int(xmin + x), int(ymin + y)) for y, x in zip(ys.tolist(), xs.tolist())]

        points = []
        try:
            for point in pc2.read_points(cloud_msg, skip_nans=True, uvs=uvs):
                points.append(self._point_to_tuple(point))
        except Exception as exc:
            self.logger.warn(f"ROI PointCloud(mask) 추출 실패: {exc}")
            points = []
        return points, len(points)

    def _frustum_filter(
        self,
        cloud_msg: PointCloud2,
        bbox: dict[str, int],
        camera_info: CameraInfo,
    ) -> tuple[list[tuple], int]:
        """카메라 핀홀 투영으로 unorganized cloud를 bbox 시야각 내로 필터링합니다."""
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]
        xmin, xmax = float(bbox["xmin"]), float(bbox["xmax"])
        ymin, ymax = float(bbox["ymin"]), float(bbox["ymax"])

        points = []
        try:
            for point in pc2.read_points(cloud_msg, skip_nans=True):
                pt = self._point_to_tuple(point)
                z = float(pt[2])
                if z <= 0.0:
                    continue
                u = float(pt[0]) * fx / z + cx
                v = float(pt[1]) * fy / z + cy
                if xmin <= u <= xmax and ymin <= v <= ymax:
                    points.append(pt)
        except Exception as exc:
            self.logger.warn(f"ROI PointCloud frustum 필터 실패: {exc}")
            points = []

        return points, len(points)

    def make_cloud(self, cloud_msg: PointCloud2, points: list[tuple]) -> PointCloud2:
        return pc2.create_cloud(cloud_msg.header, cloud_msg.fields, points)

    def make_xyz_cloud(self, header, points: list[tuple]) -> PointCloud2:
        return pc2.create_cloud(header, self.xyz_fields, points)

    @staticmethod
    def _point_to_tuple(point: Any) -> tuple:
        if hasattr(point, "dtype") and point.dtype.names:
            values = []
            for name in point.dtype.names:
                value = point[name]
                values.append(value.item() if hasattr(value, "item") else value)
            return tuple(values)
        return tuple(point)


class DetectionVisualizer:
    """Debug image와 RViz MarkerArray 생성을 담당합니다."""

    def draw_debug(
        self,
        image_bgr: np.ndarray,
        objects: list[dict[str, Any]],
        mask_overlays: list[dict[str, Any]] | None = None,
    ) -> np.ndarray:
        output = image_bgr.copy()
        if mask_overlays:
            overlay = output.copy()
            for item in mask_overlays:
                bbox = item.get("bbox", {})
                mask = item.get("mask")
                if mask is None:
                    continue
                x1, y1 = int(bbox.get("xmin", 0)), int(bbox.get("ymin", 0))
                x2, y2 = int(bbox.get("xmax", x1)), int(bbox.get("ymax", y1))
                if x2 < x1 or y2 < y1:
                    continue
                h, w = mask.shape[:2]
                roi = overlay[y1 : y1 + h, x1 : x1 + w]
                if roi.shape[:2] != mask.shape[:2]:
                    continue
                roi[mask.astype(bool)] = (0, 180, 255)
            output = cv2.addWeighted(overlay, 0.35, output, 0.65, 0.0)
        for obj in objects:
            bbox = obj["bbox"]
            x1, y1, x2, y2 = bbox["xmin"], bbox["ymin"], bbox["xmax"], bbox["ymax"]
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            pos = obj.get("position_target_frame") or obj["position_camera_frame"]
            label = (
                f"{obj['object_id']} {obj['object_name']} "
                f"({pos['x']:.2f}, {pos['y']:.2f}, {pos['z']:.2f})"
            )
            cv2.putText(output, label, (x1, max(20, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        return output

    def make_markers(self, objects: list[dict[str, Any]], target_frame: str, stamp) -> MarkerArray:
        markers = MarkerArray()
        for idx, obj in enumerate(objects):
            pos = obj.get("position_target_frame") or obj["position_camera_frame"]
            frame_id = pos.get("frame_id", target_frame)

            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = "sm_florence_2_vlm"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = Point(x=float(pos["x"]), y=float(pos["y"]), z=float(pos["z"]))
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color = ColorRGBA(r=0.1, g=1.0, b=0.1, a=0.9)
            markers.markers.append(marker)

            text = Marker()
            text.header = marker.header
            text.ns = "sm_florence_2_vlm_text"
            text.id = idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position = Point(x=float(pos["x"]), y=float(pos["y"]), z=float(pos["z"]) + 0.25)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.18
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = obj["object_id"]
            markers.markers.append(text)
        return markers


class Florence2VLMNode(Node):
    """RealSense RGB-D 입력을 Florence-2 VLM으로 검출하고 3D 결과를 발행합니다."""

    def __init__(self):
        super().__init__("sm_florence_2_vlm")
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.cloud_lock = threading.Lock()
        self.objects_lock = threading.Lock()
        self.target_lock = threading.Lock()
        self.latest_frame = None
        self.latest_cloud: PointCloud2 | None = None
        self.latest_objects: list[dict[str, Any]] = []
        self.latest_mask_overlays: list[dict[str, Any]] = []
        self._stop_event = threading.Event()
        self.last_detection_msg = None
        self.last_marker_msg = None
        self.last_roi_cloud_msg = None
        self._last_segmentation_fallback_log_time = 0.0

        self._declare_parameters()
        self._load_parameters()

        self.detector = Florence2Detector(
            self.model_id,
            self.device,
            self.confidence_threshold,
            self.model_cache_dir,
            self.get_logger(),
            num_beams=self.num_beams,
            max_new_tokens=self.max_new_tokens,
        )
        self.depth_converter = DepthToPointConverter(
            self.depth_scale,
            self.min_depth_m,
            self.max_depth_m,
            self.roi_depth_inner_scale,
            self.roi_depth_percentile,
            self.roi_depth_band_m,
            self.roi_depth_segmentation_min_area_px,
            self.roi_depth_segmentation_kernel_px,
        )
        self.tf_transformer = TFPointTransformer(self)
        self.roi_filter = PointCloudROIFilter(self.get_logger())
        self.visualizer = DetectionVisualizer()

        self.detection_pub = self.create_publisher(String, "/sm_florence_2_vlm/detections", 10)
        self.debug_pub = self.create_publisher(Image, "/sm_florence_2_vlm/debug/image", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/sm_florence_2_vlm/markers", 10)
        self.roi_cloud_pub = self.create_publisher(PointCloud2, "/sm_florence_2_vlm/roi_pointcloud", 10)

        self._setup_subscribers()
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread.start()
        self.get_logger().info(f"sm_florence_2_vlm 시작: inference_rate_hz={self.inference_rate_hz}")
        self.get_logger().info(
            "ROI 모드=%s, fallback_to_bbox=%s, roi_pointcloud_source=%s"
            % (self.roi_geometry_mode, self.roi_geometry_fallback_to_bbox, self.roi_pointcloud_source)
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("pointcloud_topic", "/camera/camera/depth/color/points")
        self.declare_parameter("target_objects", ["person", "chair", "box"])
        self.declare_parameter("target_objects_topic", "/sm_florence_2_vlm/target_objects")
        self.declare_parameter("use_target_objects_topic", True)
        self.declare_parameter("model_id", "microsoft/Florence-2-base")
        self.declare_parameter("model_cache_dir", "/home/kiro/colcon_ws/src/sm_florence_2_vlm_ros2/models")
        self.declare_parameter("device", "auto")
        self.declare_parameter("confidence_threshold", 0.3)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_roi_pointcloud", True)
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("min_depth_m", 0.2)
        self.declare_parameter("max_depth_m", 10.0)
        self.declare_parameter("roi_depth_inner_scale", 0.6)
        self.declare_parameter("roi_depth_percentile", 35.0)
        self.declare_parameter("roi_depth_band_m", 0.03)
        self.declare_parameter("roi_geometry_mode", "bbox")
        self.declare_parameter("roi_geometry_fallback_to_bbox", True)
        self.declare_parameter("roi_depth_segmentation_min_area_px", 10)
        self.declare_parameter("roi_depth_segmentation_kernel_px", 1)
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("tf_use_latest_transform", False)
        self.declare_parameter("object_id_mode", "per_frame")
        self.declare_parameter("inference_rate_hz", 3.0)
        self.declare_parameter("drop_old_frames", True)
        self.declare_parameter("publish_last_detection_when_no_update", False)
        self.declare_parameter("num_beams", 1)
        self.declare_parameter("max_new_tokens", 64)
        self.declare_parameter("inference_input_width", 640)
        self.declare_parameter("enable_inference_clahe", True)
        self.declare_parameter("clahe_clip_limit", 2.0)
        self.declare_parameter("clahe_tile_grid_size", 8)
        self.declare_parameter("enable_perf_log", True)
        self.declare_parameter("perf_log_interval_sec", 2.0)
        self.declare_parameter("roi_pointcloud_source", "depth")
        self.declare_parameter("roi_pointcloud_require_organized", True)

    def _load_parameters(self) -> None:
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.target_objects = self._safe_get_target_objects()
        self.target_objects_topic = self.get_parameter("target_objects_topic").value
        self.use_target_objects_topic = bool(self.get_parameter("use_target_objects_topic").value)
        self.model_id = self.get_parameter("model_id").value
        self.model_cache_dir = self.get_parameter("model_cache_dir").value
        self.device = self.get_parameter("device").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.publish_markers = bool(self.get_parameter("publish_markers").value)
        self.publish_roi_pointcloud = bool(self.get_parameter("publish_roi_pointcloud").value)
        self.queue_size = int(self.get_parameter("queue_size").value)
        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.min_depth_m = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)
        self.roi_depth_inner_scale = float(self.get_parameter("roi_depth_inner_scale").value)
        self.roi_depth_percentile = float(self.get_parameter("roi_depth_percentile").value)
        self.roi_depth_band_m = float(self.get_parameter("roi_depth_band_m").value)
        roi_geometry_mode = str(self.get_parameter("roi_geometry_mode").value).strip().lower()
        if roi_geometry_mode in {"segmentation", "mask", "depth_segmentation"}:
            self.roi_geometry_mode = "depth_segmentation"
        elif roi_geometry_mode == "bbox":
            self.roi_geometry_mode = "bbox"
        else:
            self.get_logger().warn(f"roi_geometry_mode={roi_geometry_mode}는 지원되지 않아 bbox로 대체합니다.")
            self.roi_geometry_mode = "bbox"
        self.roi_geometry_fallback_to_bbox = bool(self.get_parameter("roi_geometry_fallback_to_bbox").value)
        self.roi_depth_segmentation_min_area_px = int(self.get_parameter("roi_depth_segmentation_min_area_px").value)
        self.roi_depth_segmentation_kernel_px = int(self.get_parameter("roi_depth_segmentation_kernel_px").value)
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.use_tf_transform = bool(self.get_parameter("use_tf_transform").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.tf_use_latest_transform = bool(self.get_parameter("tf_use_latest_transform").value)
        self.object_id_mode = self.get_parameter("object_id_mode").value
        self.inference_rate_hz = float(self.get_parameter("inference_rate_hz").value)
        self.drop_old_frames = bool(self.get_parameter("drop_old_frames").value)
        self.publish_last_detection_when_no_update = bool(
            self.get_parameter("publish_last_detection_when_no_update").value
        )
        self.num_beams = int(self.get_parameter("num_beams").value)
        self.max_new_tokens = int(self.get_parameter("max_new_tokens").value)
        self.inference_input_width = int(self.get_parameter("inference_input_width").value)
        self.enable_inference_clahe = bool(self.get_parameter("enable_inference_clahe").value)
        self.clahe_clip_limit = float(self.get_parameter("clahe_clip_limit").value)
        self.clahe_tile_grid_size = max(2, int(self.get_parameter("clahe_tile_grid_size").value))
        self.enable_perf_log = bool(self.get_parameter("enable_perf_log").value)
        self.perf_log_interval_sec = float(self.get_parameter("perf_log_interval_sec").value)
        self.roi_pointcloud_source = str(self.get_parameter("roi_pointcloud_source").value).strip().lower()
        if self.roi_pointcloud_source not in {"depth", "pointcloud"}:
            self.get_logger().warn(
                f"roi_pointcloud_source={self.roi_pointcloud_source}는 지원되지 않아 depth로 대체합니다."
            )
            self.roi_pointcloud_source = "depth"
        self.roi_pointcloud_require_organized = bool(
            self.get_parameter("roi_pointcloud_require_organized").value
        )
        self._last_perf_log_time = 0.0

    def _safe_get_target_objects(self) -> list[str]:
        """빈 배열 override에서 파라미터가 미초기화(NOT_SET)로 들어오는 경우를 안전 처리합니다."""
        try:
            value = self.get_parameter("target_objects").value
            if value is None:
                return []
            return [str(x).strip() for x in list(value) if str(x).strip()]
        except ParameterUninitializedException:
            self.get_logger().warn(
                "target_objects 파라미터가 비어 있어 []로 초기화합니다. "
                "런타임에 /sm_florence_2_vlm/target_objects 토픽으로 객체를 입력하세요."
            )
            return []

    def _setup_subscribers(self) -> None:
        # RGB + depth + CameraInfo만 동기화 (3-topic sync가 4-topic보다 빠르게 매칭됨)
        rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        info_sub = message_filters.Subscriber(self, CameraInfo, self.camera_info_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub],
            queue_size=self.queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        self.target_objects_sub = None
        if self.use_target_objects_topic:
            self.target_objects_sub = self.create_subscription(
                String,
                self.target_objects_topic,
                self._target_objects_callback,
                self.queue_size,
            )

        # pointcloud 소스를 선택한 경우에만 PointCloud2를 구독합니다.
        self.cloud_sub = None
        if self.publish_roi_pointcloud and self.roi_pointcloud_source == "pointcloud":
            self.cloud_sub = self.create_subscription(
                PointCloud2,
                self.pointcloud_topic,
                self._cloud_callback,
                self.queue_size,
            )

    def _cloud_callback(self, cloud_msg: PointCloud2) -> None:
        with self.cloud_lock:
            self.latest_cloud = cloud_msg

    def _target_objects_callback(self, msg: String) -> None:
        parsed = self._parse_target_objects(msg.data)
        if not parsed:
            self.get_logger().warn(
                "target_objects 토픽 입력이 비어 있거나 형식이 잘못되었습니다. "
                "예: [\"mug\", \"bottle\"] 또는 mug,bottle"
            )
            return
        with self.target_lock:
            self.target_objects = parsed
        self.get_logger().info(f"target_objects 갱신: {parsed}")

    @staticmethod
    def _parse_target_objects(text: str) -> list[str]:
        raw = (text or "").strip()
        if not raw:
            return []

        values: list[str] = []
        if raw.startswith("[") or raw.startswith("{"):
            try:
                payload = json.loads(raw)
                if isinstance(payload, list):
                    values = [str(x).strip() for x in payload]
                elif isinstance(payload, dict):
                    vals = payload.get("target_objects", [])
                    if isinstance(vals, list):
                        values = [str(x).strip() for x in vals]
            except Exception:
                values = []
        else:
            values = [token.strip() for token in raw.split(",")]

        return [v for v in values if v]

    def synced_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
        """프레임을 저장하고 디버그 이미지를 카메라 rate로 즉시 발행합니다."""
        with self.frame_lock:
            self.latest_frame = (rgb_msg, depth_msg, info_msg)

        if self.publish_debug_image:
            rgb_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            with self.objects_lock:
                objects = self.latest_objects
                mask_overlays = self.latest_mask_overlays
            debug = self.visualizer.draw_debug(rgb_bgr, objects, mask_overlays)
            debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_msg.header = rgb_msg.header
            self.debug_pub.publish(debug_msg)

    def _inference_loop(self) -> None:
        """GPU 속도에 맞춰 최신 프레임을 연속 추론합니다. inference_rate_hz는 상한 속도 캡입니다."""
        min_interval = 1.0 / max(self.inference_rate_hz, 0.1)

        while not self._stop_event.is_set():
            t0 = time.monotonic()

            with self.frame_lock:
                frame = self.latest_frame
                if frame is not None and self.drop_old_frames:
                    self.latest_frame = None

            if frame is None:
                if self.publish_last_detection_when_no_update:
                    self._publish_last_results()
                self._stop_event.wait(timeout=0.005)
                continue

            try:
                self._run_inference(frame)
            except Exception as exc:
                self.get_logger().error(f"추론 처리 실패: {exc}")

            elapsed = time.monotonic() - t0
            remaining = min_interval - elapsed
            if remaining > 0:
                self._stop_event.wait(timeout=remaining)

    def destroy_node(self) -> None:
        self._stop_event.set()
        super().destroy_node()

    def _run_inference(self, frame) -> None:
        t_start = time.monotonic()
        rgb_msg, depth_msg, info_msg = frame
        cloud_msg = None
        if self.publish_roi_pointcloud and self.roi_pointcloud_source == "pointcloud":
            with self.cloud_lock:
                cloud_msg = self.latest_cloud
        t_after_msg = time.monotonic()

        rgb_bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        t_after_cv = time.monotonic()

        infer_bgr, sx, sy = self._prepare_inference_image(rgb_bgr)
        t_before_detect = time.monotonic()
        with self.target_lock:
            current_targets = list(self.target_objects)
        raw_detections = self.detector.detect(infer_bgr, current_targets)
        t_after_detect = time.monotonic()

        objects = []
        roi_points = []
        mask_overlays = []
        object_count_by_name = {}

        for det in raw_detections:
            det_bbox = self._scale_bbox_to_original(det["bbox"], sx, sy)
            bbox = self._clip_bbox(det_bbox, rgb_bgr.shape[1], rgb_bgr.shape[0])
            object_name = det["object_name"]
            index = object_count_by_name.get(object_name, 0)
            object_count_by_name[object_name] = index + 1
            object_id = f"{object_name}_{index}"

            roi_mask = None
            roi_mask_used = False
            roi_mask_area_px = 0
            if self.roi_geometry_mode == "depth_segmentation":
                roi_mask = self.depth_converter.build_depth_segmentation_mask(depth_image, bbox)
                if roi_mask is None and not self.roi_geometry_fallback_to_bbox:
                    continue
                if roi_mask is None and self.roi_geometry_fallback_to_bbox:
                    now = time.monotonic()
                    if now - self._last_segmentation_fallback_log_time > 2.0:
                        self.get_logger().warn(
                            "depth segmentation mask 생성 실패로 bbox ROI로 폴백합니다."
                        )
                        self._last_segmentation_fallback_log_time = now
                elif roi_mask is not None:
                    roi_mask_used = True
                    roi_mask_area_px = int(np.count_nonzero(roi_mask))
                    mask_overlays.append({"bbox": bbox, "mask": roi_mask.copy()})

            depth_m, (u, v) = self.depth_converter.depth_at_roi(depth_image, bbox, roi_mask)
            if depth_m is None:
                continue

            camera_point = self.depth_converter.pixel_to_camera_point(u, v, depth_m, info_msg, self.camera_frame)
            target_point = None
            transform_available = False
            if self.use_tf_transform:
                target_point, transform_available = self.tf_transformer.transform(
                    camera_point,
                    self.target_frame,
                    self.tf_timeout_sec,
                    self.tf_use_latest_transform,
                )

            roi_count = 0
            if self.publish_roi_pointcloud:
                if self.roi_pointcloud_source == "depth":
                    points = self.depth_converter.roi_to_camera_points(depth_image, bbox, info_msg, roi_mask)
                    roi_count = len(points)
                    roi_points.extend(points)
                elif cloud_msg is not None:
                    if self.roi_pointcloud_require_organized and cloud_msg.height <= 1:
                        # 속도 우선 모드: unorganized cloud 전체 순회를 건너뜁니다.
                        pass
                    elif roi_mask is not None:
                        points, roi_count = self.roi_filter.extract_points_by_mask(cloud_msg, bbox, roi_mask, info_msg)
                        roi_points.extend(points)
                    else:
                        points, roi_count = self.roi_filter.extract_points(cloud_msg, bbox, info_msg)
                        roi_points.extend(points)

            obj = self._build_detection_object(
                object_id,
                object_name,
                float(det["confidence"]),
                bbox,
                u,
                v,
                camera_point,
                target_point,
                transform_available,
                roi_count,
                self.roi_geometry_mode,
                roi_mask_used,
                roi_mask_area_px,
            )
            objects.append(obj)
        t_after_post = time.monotonic()

        roi_cloud = None
        if self.publish_roi_pointcloud:
            if self.roi_pointcloud_source == "depth":
                roi_header = deepcopy(info_msg.header)
                roi_header.frame_id = self.camera_frame
                if self.use_tf_transform and self.tf_use_latest_transform:
                    roi_header.stamp = Time().to_msg()
                roi_cloud = self.roi_filter.make_xyz_cloud(roi_header, roi_points)
            elif cloud_msg is not None:
                roi_cloud = self.roi_filter.make_cloud(cloud_msg, roi_points)
        self._publish_results(rgb_msg, objects, roi_cloud, mask_overlays)
        t_after_pub = time.monotonic()
        self._log_perf(
            total_ms=(t_after_pub - t_start) * 1000.0,
            msg_ms=(t_after_msg - t_start) * 1000.0,
            cv_ms=(t_after_cv - t_after_msg) * 1000.0,
            detect_ms=(t_after_detect - t_before_detect) * 1000.0,
            post_ms=(t_after_post - t_after_detect) * 1000.0,
            pub_ms=(t_after_pub - t_after_post) * 1000.0,
            det_count=len(raw_detections),
            obj_count=len(objects),
        )

    def _publish_results(
        self,
        rgb_msg: Image,
        objects: list[dict[str, Any]],
        roi_cloud: PointCloud2 | None,
        mask_overlays: list[dict[str, Any]] | None = None,
    ) -> None:
        with self.objects_lock:
            self.latest_objects = objects
            self.latest_mask_overlays = mask_overlays or []

        payload = {
            "header": {
                "stamp": {"sec": rgb_msg.header.stamp.sec, "nanosec": rgb_msg.header.stamp.nanosec},
                "frame_id": self.camera_frame,
            },
            "target_frame": self.target_frame,
            "objects": objects,
        }
        self.detection_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.last_detection_msg = String(data=json.dumps(payload, ensure_ascii=False))

        if self.publish_markers:
            marker_msg = self.visualizer.make_markers(objects, self.target_frame, rgb_msg.header.stamp)
            self.marker_pub.publish(marker_msg)
            self.last_marker_msg = marker_msg

        if self.publish_roi_pointcloud:
            if roi_cloud is not None:
                self.roi_cloud_pub.publish(roi_cloud)
                self.last_roi_cloud_msg = roi_cloud

    def _publish_last_results(self) -> None:
        """새 프레임이 없을 때 마지막 추론 결과를 선택적으로 재발행합니다."""
        if self.last_detection_msg is not None:
            self.detection_pub.publish(self.last_detection_msg)
        if self.publish_markers and self.last_marker_msg is not None:
            self.marker_pub.publish(self.last_marker_msg)
        if self.publish_roi_pointcloud and self.last_roi_cloud_msg is not None:
            self.roi_cloud_pub.publish(self.last_roi_cloud_msg)

    def _build_detection_object(
        self,
        object_id: str,
        object_name: str,
        confidence: float,
        bbox: dict[str, int],
        u: int,
        v: int,
        camera_point: PointStamped,
        target_point: PointStamped | None,
        transform_available: bool,
        roi_count: int,
        roi_geometry_mode: str,
        roi_mask_used: bool,
        roi_mask_area_px: int,
    ) -> dict[str, Any]:
        camera_pos = self._point_to_dict(camera_point, self.camera_frame)
        target_pos = self._point_to_dict(target_point, self.target_frame) if target_point else None
        return {
            "object_id": object_id,
            "object_name": object_name,
            "confidence": confidence,
            "bbox": bbox,
            "center_pixel": {"u": u, "v": v},
            "position_camera_frame": camera_pos,
            "position_target_frame": target_pos,
            "transform_available": transform_available,
            "roi_point_count": roi_count,
            "roi_geometry_mode": roi_geometry_mode,
            "roi_mask_used": bool(roi_mask_used),
            "roi_mask_area_px": int(roi_mask_area_px),
        }

    @staticmethod
    def _point_to_dict(point_msg: PointStamped, fallback_frame: str) -> dict[str, Any]:
        return {
            "frame_id": point_msg.header.frame_id or fallback_frame,
            "x": float(point_msg.point.x),
            "y": float(point_msg.point.y),
            "z": float(point_msg.point.z),
        }

    @staticmethod
    def _clip_bbox(bbox: dict[str, int], width: int, height: int) -> dict[str, int]:
        xmin = max(0, min(width - 1, int(bbox["xmin"])))
        ymin = max(0, min(height - 1, int(bbox["ymin"])))
        xmax = max(0, min(width - 1, int(bbox["xmax"])))
        ymax = max(0, min(height - 1, int(bbox["ymax"])))
        if xmax < xmin:
            xmin, xmax = xmax, xmin
        if ymax < ymin:
            ymin, ymax = ymax, ymin
        return {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax}

    def _prepare_inference_image(self, rgb_bgr: np.ndarray) -> tuple[np.ndarray, float, float]:
        """추론 해상도를 줄여 GPU/CPU 부하를 낮추고, bbox를 원본으로 복원할 배율을 반환합니다."""
        h, w = rgb_bgr.shape[:2]
        target_w = max(0, self.inference_input_width)
        if target_w <= 0 or target_w >= w:
            return self._stabilize_inference_brightness(rgb_bgr), 1.0, 1.0
        target_h = max(2, int(round(h * float(target_w) / float(w))))
        resized = cv2.resize(rgb_bgr, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
        resized = self._stabilize_inference_brightness(resized)
        sx = float(w) / float(target_w)
        sy = float(h) / float(target_h)
        return resized, sx, sy

    def _stabilize_inference_brightness(self, bgr: np.ndarray) -> np.ndarray:
        """조명 변화에 덜 민감하도록 추론 입력의 밝기 대비만 완만하게 보정합니다."""
        if not self.enable_inference_clahe:
            return bgr
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        tile = max(2, int(self.clahe_tile_grid_size))
        clahe = cv2.createCLAHE(
            clipLimit=max(0.1, float(self.clahe_clip_limit)),
            tileGridSize=(tile, tile),
        )
        l_channel = clahe.apply(l_channel)
        merged = cv2.merge((l_channel, a_channel, b_channel))
        return cv2.cvtColor(merged, cv2.COLOR_LAB2BGR)

    @staticmethod
    def _scale_bbox_to_original(bbox: dict[str, int], sx: float, sy: float) -> dict[str, int]:
        return {
            "xmin": int(round(float(bbox["xmin"]) * sx)),
            "ymin": int(round(float(bbox["ymin"]) * sy)),
            "xmax": int(round(float(bbox["xmax"]) * sx)),
            "ymax": int(round(float(bbox["ymax"]) * sy)),
        }

    def _log_perf(
        self,
        total_ms: float,
        msg_ms: float,
        cv_ms: float,
        detect_ms: float,
        post_ms: float,
        pub_ms: float,
        det_count: int,
        obj_count: int,
    ) -> None:
        if not self.enable_perf_log:
            return
        now = time.monotonic()
        if now - self._last_perf_log_time < max(0.1, self.perf_log_interval_sec):
            return
        self._last_perf_log_time = now
        self.get_logger().info(
            "PERF total=%.1fms detect=%.1fms post=%.1fms pub=%.1fms cv=%.1fms det=%d obj=%d"
            % (total_ms, detect_ms, post_ms, pub_ms, cv_ms, det_count, obj_count)
        )


def main(args=None):
    rclpy.init(args=args)
    node = Florence2VLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if rclpy.ok():
            node.get_logger().info("사용자 종료 요청을 받았습니다.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
