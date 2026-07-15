#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import threading
import time
from copy import deepcopy
from typing import Any

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

from sm_florence_2_vlm.florence_2_vlm_node import (
    DepthToPointConverter,
    DetectionVisualizer,
    PointCloudROIFilter,
    TFPointTransformer,
)
from sm_florence_2_vlm.yoloe_utils import (
    crop_mask_to_bbox,
    estimate_mask_orientation,
    expand_yoloe_prompts,
    is_yoloe_geometry_valid,
    match_yoloe_detection_to_target,
    score_mask_colors,
    yoloe_candidate_reliability,
)


class YOLOEDetector:
    """Ultralytics YOLOE segmentation wrapper with cached text prompts."""

    def __init__(
        self,
        model_path: str,
        device: str,
        confidence_threshold: float,
        iou_threshold: float,
        imgsz: int,
        logger,
    ):
        self.model_path = str(model_path)
        self.requested_device = str(device)
        self.confidence_threshold = float(confidence_threshold)
        self.iou_threshold = float(iou_threshold)
        self.imgsz = int(imgsz)
        self.logger = logger
        self.model = None
        self.torch = None
        self.predict_device: str | int = "cpu"
        self._prompt_key: tuple[str, ...] = ()
        self._load_model()

    def _load_model(self) -> None:
        try:
            import torch
            from ultralytics import YOLOE
        except Exception as exc:
            raise RuntimeError(f"YOLOE 의존성 로딩 실패: {exc}") from exc

        self.torch = torch
        cuda_available = bool(torch.cuda.is_available())
        if self.requested_device == "auto":
            self.predict_device = 0 if cuda_available else "cpu"
        elif self.requested_device in {"cuda", "cuda:0", "0"}:
            self.predict_device = 0
        else:
            self.predict_device = self.requested_device

        self.logger.info(
            f"YOLOE 모델 로딩: {self.model_path}, device={self.predict_device}, "
            f"torch={torch.__version__}, cuda={cuda_available}"
        )
        self.model = YOLOE(self.model_path)

    def detect(self, rgb_bgr: np.ndarray, target_objects: list[str]) -> list[dict[str, Any]]:
        prompts = expand_yoloe_prompts(target_objects)
        if not prompts:
            return []
        self._set_prompts_if_needed(prompts)

        results = self.model.predict(
            rgb_bgr,
            imgsz=self.imgsz,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self.predict_device,
            verbose=False,
        )
        if not results:
            return []
        return self._normalize_result(results[0])

    def _set_prompts_if_needed(self, prompts: list[str]) -> None:
        key = tuple(prompts)
        if key == self._prompt_key:
            return
        t0 = time.monotonic()
        embeddings = self.model.get_text_pe(prompts)
        self.model.set_classes(prompts, embeddings)
        self._prompt_key = key
        self.logger.info(
            "YOLOE target prompts=%s embedding_ms=%.1f"
            % (prompts, (time.monotonic() - t0) * 1000.0)
        )

    @staticmethod
    def _normalize_result(result) -> list[dict[str, Any]]:
        boxes = result.boxes
        if boxes is None:
            return []

        masks = None
        if result.masks is not None and result.masks.data is not None:
            masks = result.masks.data.detach().cpu().numpy().astype(bool)

        detections: list[dict[str, Any]] = []
        for idx, box in enumerate(boxes):
            cls = int(box.cls[0].item()) if box.cls is not None else -1
            confidence = float(box.conf[0].item()) if box.conf is not None else 0.0
            label = str(result.names.get(cls, cls)) if isinstance(result.names, dict) else str(cls)
            xmin, ymin, xmax, ymax = [int(round(float(v))) for v in box.xyxy[0].tolist()]
            mask = masks[idx] if masks is not None and idx < masks.shape[0] else None
            detections.append(
                {
                    "object_name": label,
                    "confidence": confidence,
                    "bbox": {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax},
                    "mask": mask,
                }
            )
        return detections


class YOLOEVLMNode(Node):
    """YOLOE-seg RGB-D detector that publishes the Florence-compatible topic contract."""

    def __init__(self):
        super().__init__("sm_yoloe_vlm")
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.cloud_lock = threading.Lock()
        self.objects_lock = threading.Lock()
        self.target_lock = threading.Lock()
        self.latest_frame = None
        self.latest_cloud: PointCloud2 | None = None
        self.latest_objects: list[dict[str, Any]] = []
        self.latest_mask_overlays: list[dict[str, Any]] = []
        self.last_detection_msg = None
        self.last_marker_msg = None
        self.last_roi_cloud_msg = None
        self._stop_event = threading.Event()

        self._declare_parameters()
        self._load_parameters()

        self.detector = YOLOEDetector(
            self.model_path,
            self.device,
            self.confidence_threshold,
            self.iou_threshold,
            self.imgsz,
            self.get_logger(),
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

        self.detection_pub = self.create_publisher(String, self.detections_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.markers_topic, 10)
        self.roi_cloud_pub = self.create_publisher(PointCloud2, self.roi_pointcloud_topic, 10)

        self._setup_subscribers()
        self._inference_thread = threading.Thread(target=self._inference_loop, daemon=True)
        self._inference_thread.start()
        self.get_logger().info(
            f"sm_yoloe_vlm 시작: inference_rate_hz={self.inference_rate_hz}, "
            f"model={self.model_path}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("rgb_topic", "/rsd455/rgb")
        self.declare_parameter("depth_topic", "/rsd455/depth")
        self.declare_parameter("camera_info_topic", "/rsd455/camera_info")
        self.declare_parameter("pointcloud_topic", "")
        self.declare_parameter("target_objects", [])
        self.declare_parameter("roi_target_objects", [])
        self.declare_parameter("target_objects_topic", "/sm_florence_2_vlm/target_objects")
        self.declare_parameter("use_target_objects_topic", True)
        self.declare_parameter("detections_topic", "/sm_florence_2_vlm/detections")
        self.declare_parameter("debug_image_topic", "/sm_florence_2_vlm/debug/image")
        self.declare_parameter("markers_topic", "/sm_florence_2_vlm/markers")
        self.declare_parameter("roi_pointcloud_topic", "/sm_florence_2_vlm/roi_pointcloud")
        self.declare_parameter("model_path", "/home/kiro/Desktop/hw_ws/ros2_ws/src/yoloe-26l/yoloe-26l-seg.pt")
        self.declare_parameter("device", "auto")
        self.declare_parameter("confidence_threshold", 0.05)
        self.declare_parameter("iou_threshold", 0.5)
        self.declare_parameter("imgsz", 960)
        self.declare_parameter("min_color_score", 0.08)
        self.declare_parameter("min_mask_area_px", 40)
        self.declare_parameter("max_mask_area_ratio", 0.22)
        self.declare_parameter("max_bbox_area_ratio", 0.25)
        self.declare_parameter("box_min_mask_area_ratio", 0.015)
        self.declare_parameter("select_best_per_target", True)
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
        self.declare_parameter("roi_depth_segmentation_min_area_px", 10)
        self.declare_parameter("roi_depth_segmentation_kernel_px", 1)
        self.declare_parameter("camera_frame", "rsd455_color_optical_frame")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("tf_use_latest_transform", True)
        self.declare_parameter("inference_rate_hz", 5.0)
        self.declare_parameter("drop_old_frames", True)
        self.declare_parameter("publish_last_detection_when_no_update", False)
        self.declare_parameter("enable_perf_log", True)
        self.declare_parameter("perf_log_interval_sec", 2.0)
        self.declare_parameter("roi_pointcloud_source", "depth")
        self.declare_parameter("roi_pointcloud_require_organized", False)

    def _load_parameters(self) -> None:
        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.pointcloud_topic = self.get_parameter("pointcloud_topic").value
        self.target_objects = self._safe_get_target_objects()
        self.roi_target_objects = self._safe_get_string_list_parameter("roi_target_objects")
        self.target_objects_topic = self.get_parameter("target_objects_topic").value
        self.use_target_objects_topic = bool(self.get_parameter("use_target_objects_topic").value)
        self.detections_topic = self.get_parameter("detections_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.markers_topic = self.get_parameter("markers_topic").value
        self.roi_pointcloud_topic = self.get_parameter("roi_pointcloud_topic").value
        self.model_path = self.get_parameter("model_path").value
        self.device = self.get_parameter("device").value
        self.confidence_threshold = float(self.get_parameter("confidence_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.min_color_score = float(self.get_parameter("min_color_score").value)
        self.min_mask_area_px = int(self.get_parameter("min_mask_area_px").value)
        self.max_mask_area_ratio = float(self.get_parameter("max_mask_area_ratio").value)
        self.max_bbox_area_ratio = float(self.get_parameter("max_bbox_area_ratio").value)
        self.box_min_mask_area_ratio = float(self.get_parameter("box_min_mask_area_ratio").value)
        self.select_best_per_target = bool(self.get_parameter("select_best_per_target").value)
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
        self.roi_depth_segmentation_min_area_px = int(self.get_parameter("roi_depth_segmentation_min_area_px").value)
        self.roi_depth_segmentation_kernel_px = int(self.get_parameter("roi_depth_segmentation_kernel_px").value)
        self.camera_frame = self.get_parameter("camera_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.use_tf_transform = bool(self.get_parameter("use_tf_transform").value)
        self.tf_timeout_sec = float(self.get_parameter("tf_timeout_sec").value)
        self.tf_use_latest_transform = bool(self.get_parameter("tf_use_latest_transform").value)
        self.inference_rate_hz = float(self.get_parameter("inference_rate_hz").value)
        self.drop_old_frames = bool(self.get_parameter("drop_old_frames").value)
        self.publish_last_detection_when_no_update = bool(
            self.get_parameter("publish_last_detection_when_no_update").value
        )
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
        return self._safe_get_string_list_parameter("target_objects")

    def _safe_get_string_list_parameter(self, parameter_name: str) -> list[str]:
        try:
            value = self.get_parameter(parameter_name).value
            if value is None:
                return []
            return [str(x).strip() for x in list(value) if str(x).strip()]
        except ParameterUninitializedException:
            return []

    def _setup_subscribers(self) -> None:
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
        target_objects, roi_target_objects = self._parse_target_payload(msg.data)
        if not target_objects:
            self.get_logger().warn("YOLOE target_objects 입력이 비어 있습니다.")
            return
        with self.target_lock:
            self.target_objects = target_objects
            self.roi_target_objects = roi_target_objects
        roi_log = roi_target_objects or "<all targets>"
        self.get_logger().info(
            f"YOLOE target_objects 갱신: {target_objects}, roi_target_objects={roi_log}"
        )

    @staticmethod
    def _parse_target_objects(text: str) -> list[str]:
        target_objects, _ = YOLOEVLMNode._parse_target_payload(text)
        return target_objects

    @staticmethod
    def _parse_target_payload(text: str) -> tuple[list[str], list[str]]:
        raw = (text or "").strip()
        if not raw:
            return [], []
        values: list[str] = []
        roi_values: list[str] = []
        if raw.startswith("[") or raw.startswith("{"):
            try:
                payload = json.loads(raw)
                if isinstance(payload, list):
                    values = [str(x).strip() for x in payload]
                elif isinstance(payload, dict):
                    vals = payload.get("target_objects", [])
                    if isinstance(vals, list):
                        values = [str(x).strip() for x in vals]
                    roi_vals = payload.get("roi_target_objects", [])
                    if isinstance(roi_vals, list):
                        roi_values = [str(x).strip() for x in roi_vals]
            except Exception:
                values = []
                roi_values = []
        else:
            values = [token.strip() for token in raw.split(",")]
        return [value for value in values if value], [value for value in roi_values if value]

    @staticmethod
    def _is_roi_target_object(object_name: str, roi_target_objects: list[str]) -> bool:
        if not roi_target_objects:
            return True
        normalized_name = str(object_name).strip().lower()
        normalized_targets = {str(target).strip().lower() for target in roi_target_objects}
        return normalized_name in normalized_targets

    def synced_callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo) -> None:
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
                self.get_logger().error(f"YOLOE 추론 처리 실패: {exc}")

            remaining = min_interval - (time.monotonic() - t0)
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

        with self.target_lock:
            current_targets = list(self.target_objects)
            current_roi_targets = list(self.roi_target_objects)
        t_before_detect = time.monotonic()
        raw_detections = self.detector.detect(rgb_bgr, current_targets)
        t_after_detect = time.monotonic()

        objects = []
        roi_points = []
        mask_overlays = []
        object_count_by_name: dict[str, int] = {}
        best_candidate_by_name: dict[str, dict[str, Any]] = {}
        image_h, image_w = rgb_bgr.shape[:2]

        for det in raw_detections:
            bbox = self._clip_bbox(det["bbox"], image_w, image_h)
            raw_object_name = str(det["object_name"])
            global_mask = det.get("mask")
            roi_mask = None
            roi_mask_used = False
            roi_mask_area_px = 0
            if global_mask is not None:
                roi_mask = crop_mask_to_bbox(global_mask, bbox, image_w, image_h)
                roi_mask_area_px = int(np.count_nonzero(roi_mask))
                roi_mask_used = roi_mask_area_px > 0

            if global_mask is not None:
                color_info = score_mask_colors(rgb_bgr, global_mask)
            else:
                roi_bgr = rgb_bgr[bbox["ymin"] : bbox["ymax"] + 1, bbox["xmin"] : bbox["xmax"] + 1]
                color_info = score_mask_colors(roi_bgr, None)
            semantic_match = match_yoloe_detection_to_target(
                raw_object_name,
                current_targets,
                color_info,
                self.min_color_score,
            )
            if semantic_match is None:
                continue
            if self._reject_by_geometry(
                bbox,
                roi_mask_area_px,
                image_w,
                image_h,
                semantic_match.get("semantic_class", ""),
            ):
                continue
            if roi_mask_used:
                mask_overlays.append({"bbox": bbox, "mask": roi_mask.copy()})
            mask_orientation = estimate_mask_orientation(roi_mask if roi_mask_used else None)

            object_name = semantic_match["target_name"]
            roi_target_object = self._is_roi_target_object(object_name, current_roi_targets)

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
            candidate_points = []
            if self.publish_roi_pointcloud:
                if self.roi_pointcloud_source == "depth":
                    points = self.depth_converter.roi_to_camera_points(depth_image, bbox, info_msg, roi_mask)
                    roi_count = len(points)
                    candidate_points.extend(points)
                elif cloud_msg is not None:
                    if self.roi_pointcloud_require_organized and cloud_msg.height <= 1:
                        pass
                    elif roi_mask is not None:
                        points, roi_count = self.roi_filter.extract_points_by_mask(cloud_msg, bbox, roi_mask, info_msg)
                        candidate_points.extend(points)
                    else:
                        points, roi_count = self.roi_filter.extract_points(cloud_msg, bbox, info_msg)
                        candidate_points.extend(points)

            reliability = yoloe_candidate_reliability(
                float(det["confidence"]),
                color_info,
                semantic_match,
                roi_mask_area_px,
                image_w,
                image_h,
            )
            if self.select_best_per_target:
                candidate = {
                    "object_name": object_name,
                    "confidence": float(det["confidence"]),
                    "bbox": bbox,
                    "u": u,
                    "v": v,
                    "camera_point": camera_point,
                    "target_point": target_point,
                    "transform_available": transform_available,
                    "roi_count": roi_count,
                    "roi_mask_used": roi_mask_used,
                    "roi_mask_area_px": roi_mask_area_px,
                    "raw_object_name": raw_object_name,
                    "semantic_match": semantic_match,
                    "color_info": color_info,
                    "mask_orientation": mask_orientation,
                    "points": candidate_points,
                    "roi_target_object": roi_target_object,
                    "overlay": {"bbox": bbox, "mask": roi_mask.copy()} if roi_mask_used else None,
                    "reliability": reliability,
                }
                previous = best_candidate_by_name.get(object_name)
                if previous is None or reliability > float(previous["reliability"]):
                    best_candidate_by_name[object_name] = candidate
            else:
                index = object_count_by_name.get(object_name, 0)
                object_count_by_name[object_name] = index + 1
                object_id = f"{object_name}_{index}"
                objects.append(
                    self._build_detection_object(
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
                        roi_mask_used,
                        roi_mask_area_px,
                        raw_object_name,
                        semantic_match,
                        color_info,
                        mask_orientation,
                    )
                )
                if roi_target_object:
                    roi_points.extend(candidate_points)
        if self.select_best_per_target:
            for object_name, candidate in sorted(
                best_candidate_by_name.items(),
                key=lambda item: float(item[1]["reliability"]),
                reverse=True,
            ):
                object_id = f"{object_name}_0"
                objects.append(
                    self._build_detection_object(
                        object_id,
                        candidate["object_name"],
                        float(candidate["confidence"]),
                        candidate["bbox"],
                        int(candidate["u"]),
                        int(candidate["v"]),
                        candidate["camera_point"],
                        candidate["target_point"],
                        bool(candidate["transform_available"]),
                        int(candidate["roi_count"]),
                        bool(candidate["roi_mask_used"]),
                        int(candidate["roi_mask_area_px"]),
                        str(candidate["raw_object_name"]),
                        candidate["semantic_match"],
                        candidate["color_info"],
                        candidate["mask_orientation"],
                    )
                )
                if bool(candidate["roi_target_object"]):
                    roi_points.extend(candidate["points"])
                if candidate["overlay"] is not None:
                    mask_overlays.append(candidate["overlay"])
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

    def _reject_by_geometry(
        self,
        bbox: dict[str, int],
        roi_mask_area_px: int,
        image_w: int,
        image_h: int,
        semantic_class: str,
    ) -> bool:
        return not is_yoloe_geometry_valid(
            bbox,
            roi_mask_area_px,
            image_w,
            image_h,
            semantic_class=semantic_class,
            min_mask_area_px=self.min_mask_area_px,
            max_mask_area_ratio=self.max_mask_area_ratio,
            max_bbox_area_ratio=self.max_bbox_area_ratio,
            box_min_mask_area_ratio=self.box_min_mask_area_ratio,
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
        msg = String(data=json.dumps(payload, ensure_ascii=False))
        self.detection_pub.publish(msg)
        self.last_detection_msg = msg

        if self.publish_markers:
            marker_msg = self.visualizer.make_markers(objects, self.target_frame, rgb_msg.header.stamp)
            self.marker_pub.publish(marker_msg)
            self.last_marker_msg = marker_msg

        if self.publish_roi_pointcloud and roi_cloud is not None:
            self.roi_cloud_pub.publish(roi_cloud)
            self.last_roi_cloud_msg = roi_cloud

    def _publish_last_results(self) -> None:
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
        camera_point,
        target_point,
        transform_available: bool,
        roi_count: int,
        roi_mask_used: bool,
        roi_mask_area_px: int,
        raw_object_name: str,
        semantic_match: dict[str, Any],
        color_info: dict[str, Any],
        mask_orientation: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        camera_pos = self._point_to_dict(camera_point, self.camera_frame)
        target_pos = self._point_to_dict(target_point, self.target_frame) if target_point else None
        obj = {
            "object_id": object_id,
            "object_name": object_name,
            "raw_object_name": raw_object_name,
            "semantic_class": semantic_match.get("semantic_class", ""),
            "requested_color": semantic_match.get("requested_color") or "",
            "observed_color": color_info.get("observed_color", ""),
            "color_scores": color_info.get("scores", {}),
            "confidence": confidence,
            "bbox": bbox,
            "center_pixel": {"u": u, "v": v},
            "position_camera_frame": camera_pos,
            "position_target_frame": target_pos,
            "transform_available": transform_available,
            "roi_point_count": roi_count,
            "roi_geometry_mode": "yoloe_mask" if roi_mask_used else "bbox",
            "roi_mask_used": bool(roi_mask_used),
            "roi_mask_area_px": int(roi_mask_area_px),
        }
        orientation = self._orientation_to_dict(mask_orientation, self.camera_frame)
        if orientation is not None:
            obj["orientation_camera_frame"] = orientation
        return obj

    @staticmethod
    def _orientation_to_dict(
        orientation: dict[str, Any] | None,
        fallback_frame: str,
    ) -> dict[str, Any] | None:
        if not orientation:
            return None
        quaternion = orientation.get("quaternion_xyzw")
        if not isinstance(quaternion, (list, tuple)) or len(quaternion) != 4:
            return None
        return {
            "frame_id": fallback_frame,
            "x": float(quaternion[0]),
            "y": float(quaternion[1]),
            "z": float(quaternion[2]),
            "w": float(quaternion[3]),
            "angle_rad": float(orientation.get("angle_rad", 0.0)),
            "confidence": float(orientation.get("confidence", 0.0)),
        }

    @staticmethod
    def _point_to_dict(point_msg, fallback_frame: str) -> dict[str, Any]:
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
    node = YOLOEVLMNode()
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
