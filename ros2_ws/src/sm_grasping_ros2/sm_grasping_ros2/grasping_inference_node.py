#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
import time

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_geometry_msgs  # noqa: F401
import tf2_ros
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, String

from .grasp_filters import EmaFilterState, apply_ema, clamp_translation_step, normalize_quaternion_xyzw
from .grasp_postprocess import filter_candidates
from .gpd_wrapper import GpdWrapper


def sanitize_candidate_openings(
    candidate_openings: list[float],
    max_opening_m: float,
) -> tuple[list[float], list[float]]:
    sanitized: list[float] = []
    oversized: list[float] = []
    max_opening = max(0.0, float(max_opening_m))
    for opening in candidate_openings:
        value = float(opening)
        if value > max_opening:
            oversized.append(value)
            sanitized.append(max_opening)
        else:
            sanitized.append(value)
    return sanitized, oversized


def apply_radial_xy_offset(position: np.ndarray, offset_m: float) -> np.ndarray:
    """Offset a target along its chassis-frame radial direction."""
    adjusted = np.asarray(position, dtype=float).copy()
    radius = float(np.linalg.norm(adjusted[:2]))
    if radius <= 1e-9 or abs(float(offset_m)) <= 1e-12:
        return adjusted
    adjusted[:2] += float(offset_m) * adjusted[:2] / radius
    return adjusted


class AnyGraspInferenceNode(Node):
    """ROI 포인트클라우드 기반 GPD 추론 노드."""

    def __init__(self):
        super().__init__("sm_grasping_inference")
        self._declare_parameters()
        self._lock = threading.Lock()
        self._latest_cloud: PointCloud2 | None = None
        self._is_inferencing = False
        self._ema_state = EmaFilterState()
        self._target_key: tuple[str, ...] = ()
        self._accept_cloud_after_ns = 0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_candidates = self.create_publisher(PoseArray, self.p_output_grasp_candidates_topic, 10)
        self._pub_scores = self.create_publisher(Float32MultiArray, self.p_output_grasp_scores_topic, 10)
        self._pub_openings = self.create_publisher(Float32MultiArray, self.p_output_grasp_openings_topic, 10)
        self._pub_best = self.create_publisher(PoseStamped, self.p_output_grasp_best_topic, 10)
        self._pub_debug = self.create_publisher(String, self.p_output_grasp_debug_topic, 10)

        self.create_subscription(
            PointCloud2,
            self.p_input_roi_pointcloud_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            self.p_target_objects_topic,
            self._target_objects_callback,
            10,
        )

        self._model_ready = True
        self._device = "cpu"
        self._device_status = "OK_GPD"
        self._wrapper = GpdWrapper(
            logger=self.get_logger(),
            default_opening_m=self.p_gpd_default_opening_m,
            lateral_backoff_m=self.p_heuristic_lateral_backoff_m,
        )
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(1.0 / max(0.1, self.p_inference_rate_hz), self._on_timer)

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_roi_pointcloud_topic", "/sm_florence_2_vlm/roi_pointcloud")
        self.declare_parameter("target_objects_topic", "/sm_florence_2_vlm/target_objects")
        self.declare_parameter("target_change_settle_sec", 0.40)
        self.declare_parameter("output_grasp_candidates_topic", "/sm_grasping/grasp_candidates")
        self.declare_parameter("output_grasp_scores_topic", "/sm_grasping/grasp_scores")
        self.declare_parameter("output_grasp_openings_topic", "/sm_grasping/grasp_openings")
        self.declare_parameter("output_grasp_best_topic", "/sm_grasping/grasp_best")
        self.declare_parameter("output_grasp_debug_topic", "/sm_grasping/grasp_debug")
        self.declare_parameter("use_tf_transform", True)
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("inference_rate_hz", 7.0)
        self.declare_parameter("max_candidates", 20)
        self.declare_parameter("min_grasp_score", 0.2)
        self.declare_parameter("voxel_size", 0.005)
        self.declare_parameter("workspace.x_min", -0.5)
        self.declare_parameter("workspace.x_max", 0.8)
        self.declare_parameter("workspace.y_min", -0.6)
        self.declare_parameter("workspace.y_max", 0.6)
        self.declare_parameter("workspace.z_min", 0.0)
        self.declare_parameter("workspace.z_max", 1.5)
        self.declare_parameter("gpd_num_samples", 400)
        self.declare_parameter("gpd_num_threads", 4)
        self.declare_parameter("gpd_default_opening_m", 0.04)
        self.declare_parameter("heuristic_lateral_backoff_m", 0.0)
        self.declare_parameter("gripper.opening_margin_m", 0.005)
        self.declare_parameter("gripper.opening_width_percentile_low", 5.0)
        self.declare_parameter("gripper.opening_width_percentile_high", 95.0)
        self.declare_parameter("gpd_force_top_down_orientation", False)
        self.declare_parameter("gpd_top_down_quaternion_xyzw", [0.0, -0.70710678, 0.0, 0.70710678])
        self.declare_parameter("gpd_to_robot_quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("grasp_z_offset_m", 0.0)
        self.declare_parameter("gpd_additional_rpy_deg", [0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.declare_parameter("base_offset_xyz_m", [0.0, 0.0, 0.0])  # target_frame(base_link) 기준
        self.declare_parameter("grasp_target_radial_offset_m", 0.0)
        self.declare_parameter("base_additional_rpy_deg", [0.0, 0.0, 0.0])  # target_frame(base_link) 기준
        self.declare_parameter("target_frame_yaw_only", True)  # base_link 기준 객체 yaw만 추종
        self.declare_parameter("target_frame_apply_fixed_roll_pitch", True)  # yaw_only에서 고정 roll/pitch 보정 적용
        self.declare_parameter("position_ema_alpha", 0.6)
        self.declare_parameter("orientation_ema_alpha", 0.5)
        self.declare_parameter("max_translation_step_m", 0.05)
        self.declare_parameter("gripper.min_opening_m", 0.01)
        self.declare_parameter("gripper.max_opening_m", 0.09)

        gp = self.get_parameter
        self.p_input_roi_pointcloud_topic = gp("input_roi_pointcloud_topic").value
        self.p_target_objects_topic = str(gp("target_objects_topic").value)
        self.p_target_change_settle_sec = max(
            0.0, float(gp("target_change_settle_sec").value)
        )
        self.p_output_grasp_candidates_topic = gp("output_grasp_candidates_topic").value
        self.p_output_grasp_scores_topic = gp("output_grasp_scores_topic").value
        self.p_output_grasp_openings_topic = gp("output_grasp_openings_topic").value
        self.p_output_grasp_best_topic = gp("output_grasp_best_topic").value
        self.p_output_grasp_debug_topic = gp("output_grasp_debug_topic").value
        self.p_use_tf_transform = bool(gp("use_tf_transform").value)
        self.p_target_frame = gp("target_frame").value
        self.p_tf_timeout_sec = float(gp("tf_timeout_sec").value)
        self.p_inference_rate_hz = float(gp("inference_rate_hz").value)
        self.p_max_candidates = int(gp("max_candidates").value)
        self.p_min_grasp_score = float(gp("min_grasp_score").value)
        self.p_workspace = {
            "x_min": float(gp("workspace.x_min").value),
            "x_max": float(gp("workspace.x_max").value),
            "y_min": float(gp("workspace.y_min").value),
            "y_max": float(gp("workspace.y_max").value),
            "z_min": float(gp("workspace.z_min").value),
            "z_max": float(gp("workspace.z_max").value),
        }
        self.p_gpd_num_samples = int(gp("gpd_num_samples").value)
        self.p_gpd_num_threads = int(gp("gpd_num_threads").value)
        self.p_gpd_default_opening_m = float(gp("gpd_default_opening_m").value)
        self.p_heuristic_lateral_backoff_m = float(
            gp("heuristic_lateral_backoff_m").value
        )
        self.p_opening_margin_m = float(gp("gripper.opening_margin_m").value)
        self.p_opening_width_percentile_low = float(gp("gripper.opening_width_percentile_low").value)
        self.p_opening_width_percentile_high = float(gp("gripper.opening_width_percentile_high").value)
        self.p_gpd_force_top_down_orientation = bool(gp("gpd_force_top_down_orientation").value)
        self.p_gpd_top_down_quaternion = np.array(gp("gpd_top_down_quaternion_xyzw").value, dtype=float)
        self.p_gpd_to_robot_quaternion = np.array(gp("gpd_to_robot_quaternion_xyzw").value, dtype=float)
        self.p_grasp_z_offset_m = float(gp("grasp_z_offset_m").value)
        self.p_gpd_additional_rpy_deg = np.array(gp("gpd_additional_rpy_deg").value, dtype=float)
        self.p_base_offset_xyz_m = np.array(gp("base_offset_xyz_m").value, dtype=float)
        self.p_grasp_target_radial_offset_m = float(gp("grasp_target_radial_offset_m").value)
        self.p_base_additional_rpy_deg = np.array(gp("base_additional_rpy_deg").value, dtype=float)
        self.p_target_frame_yaw_only = bool(gp("target_frame_yaw_only").value)
        self.p_target_frame_apply_fixed_roll_pitch = bool(gp("target_frame_apply_fixed_roll_pitch").value)
        self.p_position_ema_alpha = float(gp("position_ema_alpha").value)
        self.p_orientation_ema_alpha = float(gp("orientation_ema_alpha").value)
        self.p_max_translation_step_m = float(gp("max_translation_step_m").value)
        self.p_min_opening_m = float(gp("gripper.min_opening_m").value)
        self.p_max_opening_m = float(gp("gripper.max_opening_m").value)

    def _cloud_callback(self, msg: PointCloud2) -> None:
        if self.get_clock().now().nanoseconds < self._accept_cloud_after_ns:
            return
        with self._lock:
            self._latest_cloud = msg

    def _target_objects_callback(self, msg: String) -> None:
        target_key = self._parse_roi_target_key(msg.data)
        if not target_key or target_key == self._target_key:
            return
        self._target_key = target_key
        self._accept_cloud_after_ns = self.get_clock().now().nanoseconds + int(
            self.p_target_change_settle_sec * 1e9
        )
        with self._lock:
            self._latest_cloud = None
        self._ema_state = EmaFilterState()
        self._wrapper.reset_tracking_state()
        self.get_logger().warn(
            f"Grasp target changed to {list(target_key)}; cleared stale ROI/EMA state"
        )

    @staticmethod
    def _parse_roi_target_key(text: str) -> tuple[str, ...]:
        raw = str(text or "").strip()
        if not raw:
            return ()
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return tuple(token.strip().lower() for token in raw.split(",") if token.strip())
        if isinstance(payload, dict):
            values = payload.get("roi_target_objects", [])
            if not isinstance(values, list) or not values:
                values = payload.get("target_objects", [])
        elif isinstance(payload, list):
            values = payload
        else:
            values = []
        return tuple(str(value).strip().lower() for value in values if str(value).strip())

    def _take_latest_cloud(self) -> PointCloud2 | None:
        with self._lock:
            cloud = self._latest_cloud
            self._latest_cloud = None
        return cloud

    def _on_set_parameters(self, params):
        """런타임 파라미터 변경을 내부 변수에 즉시 반영한다."""
        try:
            for p in params:
                if p.name == "gpd_force_top_down_orientation":
                    self.p_gpd_force_top_down_orientation = bool(p.value)
                elif p.name == "gpd_top_down_quaternion_xyzw":
                    self.p_gpd_top_down_quaternion = np.array(p.value, dtype=float)
                elif p.name == "gpd_to_robot_quaternion_xyzw":
                    self.p_gpd_to_robot_quaternion = np.array(p.value, dtype=float)
                elif p.name == "grasp_z_offset_m":
                    self.p_grasp_z_offset_m = float(p.value)
                elif p.name == "heuristic_lateral_backoff_m":
                    self.p_heuristic_lateral_backoff_m = float(p.value)
                    self._wrapper.lateral_backoff_m = self.p_heuristic_lateral_backoff_m
                elif p.name == "gpd_additional_rpy_deg":
                    self.p_gpd_additional_rpy_deg = np.array(p.value, dtype=float)
                elif p.name == "base_offset_xyz_m":
                    self.p_base_offset_xyz_m = np.array(p.value, dtype=float)
                elif p.name == "grasp_target_radial_offset_m":
                    self.p_grasp_target_radial_offset_m = float(p.value)
                elif p.name == "base_additional_rpy_deg":
                    self.p_base_additional_rpy_deg = np.array(p.value, dtype=float)
                elif p.name == "target_frame_yaw_only":
                    self.p_target_frame_yaw_only = bool(p.value)
                elif p.name == "target_frame_apply_fixed_roll_pitch":
                    self.p_target_frame_apply_fixed_roll_pitch = bool(p.value)
                elif p.name == "position_ema_alpha":
                    self.p_position_ema_alpha = float(p.value)
                elif p.name == "orientation_ema_alpha":
                    self.p_orientation_ema_alpha = float(p.value)
                elif p.name == "max_translation_step_m":
                    self.p_max_translation_step_m = float(p.value)
                elif p.name == "gripper.opening_margin_m":
                    self.p_opening_margin_m = float(p.value)
                elif p.name == "gripper.opening_width_percentile_low":
                    self.p_opening_width_percentile_low = float(p.value)
                elif p.name == "gripper.opening_width_percentile_high":
                    self.p_opening_width_percentile_high = float(p.value)
                elif p.name == "gripper.min_opening_m":
                    self.p_min_opening_m = float(p.value)
                elif p.name == "gripper.max_opening_m":
                    self.p_max_opening_m = float(p.value)
            self.get_logger().info(
                "파라미터 갱신 반영: "
                f"force_top_down={self.p_gpd_force_top_down_orientation}, "
                f"q_top_down={self.p_gpd_top_down_quaternion.tolist()}, "
                f"q_gpd_to_robot={self.p_gpd_to_robot_quaternion.tolist()}, "
                f"z_offset={self.p_grasp_z_offset_m}, "
                f"lateral_backoff={self.p_heuristic_lateral_backoff_m}, "
                f"rpy_deg={self.p_gpd_additional_rpy_deg.tolist()}, "
                f"base_offset={self.p_base_offset_xyz_m.tolist()}, "
                f"radial_offset={self.p_grasp_target_radial_offset_m}, "
                f"base_rpy_deg={self.p_base_additional_rpy_deg.tolist()}"
            )
            return SetParametersResult(successful=True)
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

    def _on_timer(self) -> None:
        if self._is_inferencing:
            return

        cloud = self._take_latest_cloud()

        if cloud is None:
            return

        self._is_inferencing = True
        started = time.time()
        try:
            if not self._model_ready:
                self._publish_empty_grasp_result(cloud)
                self._publish_debug("ERR_MODEL_LOAD", "모델 준비가 완료되지 않았습니다.", 0, 0.0, 0.0)
                return

            if not self._wrapper.is_ready:
                reason = self._wrapper.load_error or "GPD 백엔드가 준비되지 않았습니다."
                self._publish_empty_grasp_result(cloud)
                self._publish_debug("ERR_MODEL_LOAD", reason, 0, 0.0, 0.0)
                return

            points = self._pointcloud_to_xyz(cloud)
            if points.shape[0] == 0:
                self._publish_empty_grasp_result(cloud)
                self._publish_debug("ERR_INSUFFICIENT_POINTS", "유효 포인트가 없습니다.", 0, 0.0, 0.0)
                return

            lims = [
                self.p_workspace["x_min"],
                self.p_workspace["x_max"],
                self.p_workspace["y_min"],
                self.p_workspace["y_max"],
                self.p_workspace["z_min"],
                self.p_workspace["z_max"],
            ]
            candidates = self._wrapper.infer(
                points,
                self.p_max_candidates,
                lims=lims,
            )
            candidates = filter_candidates(candidates, self.p_min_grasp_score)
            if not candidates:
                self._publish_empty_grasp_result(cloud)
                self._publish_debug(self._device_status, "유효한 grasp 후보가 없습니다.", 0, 0.0, 0.0)
                return

            pose_array = PoseArray()
            pose_array.header = cloud.header
            score_msg = Float32MultiArray()
            score_msg.data = []
            opening_msg = Float32MultiArray()
            opening_msg.data = []
            raw_candidate_openings = [
                self._estimate_opening_from_roi(points, item.quaternion_xyzw)
                for item in candidates
            ]
            candidate_openings, oversized_openings = sanitize_candidate_openings(
                raw_candidate_openings,
                self.p_max_opening_m,
            )
            if oversized_openings:
                self.get_logger().warn(
                    "ROI 폭 추정이 그리퍼 최대 벌림보다 커 opening만 clamp합니다: "
                    f"required={max(oversized_openings):.3f}m, max={self.p_max_opening_m:.3f}m"
                )

            for item, adaptive_opening in zip(candidates, candidate_openings):
                pose = Pose()
                pose.position.x = float(item.translation[0])
                pose.position.y = float(item.translation[1])
                pose.position.z = float(item.translation[2] + self.p_grasp_z_offset_m)
                q = normalize_quaternion_xyzw(item.quaternion_xyzw)
                pose.orientation.x = float(q[0])
                pose.orientation.y = float(q[1])
                pose.orientation.z = float(q[2])
                pose.orientation.w = float(q[3])
                if self.p_gpd_force_top_down_orientation:
                    tq = normalize_quaternion_xyzw(self.p_gpd_top_down_quaternion)
                    pose.orientation.x = float(tq[0])
                    pose.orientation.y = float(tq[1])
                    pose.orientation.z = float(tq[2])
                    pose.orientation.w = float(tq[3])
                g2r = normalize_quaternion_xyzw(self.p_gpd_to_robot_quaternion)
                q_pose = np.array(
                    [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=float
                )
                q_corr = _quat_multiply_xyzw(q_pose, g2r)
                q_rpy = _rpy_deg_to_quat_xyzw(self.p_gpd_additional_rpy_deg)
                q_corr = _quat_multiply_xyzw(q_corr, q_rpy)
                q_corr = normalize_quaternion_xyzw(q_corr)
                pose.orientation.x = float(q_corr[0])
                pose.orientation.y = float(q_corr[1])
                pose.orientation.z = float(q_corr[2])
                pose.orientation.w = float(q_corr[3])
                pose_array.poses.append(pose)
                score_msg.data.append(float(item.score))
                opening = float(max(adaptive_opening, self.p_min_opening_m))
                opening_msg.data.append(opening)

            best = PoseStamped()
            best.header = cloud.header
            best.pose = pose_array.poses[0]

            prev_pos = None if self._ema_state.position is None else self._ema_state.position.copy()
            filtered_pos, filtered_q = apply_ema(
                self._ema_state,
                np.array([best.pose.position.x, best.pose.position.y, best.pose.position.z], dtype=float),
                np.array(
                    [
                        best.pose.orientation.x,
                        best.pose.orientation.y,
                        best.pose.orientation.z,
                        best.pose.orientation.w,
                    ],
                    dtype=float,
                ),
                self.p_position_ema_alpha,
                self.p_orientation_ema_alpha,
            )
            if prev_pos is not None:
                filtered_pos = clamp_translation_step(
                    prev_pos,
                    filtered_pos,
                    self.p_max_translation_step_m,
                )

            best.pose.position.x = float(filtered_pos[0])
            best.pose.position.y = float(filtered_pos[1])
            best.pose.position.z = float(filtered_pos[2])
            best.pose.orientation.x = float(filtered_q[0])
            best.pose.orientation.y = float(filtered_q[1])
            best.pose.orientation.z = float(filtered_q[2])
            best.pose.orientation.w = float(filtered_q[3])

            if self.p_gpd_force_top_down_orientation:
                tq = normalize_quaternion_xyzw(self.p_gpd_top_down_quaternion)
                best.pose.orientation.x = float(tq[0])
                best.pose.orientation.y = float(tq[1])
                best.pose.orientation.z = float(tq[2])
                best.pose.orientation.w = float(tq[3])

            if self.p_use_tf_transform and self.p_target_frame and best.header.frame_id != self.p_target_frame:
                try:
                    transformed = self._tf_buffer.transform(
                        best,
                        self.p_target_frame,
                        timeout=Duration(seconds=self.p_tf_timeout_sec),
                    )
                    best = transformed
                except Exception as exc:
                    self._publish_empty_grasp_result(cloud)
                    self._publish_debug("ERR_TF_TRANSFORM", f"TF 변환 실패: {exc}", len(candidates), score_msg.data[0], 0.0)
                    return
            # base_link(target_frame) 기준 위치/회전 보정
            if best.header.frame_id == self.p_target_frame:
                corrected_position = apply_radial_xy_offset(
                    np.array(
                        [
                            best.pose.position.x,
                            best.pose.position.y,
                            best.pose.position.z,
                        ],
                        dtype=float,
                    ),
                    self.p_grasp_target_radial_offset_m,
                )
                best.pose.position.x = float(corrected_position[0])
                best.pose.position.y = float(corrected_position[1])
                best.pose.position.x += float(self.p_base_offset_xyz_m[0])
                best.pose.position.y += float(self.p_base_offset_xyz_m[1])
                best.pose.position.z += float(self.p_base_offset_xyz_m[2])

                q_pose = np.array(
                    [
                        best.pose.orientation.x,
                        best.pose.orientation.y,
                        best.pose.orientation.z,
                        best.pose.orientation.w,
                    ],
                    dtype=float,
                )
                if self.p_target_frame_yaw_only:
                    # base_link 기준 roll/pitch 흔들림을 억제하고 yaw만 추종한다.
                    yaw_obj = _quat_to_yaw_rad(q_pose)
                    if self.p_target_frame_apply_fixed_roll_pitch:
                        roll_fix = float(self.p_base_additional_rpy_deg[0])
                        pitch_fix = float(self.p_base_additional_rpy_deg[1])
                    else:
                        roll_fix = 0.0
                        pitch_fix = 0.0
                    yaw_fix = float(self.p_base_additional_rpy_deg[2])
                    q_new = _rpy_deg_to_quat_xyzw(np.array([roll_fix, pitch_fix, np.rad2deg(yaw_obj) + yaw_fix]))
                    q_new = normalize_quaternion_xyzw(q_new)
                else:
                    q_base = _rpy_deg_to_quat_xyzw(self.p_base_additional_rpy_deg)
                    # base 기준 전역 회전은 pre-multiply(q_base * q_pose)
                    q_new = _quat_multiply_xyzw(q_base, q_pose)
                    q_new = normalize_quaternion_xyzw(q_new)
                best.pose.orientation.x = float(q_new[0])
                best.pose.orientation.y = float(q_new[1])
                best.pose.orientation.z = float(q_new[2])
                best.pose.orientation.w = float(q_new[3])

            self._pub_candidates.publish(pose_array)
            self._pub_scores.publish(score_msg)
            self._pub_openings.publish(opening_msg)
            self._pub_best.publish(best)

            latency_ms = (time.time() - started) * 1000.0
            self._publish_debug(self._device_status, "정상 추론", len(candidates), score_msg.data[0], latency_ms)
        except Exception as exc:
            self.get_logger().exception("추론 중 예외가 발생했습니다.")
            self._publish_empty_grasp_result(cloud)
            self._publish_debug("ERR_INFERENCE_RUNTIME", str(exc), 0, 0.0, 0.0)
        finally:
            self._is_inferencing = False

    def _publish_empty_grasp_result(self, cloud: PointCloud2) -> None:
        """이전 grasp 결과가 남지 않도록 빈 후보/점수/오프닝을 발행합니다."""
        empty_candidates = PoseArray()
        empty_candidates.header = cloud.header
        self._pub_candidates.publish(empty_candidates)
        self._pub_scores.publish(Float32MultiArray(data=[]))
        self._pub_openings.publish(Float32MultiArray(data=[]))
        # 후보가 사라진 시점에서 EMA 상태도 리셋해 다음 유효 검출에 빠르게 수렴시킵니다.
        self._ema_state = EmaFilterState()

    def _pointcloud_to_xyz(self, cloud: PointCloud2) -> np.ndarray:
        rows = []
        for p in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            if (
                self.p_workspace["x_min"] <= x <= self.p_workspace["x_max"]
                and self.p_workspace["y_min"] <= y <= self.p_workspace["y_max"]
                and self.p_workspace["z_min"] <= z <= self.p_workspace["z_max"]
            ):
                rows.append([x, y, z])
        if not rows:
            return np.empty((0, 3), dtype=float)
        return np.asarray(rows, dtype=float)

    def _estimate_opening_from_roi(self, points_xyz: np.ndarray, quaternion_xyzw: np.ndarray) -> float:
        """그리퍼가 닫히는 로컬 Y축 기준 객체 폭을 추정해 파지 폭으로 사용한다."""
        if points_xyz.shape[0] < 10:
            return max(self.p_gpd_default_opening_m, self.p_min_opening_m)
        q = normalize_quaternion_xyzw(quaternion_xyzw)
        closing_axis = _rotate_vec_by_quat(np.array([0.0, 1.0, 0.0], dtype=float), q)
        closing_axis[2] = 0.0
        norm = float(np.linalg.norm(closing_axis))
        if norm < 1e-6:
            closing_axis = np.array([0.0, 1.0, 0.0], dtype=float)
        else:
            closing_axis = closing_axis / norm
        projected = points_xyz[:, :3].astype(float) @ closing_axis
        pct_low = float(np.clip(self.p_opening_width_percentile_low, 0.0, 49.0))
        pct_high = float(np.clip(self.p_opening_width_percentile_high, 51.0, 100.0))
        low = float(np.percentile(projected, pct_low))
        high = float(np.percentile(projected, pct_high))
        width_est = max(0.0, high - low)
        opening = width_est + max(0.0, self.p_opening_margin_m)
        return float(max(opening, self.p_min_opening_m))

    def _publish_debug(self, status_code: str, message: str, candidate_count: int, best_score: float, latency_ms: float) -> None:
        payload = {
            "status_code": status_code,
            "message": message,
            "candidate_count": int(candidate_count),
            "best_score": float(best_score),
            "latency_ms": float(latency_ms),
            "stamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_debug.publish(msg)


def _quat_multiply_xyzw(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """쿼터니언 곱셈 (XYZW)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return np.array([x, y, z, w], dtype=float)


def _rotate_vec_by_quat(vec: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """벡터를 XYZW 쿼터니언으로 회전한다."""
    q_vec = np.array([float(vec[0]), float(vec[1]), float(vec[2]), 0.0], dtype=float)
    q_conj = np.array(
        [-float(quat_xyzw[0]), -float(quat_xyzw[1]), -float(quat_xyzw[2]), float(quat_xyzw[3])],
        dtype=float,
    )
    return _quat_multiply_xyzw(_quat_multiply_xyzw(quat_xyzw, q_vec), q_conj)[:3]


def _rpy_deg_to_quat_xyzw(rpy_deg: np.ndarray) -> np.ndarray:
    """RPY(deg)를 XYZW 쿼터니언으로 변환한다."""
    roll = np.deg2rad(float(rpy_deg[0]))
    pitch = np.deg2rad(float(rpy_deg[1]))
    yaw = np.deg2rad(float(rpy_deg[2]))

    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return np.array([x, y, z, w], dtype=float)


def _quat_to_yaw_rad(q: np.ndarray) -> float:
    """XYZW 쿼터니언에서 yaw(rad)만 추출한다."""
    x, y, z, w = [float(v) for v in q]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def main(args=None):
    rclpy.init(args=args)
    node = AnyGraspInferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
