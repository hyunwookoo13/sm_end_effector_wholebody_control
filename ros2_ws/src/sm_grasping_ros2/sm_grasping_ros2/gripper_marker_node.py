#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from copy import deepcopy

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class GripperMarkerNode(Node):
    """grasp 포즈를 그리퍼 모양 MarkerArray로 변환한다."""

    def __init__(self):
        super().__init__("sm_gripper_marker")
        self._declare_parameters()

        self._latest_candidates: PoseArray | None = None
        self._latest_best: PoseStamped | None = None
        self._latest_openings: list[float] = []
        self._last_tf_warn_ns = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(PoseArray, self.p_input_candidates_topic, self._on_candidates, 10)
        self.create_subscription(PoseStamped, self.p_input_best_topic, self._on_best, 10)
        self.create_subscription(Float32MultiArray, self.p_input_openings_topic, self._on_openings, 10)
        self._pub_markers = self.create_publisher(MarkerArray, self.p_output_markers_topic, 10)
        self.create_timer(0.1, self._on_timer)

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_grasp_candidates_topic", "/sm_grasping/grasp_candidates")
        self.declare_parameter("input_grasp_best_topic", "/sm_grasping/grasp_best")
        self.declare_parameter("input_grasp_openings_topic", "/sm_grasping/grasp_openings")
        self.declare_parameter("output_gripper_markers_topic", "/sm_grasping/gripper_markers")
        self.declare_parameter("output_frame", "")
        self.declare_parameter("tf_timeout_sec", 0.05)
        self.declare_parameter("gripper.finger_length_m", 0.06)
        self.declare_parameter("gripper.finger_width_m", 0.01)
        self.declare_parameter("gripper.finger_thickness_m", 0.008)
        self.declare_parameter("gripper.palm_width_m", 0.05)
        self.declare_parameter("gripper.default_opening_m", 0.04)
        self.declare_parameter("marker.best_color_rgba", [0.1, 0.9, 0.2, 0.95])
        self.declare_parameter("marker.candidate_color_rgba", [0.2, 0.6, 1.0, 0.35])
        self.declare_parameter("marker.lifetime_sec", 0.3)
        self.declare_parameter("marker.show_candidates", True)
        self.declare_parameter("marker.show_best_only", False)
        self.declare_parameter("marker.show_grasp_point", True)
        self.declare_parameter("marker.grasp_point_radius_m", 0.025)
        self.declare_parameter("marker.grasp_point_color_rgba", [1.0, 0.25, 0.05, 1.0])
        self.declare_parameter("marker.show_axes", True)
        self.declare_parameter("marker.axes_length_m", 0.08)
        self.declare_parameter("marker.axes_width_m", 0.006)

        gp = self.get_parameter
        self.p_input_candidates_topic = gp("input_grasp_candidates_topic").value
        self.p_input_best_topic = gp("input_grasp_best_topic").value
        self.p_input_openings_topic = gp("input_grasp_openings_topic").value
        self.p_output_markers_topic = gp("output_gripper_markers_topic").value
        self.p_output_frame = str(gp("output_frame").value).strip()
        self.p_tf_timeout_sec = float(gp("tf_timeout_sec").value)
        self.p_finger_length_m = float(gp("gripper.finger_length_m").value)
        self.p_finger_width_m = float(gp("gripper.finger_width_m").value)
        self.p_finger_thickness_m = float(gp("gripper.finger_thickness_m").value)
        self.p_palm_width_m = float(gp("gripper.palm_width_m").value)
        self.p_default_opening_m = float(gp("gripper.default_opening_m").value)
        self.p_best_color = self._to_color(gp("marker.best_color_rgba").value)
        self.p_candidate_color = self._to_color(gp("marker.candidate_color_rgba").value)
        self.p_lifetime_sec = float(gp("marker.lifetime_sec").value)
        self.p_show_candidates = bool(gp("marker.show_candidates").value)
        self.p_show_best_only = bool(gp("marker.show_best_only").value)
        self.p_show_grasp_point = bool(gp("marker.show_grasp_point").value)
        self.p_grasp_point_radius_m = float(gp("marker.grasp_point_radius_m").value)
        self.p_grasp_point_color = self._to_color(gp("marker.grasp_point_color_rgba").value)
        self.p_show_axes = bool(gp("marker.show_axes").value)
        self.p_axes_length_m = float(gp("marker.axes_length_m").value)
        self.p_axes_width_m = float(gp("marker.axes_width_m").value)

    @staticmethod
    def _to_color(values) -> ColorRGBA:
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = [float(v) for v in values]
        return c

    def _on_candidates(self, msg: PoseArray) -> None:
        self._latest_candidates = msg
        if not msg.poses:
            # 후보가 비면 이전 best 잔상을 제거합니다.
            self._latest_best = None
            self._latest_openings = []

    def _on_best(self, msg: PoseStamped) -> None:
        self._latest_best = msg

    def _on_openings(self, msg: Float32MultiArray) -> None:
        self._latest_openings = [float(v) for v in msg.data]

    def _on_timer(self) -> None:
        marker_array = MarkerArray()
        marker_id = 0

        if self._latest_best is not None:
            best_pose, frame_id = self._pose_in_output_frame(
                self._latest_best.pose,
                self._latest_best.header.frame_id,
            )
            if best_pose is None:
                return
            opening_best = self._latest_openings[0] if self._latest_openings else self.p_default_opening_m
            if self.p_show_grasp_point:
                marker_id = self._append_grasp_point_marker(marker_array, frame_id, best_pose, "best_grasp_point", marker_id)
            marker_id = self._append_gripper_markers(
                marker_array,
                frame_id,
                best_pose,
                self.p_best_color,
                "best",
                marker_id,
                opening_best,
            )
            if self.p_show_axes:
                marker_id = self._append_axes_markers(marker_array, frame_id, best_pose, "best_axes", marker_id)

        if (
            not self.p_show_best_only
            and self.p_show_candidates
            and self._latest_candidates is not None
            and self._latest_candidates.poses
        ):
            source_frame_id = self._latest_candidates.header.frame_id
            for idx, pose in enumerate(self._latest_candidates.poses):
                marker_pose, frame_id = self._pose_in_output_frame(pose, source_frame_id)
                if marker_pose is None:
                    continue
                opening = (
                    self._latest_openings[idx]
                    if idx < len(self._latest_openings)
                    else self.p_default_opening_m
                )
                marker_id = self._append_gripper_markers(
                    marker_array,
                    frame_id,
                    marker_pose,
                    self.p_candidate_color,
                    "candidate",
                    marker_id,
                    opening,
                )

        if marker_array.markers:
            self._pub_markers.publish(marker_array)

    def _pose_in_output_frame(self, pose, source_frame: str):
        if not self.p_output_frame or self.p_output_frame == source_frame:
            return deepcopy(pose), source_frame

        try:
            transform = self._tf_buffer.lookup_transform(
                self.p_output_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.p_tf_timeout_sec),
            )
        except TransformException as exc:
            self._warn_tf_throttled(source_frame, exc)
            return None, source_frame

        return self._transform_pose(pose, transform), self.p_output_frame

    def _warn_tf_throttled(self, source_frame: str, exc: Exception) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_tf_warn_ns < 2_000_000_000:
            return
        self._last_tf_warn_ns = now_ns
        self.get_logger().warn(
            f"gripper marker TF 변환 대기 중: {self.p_output_frame} <- {source_frame}: {exc}"
        )

    def _transform_pose(self, pose, transform):
        q_tf = [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        rotated_position = self._rotate_vec_by_quat(
            [pose.position.x, pose.position.y, pose.position.z],
            q_tf,
        )
        out = deepcopy(pose)
        out.position.x = float(transform.transform.translation.x + rotated_position[0])
        out.position.y = float(transform.transform.translation.y + rotated_position[1])
        out.position.z = float(transform.transform.translation.z + rotated_position[2])

        q_pose = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        q_out = self._quat_multiply(q_tf, q_pose)
        out.orientation.x = q_out[0]
        out.orientation.y = q_out[1]
        out.orientation.z = q_out[2]
        out.orientation.w = q_out[3]
        return out

    def _append_gripper_markers(
        self,
        marker_array: MarkerArray,
        frame_id: str,
        pose,
        color: ColorRGBA,
        ns: str,
        marker_id: int,
        opening: float,
    ) -> int:
        # 입력 grasp pose는 손가락 끝 중심점(TCP)으로 해석한다.
        # RViz 형상은 이 점을 기준으로 뒤쪽(-X)으로 배치한다.
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)
        palm_offset = self._rotate_vec_by_quat(
            [-self.p_finger_length_m, 0.0, 0.0],
            [qx, qy, qz, qw],
        )

        # 손바닥
        palm = Marker()
        palm.header.frame_id = frame_id
        palm.header.stamp = self.get_clock().now().to_msg()
        palm.ns = ns
        palm.id = marker_id
        palm.type = Marker.CUBE
        palm.action = Marker.ADD
        palm.pose = deepcopy(pose)
        palm.pose.position.x += float(palm_offset[0])
        palm.pose.position.y += float(palm_offset[1])
        palm.pose.position.z += float(palm_offset[2])
        palm.scale.x = self.p_finger_thickness_m
        palm.scale.y = self.p_palm_width_m
        palm.scale.z = self.p_finger_thickness_m
        palm.color = color
        palm.lifetime = rclpy.duration.Duration(seconds=self.p_lifetime_sec).to_msg()
        marker_array.markers.append(palm)
        marker_id += 1

        # 로컬 오프셋을 pose 회전에 맞춰 월드 좌표로 변환한다.
        left_offset = self._rotate_vec_by_quat(
            [-self.p_finger_length_m * 0.5, opening * 0.5, 0.0],
            [qx, qy, qz, qw],
        )
        right_offset = self._rotate_vec_by_quat(
            [-self.p_finger_length_m * 0.5, -opening * 0.5, 0.0],
            [qx, qy, qz, qw],
        )

        # 좌측 핑거
        left = Marker()
        left.header = palm.header
        left.ns = ns
        left.id = marker_id
        left.type = Marker.CUBE
        left.action = Marker.ADD
        left.pose = deepcopy(pose)
        left.pose.position.x += float(left_offset[0])
        left.pose.position.y += float(left_offset[1])
        left.pose.position.z += float(left_offset[2])
        left.scale.x = self.p_finger_length_m
        left.scale.y = self.p_finger_width_m
        left.scale.z = self.p_finger_thickness_m
        left.color = color
        left.lifetime = palm.lifetime
        marker_array.markers.append(left)
        marker_id += 1

        # 우측 핑거
        right = Marker()
        right.header = palm.header
        right.ns = ns
        right.id = marker_id
        right.type = Marker.CUBE
        right.action = Marker.ADD
        right.pose = deepcopy(pose)
        right.pose.position.x += float(right_offset[0])
        right.pose.position.y += float(right_offset[1])
        right.pose.position.z += float(right_offset[2])
        right.scale.x = self.p_finger_length_m
        right.scale.y = self.p_finger_width_m
        right.scale.z = self.p_finger_thickness_m
        right.color = color
        right.lifetime = palm.lifetime
        marker_array.markers.append(right)
        marker_id += 1

        return marker_id

    def _append_grasp_point_marker(self, marker_array: MarkerArray, frame_id: str, pose, ns: str, marker_id: int) -> int:
        """best grasp의 중심점을 RViz에서 잘 보이는 구 마커로 표시한다."""
        point = Marker()
        point.header.frame_id = frame_id
        point.header.stamp = self.get_clock().now().to_msg()
        point.ns = ns
        point.id = marker_id
        point.type = Marker.SPHERE
        point.action = Marker.ADD
        point.pose = deepcopy(pose)
        point.pose.orientation.x = 0.0
        point.pose.orientation.y = 0.0
        point.pose.orientation.z = 0.0
        point.pose.orientation.w = 1.0
        diameter = self.p_grasp_point_radius_m * 2.0
        point.scale.x = diameter
        point.scale.y = diameter
        point.scale.z = diameter
        point.color = self.p_grasp_point_color
        point.lifetime = rclpy.duration.Duration(seconds=self.p_lifetime_sec).to_msg()
        marker_array.markers.append(point)
        return marker_id + 1

    def _append_axes_markers(self, marker_array: MarkerArray, frame_id: str, pose, ns: str, marker_id: int) -> int:
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        origin = [pose.position.x, pose.position.y, pose.position.z]
        axis_x = self._rotate_vec_by_quat([self.p_axes_length_m, 0.0, 0.0], q)
        axis_y = self._rotate_vec_by_quat([0.0, self.p_axes_length_m, 0.0], q)
        axis_z = self._rotate_vec_by_quat([0.0, 0.0, self.p_axes_length_m], q)

        vectors = [
            (axis_x, ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.95)),
            (axis_y, ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.95)),
            (axis_z, ColorRGBA(r=0.0, g=0.3, b=1.0, a=0.95)),
        ]

        for vec, color in vectors:
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = ns
            m.id = marker_id
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.scale.x = self.p_axes_width_m
            m.scale.y = self.p_axes_width_m * 1.6
            m.scale.z = self.p_axes_width_m * 1.6
            m.color = color
            m.lifetime = rclpy.duration.Duration(seconds=self.p_lifetime_sec).to_msg()
            p0 = Point(x=float(origin[0]), y=float(origin[1]), z=float(origin[2]))
            p1 = Point(x=float(origin[0] + vec[0]), y=float(origin[1] + vec[1]), z=float(origin[2] + vec[2]))
            m.points = [p0, p1]
            marker_array.markers.append(m)
            marker_id += 1

        return marker_id

    @staticmethod
    def _rotate_vec_by_quat(v, q):
        """벡터 v를 쿼터니언 q(x,y,z,w)로 회전한다."""
        x, y, z, w = GripperMarkerNode._normalize_quat(q)
        vx, vy, vz = v
        # q * v * q_conj
        # v를 순허수 쿼터니언으로 보고 계산
        ix = w * vx + y * vz - z * vy
        iy = w * vy + z * vx - x * vz
        iz = w * vz + x * vy - y * vx
        iw = -x * vx - y * vy - z * vz

        rx = ix * w + iw * -x + iy * -z - iz * -y
        ry = iy * w + iw * -y + iz * -x - ix * -z
        rz = iz * w + iw * -z + ix * -y - iy * -x
        return [rx, ry, rz]

    @staticmethod
    def _normalize_quat(q):
        x, y, z, w = [float(value) for value in q]
        norm = (x * x + y * y + z * z + w * w) ** 0.5
        if norm <= 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [x / norm, y / norm, z / norm, w / norm]

    @staticmethod
    def _quat_multiply(a, b):
        ax, ay, az, aw = GripperMarkerNode._normalize_quat(a)
        bx, by, bz, bw = GripperMarkerNode._normalize_quat(b)
        return GripperMarkerNode._normalize_quat(
            [
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
                aw * bw - ax * bx - ay * by - az * bz,
            ]
        )


def main(args=None):
    rclpy.init(args=args)
    node = GripperMarkerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
