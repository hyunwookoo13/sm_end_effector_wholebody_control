#!/usr/bin/env python3
"""Drive a slow map-alignment route and score the rear scan at each stop."""

import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


HERE = Path(__file__).resolve().parent
MAP_IMAGE = HERE / "warehouse_map.png"
MAP_ORIGIN_X = -11.975
MAP_ORIGIN_Y = -17.975
MAP_RESOLUTION = 0.05
WAYPOINTS = [
    ("east", -0.5, 3.5),
    ("south", -0.5, 1.0),
    ("north", -0.5, 5.0),
]


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_yaw(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


def rotate_vector(rotation, vector):
    qx, qy, qz, qw = (
        rotation.x,
        rotation.y,
        rotation.z,
        rotation.w,
    )
    vx, vy, vz = vector
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


class AlignmentRouteValidator(Node):
    def __init__(self):
        super().__init__("alignment_route_validator")
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)]
        )
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan = None
        self.create_subscription(
            LaserScan,
            "/laser_scan_2",
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        image = cv2.imread(str(MAP_IMAGE), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"failed to load map image: {MAP_IMAGE}")
        self.map_image = image
        self.map_height, self.map_width = image.shape
        self.obstacle_distance_m = (
            distance_transform_edt(~(image < 50)) * MAP_RESOLUTION
        )

    def _scan_callback(self, message: LaserScan) -> None:
        self.scan = message

    def stop(self) -> None:
        command = Twist()
        for _ in range(5):
            self.cmd_publisher.publish(command)
            rclpy.spin_once(self, timeout_sec=0.03)

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(
            "map", "chassis_link", Time()
        ).transform
        return (
            transform.translation.x,
            transform.translation.y,
            quaternion_yaw(transform.rotation),
        )

    def map_clearance(self, x: float, y: float) -> float:
        column = int((x - MAP_ORIGIN_X) / MAP_RESOLUTION)
        row = self.map_height - 1 - int(
            (y - MAP_ORIGIN_Y) / MAP_RESOLUTION
        )
        if not (0 <= column < self.map_width and 0 <= row < self.map_height):
            return 0.0
        return float(self.obstacle_distance_m[row, column])

    def forward_obstacle_distance(self) -> float:
        if self.scan is None:
            return 0.0
        transform = self.tf_buffer.lookup_transform(
            "chassis_link", self.scan.header.frame_id, Time()
        ).transform
        nearest = math.inf
        for index, distance in enumerate(self.scan.ranges):
            if (
                not math.isfinite(distance)
                or distance < 0.55
                or distance >= self.scan.range_max
            ):
                continue
            angle = self.scan.angle_min + index * self.scan.angle_increment
            rotated = rotate_vector(
                transform.rotation,
                (
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                    0.0,
                ),
            )
            point_x = transform.translation.x + rotated[0]
            point_y = transform.translation.y + rotated[1]
            if 0.20 < point_x < nearest and abs(point_y) < 0.45:
                nearest = point_x
        return nearest

    def score_alignment(self, label: str) -> dict:
        scan = self.scan
        transform = self.tf_buffer.lookup_transform(
            "map", scan.header.frame_id, Time()
        ).transform
        distances = []
        for index, distance in enumerate(scan.ranges):
            if (
                not math.isfinite(distance)
                or distance < 1.0
                or distance >= scan.range_max
            ):
                continue
            angle = scan.angle_min + index * scan.angle_increment
            rotated = rotate_vector(
                transform.rotation,
                (
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                    0.0,
                ),
            )
            map_x = transform.translation.x + rotated[0]
            map_y = transform.translation.y + rotated[1]
            column = int((map_x - MAP_ORIGIN_X) / MAP_RESOLUTION)
            row = self.map_height - 1 - int(
                (map_y - MAP_ORIGIN_Y) / MAP_RESOLUTION
            )
            if 0 <= column < self.map_width and 0 <= row < self.map_height:
                distances.append(self.obstacle_distance_m[row, column])

        values = np.asarray(distances)
        x, y, yaw = self.current_pose()
        result = {
            "label": label,
            "x": x,
            "y": y,
            "yaw_deg": math.degrees(yaw),
            "count": len(values),
            "within_5cm": float(np.mean(values <= 0.05) * 100.0),
            "within_10cm": float(np.mean(values <= 0.10) * 100.0),
            "median_m": float(np.median(values)),
        }
        print(
            "VALIDATION SCORE "
            f"{label}: pose=({x:.3f},{y:.3f},{result['yaw_deg']:.2f}deg) "
            f"points={len(values)} within_5cm={result['within_5cm']:.1f}% "
            f"within_10cm={result['within_10cm']:.1f}% "
            f"median={result['median_m']:.3f}m",
            flush=True,
        )
        return result

    def drive_to(self, label: str, goal_x: float, goal_y: float) -> None:
        deadline = time.monotonic() + 120.0
        last_report = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            x, y, yaw = self.current_pose()
            dx = goal_x - x
            dy = goal_y - y
            distance = math.hypot(dx, dy)
            if distance <= 0.10:
                self.stop()
                print(
                    f"VALIDATION REACHED {label}: remaining={distance:.3f}m",
                    flush=True,
                )
                return

            heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
            obstacle_distance = self.forward_obstacle_distance()
            if obstacle_distance < 0.65:
                self.stop()
                raise RuntimeError(
                    f"forward obstacle at {obstacle_distance:.3f}m before {label}"
                )
            if self.map_clearance(x, y) < 0.55:
                self.stop()
                raise RuntimeError(f"map clearance below 0.55m before {label}")

            command = Twist()
            if abs(heading_error) > 0.20:
                command.angular.z = max(
                    -0.30, min(0.30, 1.2 * heading_error)
                )
            else:
                command.linear.x = min(0.20, max(0.06, 0.35 * distance))
                command.angular.z = max(
                    -0.20, min(0.20, 1.0 * heading_error)
                )
            self.cmd_publisher.publish(command)

            now = time.monotonic()
            if now - last_report >= 5.0:
                print(
                    f"VALIDATION DRIVING {label}: pose=({x:.2f},{y:.2f},"
                    f"{math.degrees(yaw):.1f}deg) remaining={distance:.2f}m",
                    flush=True,
                )
                last_report = now

        self.stop()
        raise RuntimeError(f"timeout while driving to {label}")


def main() -> int:
    rclpy.init()
    node = AlignmentRouteValidator()
    results = []
    try:
        deadline = time.monotonic() + 8.0
        while (
            node.scan is None
            or not node.tf_buffer.can_transform("map", "chassis_link", Time())
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.scan is None:
            raise RuntimeError("rear LaserScan was not received")

        node.stop()
        results.append(node.score_alignment("start"))
        for label, goal_x, goal_y in WAYPOINTS:
            node.drive_to(label, goal_x, goal_y)
            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.1)
            results.append(node.score_alignment(label))

        passed = all(
            result["within_10cm"] >= 95.0
            and result["median_m"] <= 0.05
            for result in results
        )
        print(
            f"VALIDATION RESULT: {'PASS' if passed else 'FAIL'} "
            f"({len(results)} poses)",
            flush=True,
        )
        return 0 if passed else 2
    except Exception as error:
        print(f"VALIDATION ABORTED: {error}", flush=True)
        return 1
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
