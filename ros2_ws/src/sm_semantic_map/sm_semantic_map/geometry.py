from __future__ import annotations

import math
import statistics
from dataclasses import dataclass


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def world_to_map(
    point: Point3,
    map_x_in_world: float,
    map_y_in_world: float,
    map_yaw_in_world: float,
) -> Point3:
    """Transform a world-frame point into the fixed map frame."""
    dx = float(point.x) - float(map_x_in_world)
    dy = float(point.y) - float(map_y_in_world)
    cosine = math.cos(float(map_yaw_in_world))
    sine = math.sin(float(map_yaw_in_world))
    return Point3(
        x=cosine * dx + sine * dy,
        y=-sine * dx + cosine * dy,
        z=float(point.z),
    )


def summarize_points(points: list[Point3]) -> dict[str, object]:
    if not points:
        return {"samples": 0, "median": None, "stddev": None}
    axes = ([point.x for point in points], [point.y for point in points], [point.z for point in points])
    return {
        "samples": len(points),
        "median": [statistics.median(axis) for axis in axes],
        "stddev": [statistics.pstdev(axis) for axis in axes],
    }
