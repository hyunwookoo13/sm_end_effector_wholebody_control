from dataclasses import dataclass
from math import atan2, hypot, isfinite, pi


@dataclass(frozen=True)
class PlanarPose:
    x: float
    y: float
    yaw: float


def hybrid_blend_weight(distance_m: float, outer_m: float, inner_m: float) -> float:
    """Return a smooth Nav2-to-precision blend weight in [0, 1]."""
    outer = max(float(outer_m), float(inner_m))
    inner = min(float(outer_m), float(inner_m))
    distance = float(distance_m)
    if outer - inner <= 1e-9:
        return 1.0 if distance <= inner else 0.0
    progress = max(0.0, min(1.0, (outer - distance) / (outer - inner)))
    return progress * progress * (3.0 - 2.0 * progress)


def compute_standoff_pose(
    base_xy: tuple[float, float],
    object_xy: tuple[float, float],
    standoff_m: float,
) -> PlanarPose:
    """Place the base on the base-to-object ray and point it at the object."""
    base_x, base_y = (float(value) for value in base_xy)
    object_x, object_y = (float(value) for value in object_xy)
    dx = object_x - base_x
    dy = object_y - base_y
    distance = hypot(dx, dy)
    if distance <= 1e-9:
        return PlanarPose(object_x, object_y, 0.0)

    standoff = max(0.0, float(standoff_m))
    scale = standoff / distance
    return PlanarPose(
        x=object_x - dx * scale,
        y=object_y - dy * scale,
        yaw=atan2(dy, dx),
    )


def should_skip_navigation(
    kind: str,
    base_xy: tuple[float, float],
    object_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    goal_skip_distance_m: float,
    place_direct_approach_distance_m: float,
) -> tuple[bool, str]:
    """Decide whether precision control can take over without a Nav2 goal."""
    base_x, base_y = (float(value) for value in base_xy)
    object_x, object_y = (float(value) for value in object_xy)
    goal_x, goal_y = (float(value) for value in goal_xy)

    goal_distance = hypot(goal_x - base_x, goal_y - base_y)
    if goal_distance <= max(0.0, float(goal_skip_distance_m)):
        return True, f"navigation goal is nearby ({goal_distance:.3f}m)"

    object_distance = hypot(object_x - base_x, object_y - base_y)
    direct_limit = max(0.0, float(place_direct_approach_distance_m))
    if str(kind).strip().lower() == "place" and direct_limit > 0.0:
        if object_distance <= direct_limit:
            return True, (
                "place target is inside precision approach range "
                f"({object_distance:.3f}m <= {direct_limit:.3f}m)"
            )

    return False, ""


def rear_sector_clearance(
    ranges: list[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    half_angle: float,
    ignore_below: float,
) -> float | None:
    """Return the nearest valid point around angle zero of a rear-facing scanner."""
    nearest: float | None = None
    sector = max(0.0, float(half_angle))
    lower = max(float(range_min), float(ignore_below))
    upper = float(range_max)
    for index, raw_range in enumerate(ranges):
        distance = float(raw_range)
        if not isfinite(distance) or distance < lower or distance > upper:
            continue
        angle = float(angle_min) + index * float(angle_increment)
        wrapped = (angle + pi) % (2.0 * pi) - pi
        if abs(wrapped) > sector:
            continue
        if nearest is None or distance < nearest:
            nearest = distance
    return nearest
