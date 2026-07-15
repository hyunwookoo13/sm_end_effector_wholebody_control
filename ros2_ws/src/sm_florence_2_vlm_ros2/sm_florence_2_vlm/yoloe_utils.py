from __future__ import annotations

from math import atan2, cos, pi, sin
from typing import Any

import cv2
import numpy as np


COLOR_WORDS = {"red", "blue", "yellow", "green", "pink"}

CLASS_ALIASES = {
    "can": ["can", "soda can", "tin", "soup can", "tomato soup can"],
    "box": ["box", "tray", "plastic tray", "container", "bin"],
    "tray": ["tray", "box", "plastic tray", "container"],
    "cup": ["cup", "mug", "coffee cup", "teacup"],
    "mug": ["mug", "cup", "coffee mug", "coffee cup"],
    "bottle": ["bottle", "water bottle", "drink bottle"],
}

# Open-vocabulary detection still needs visually similar negative classes in the
# prompt set.  They are used only for competition and are not published unless
# the user explicitly requested them.
CLASS_CONFUSERS = {
    "apple": ["orange"],
    "orange": ["apple"],
}


def split_semantic_target(name: str) -> tuple[str | None, str]:
    tokens = str(name).strip().lower().split()
    if tokens and tokens[0] in COLOR_WORDS:
        return tokens[0], " ".join(tokens[1:]) or tokens[0]
    return None, " ".join(tokens)


def expand_yoloe_prompts(target_objects: list[str]) -> list[str]:
    prompts: list[str] = []
    for target in target_objects:
        target_name = str(target).strip().lower()
        if not target_name:
            continue
        requested_color, base_class = split_semantic_target(target_name)
        aliases = CLASS_ALIASES.get(base_class, [base_class])

        candidates = [target_name]
        if requested_color:
            candidates.extend(f"{requested_color} {alias}" for alias in aliases)
        else:
            candidates.extend(aliases)

        for candidate in candidates:
            if candidate and candidate not in prompts:
                prompts.append(candidate)
        for confuser in CLASS_CONFUSERS.get(base_class, []):
            if confuser and confuser not in prompts:
                prompts.append(confuser)
    return prompts


def class_aliases_for(name: str) -> set[str]:
    _, base_class = split_semantic_target(name)
    aliases = set(CLASS_ALIASES.get(base_class, [base_class]))
    aliases.add(base_class)
    return {alias for alias in aliases if alias}


def match_yoloe_detection_to_target(
    raw_label: str,
    targets: list[str],
    color_info: dict[str, Any],
    min_color_score: float = 0.08,
) -> dict[str, Any] | None:
    label = str(raw_label).strip().lower()
    if not label:
        return None

    for target in targets:
        target_name = str(target).strip().lower()
        if not target_name:
            continue
        requested_color, base_class = split_semantic_target(target_name)
        aliases = class_aliases_for(target_name)
        class_match = (
            target_name in label
            or label in aliases
            or any(alias and alias in label for alias in aliases)
        )
        if not class_match:
            continue
        if requested_color:
            if str(color_info.get("observed_color", "")).lower() != requested_color:
                continue
            score = float(color_info.get("scores", {}).get(requested_color, 0.0))
            if score < float(min_color_score):
                continue
        return {
            "target_name": target_name,
            "semantic_class": base_class,
            "requested_color": requested_color,
        }
    return None


def crop_mask_to_bbox(mask: np.ndarray, bbox: dict[str, int], width: int, height: int) -> np.ndarray:
    if mask.shape[:2] != (height, width):
        mask = cv2.resize(mask.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)

    xmin = max(0, min(width - 1, int(bbox["xmin"])))
    xmax = max(0, min(width - 1, int(bbox["xmax"])))
    ymin = max(0, min(height - 1, int(bbox["ymin"])))
    ymax = max(0, min(height - 1, int(bbox["ymax"])))
    if xmax < xmin:
        xmin, xmax = xmax, xmin
    if ymax < ymin:
        ymin, ymax = ymax, ymin
    return mask[ymin : ymax + 1, xmin : xmax + 1].astype(bool)


def canonical_axis_angle(angle_rad: float) -> float:
    """Normalize an undirected 2D object axis to [-pi/2, pi/2)."""
    angle = (float(angle_rad) + pi) % (2.0 * pi) - pi
    if angle >= pi / 2.0:
        angle -= pi
    elif angle < -pi / 2.0:
        angle += pi
    return angle


def estimate_mask_orientation(
    mask: np.ndarray | None,
    min_area_px: int = 20,
) -> dict[str, Any] | None:
    if mask is None:
        return None
    mask_bool = np.asarray(mask).astype(bool)
    ys, xs = np.nonzero(mask_bool)
    if len(xs) < int(min_area_px):
        return None

    points = np.column_stack((xs.astype(float), ys.astype(float)))
    centered = points - np.mean(points, axis=0)
    covariance = np.cov(centered, rowvar=False)
    if covariance.shape != (2, 2) or not np.all(np.isfinite(covariance)):
        return None

    values, vectors = np.linalg.eigh(covariance)
    order = np.argsort(values)[::-1]
    major_value = float(values[order[0]])
    minor_value = float(values[order[1]])
    if major_value <= 1e-9:
        return None

    major_axis = vectors[:, order[0]]
    angle = canonical_axis_angle(atan2(float(major_axis[1]), float(major_axis[0])))
    confidence = max(0.0, min(1.0, (major_value - minor_value) / max(major_value, 1e-9)))
    half = 0.5 * angle
    return {
        "angle_rad": angle,
        "confidence": confidence,
        "quaternion_xyzw": (0.0, 0.0, sin(half), cos(half)),
    }


def is_yoloe_geometry_valid(
    bbox: dict[str, int],
    mask_area_px: int,
    image_width: int,
    image_height: int,
    semantic_class: str = "",
    min_mask_area_px: int = 40,
    max_mask_area_ratio: float = 0.22,
    max_bbox_area_ratio: float = 0.25,
    box_min_mask_area_ratio: float = 0.015,
) -> bool:
    image_area = max(1, int(image_width) * int(image_height))
    bbox_w = max(0, int(bbox["xmax"]) - int(bbox["xmin"]) + 1)
    bbox_h = max(0, int(bbox["ymax"]) - int(bbox["ymin"]) + 1)
    bbox_area_ratio = float(bbox_w * bbox_h) / float(image_area)
    mask_area_ratio = float(max(0, int(mask_area_px))) / float(image_area)

    if bbox_area_ratio > float(max_bbox_area_ratio):
        return False
    if int(mask_area_px) > 0:
        if int(mask_area_px) < int(min_mask_area_px):
            return False
        if mask_area_ratio > float(max_mask_area_ratio):
            return False
    if semantic_class in {"box", "tray"} and mask_area_ratio < float(box_min_mask_area_ratio):
        return False
    return True


def yoloe_candidate_reliability(
    confidence: float,
    color_info: dict[str, Any],
    semantic_match: dict[str, Any],
    mask_area_px: int,
    image_width: int,
    image_height: int,
) -> float:
    score = float(confidence)
    requested_color = semantic_match.get("requested_color")
    if requested_color:
        score += float(color_info.get("scores", {}).get(str(requested_color), 0.0))

    semantic_class = str(semantic_match.get("semantic_class", ""))
    if semantic_class in {"box", "tray"}:
        image_area = max(1, int(image_width) * int(image_height))
        mask_area_ratio = float(max(0, int(mask_area_px))) / float(image_area)
        score += min(mask_area_ratio, 0.20)
    return score


def bbox_iou(first: dict[str, int], second: dict[str, int]) -> float:
    """Return intersection-over-union for two inclusive pixel bboxes."""
    xmin = max(int(first["xmin"]), int(second["xmin"]))
    ymin = max(int(first["ymin"]), int(second["ymin"]))
    xmax = min(int(first["xmax"]), int(second["xmax"]))
    ymax = min(int(first["ymax"]), int(second["ymax"]))
    intersection = max(0, xmax - xmin + 1) * max(0, ymax - ymin + 1)
    first_area = max(0, int(first["xmax"]) - int(first["xmin"]) + 1) * max(
        0, int(first["ymax"]) - int(first["ymin"]) + 1
    )
    second_area = max(0, int(second["xmax"]) - int(second["xmin"]) + 1) * max(
        0, int(second["ymax"]) - int(second["ymin"]) + 1
    )
    union = first_area + second_area - intersection
    return float(intersection) / float(union) if union > 0 else 0.0


def suppress_cross_class_overlaps(
    candidates: list[dict[str, Any]],
    iou_threshold: float = 0.60,
) -> list[dict[str, Any]]:
    """Keep the most reliable label when different classes cover one object."""
    kept: list[dict[str, Any]] = []
    ordered = sorted(candidates, key=lambda item: float(item["reliability"]), reverse=True)
    for candidate in ordered:
        duplicate = any(
            str(candidate["object_name"]) != str(selected["object_name"])
            and bbox_iou(candidate["bbox"], selected["bbox"]) >= float(iou_threshold)
            for selected in kept
        )
        if not duplicate:
            kept.append(candidate)
    return kept


def score_mask_colors(image_bgr: np.ndarray, mask: np.ndarray | None) -> dict[str, Any]:
    if image_bgr.size == 0:
        return {"observed_color": "", "scores": {}}

    if mask is None:
        valid_region = np.ones(image_bgr.shape[:2], dtype=bool)
    else:
        if mask.shape[:2] != image_bgr.shape[:2]:
            mask = cv2.resize(
                mask.astype(np.uint8),
                (image_bgr.shape[1], image_bgr.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            ).astype(bool)
        valid_region = mask.astype(bool)

    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    valid = valid_region & (saturation > 45) & (value > 45)
    valid_count = int(np.count_nonzero(valid))
    if valid_count <= 0:
        return {"observed_color": "", "scores": {}}

    hue = hsv[:, :, 0]
    masks = {
        "red": valid & ((hue <= 10) | (hue >= 150)),
        "yellow": valid & (hue >= 18) & (hue <= 42),
        "green": valid & (hue >= 43) & (hue <= 85),
        "blue": valid & (hue >= 86) & (hue <= 135),
        "pink": valid & (hue >= 136) & (hue <= 169),
    }
    scores = {
        color: float(np.count_nonzero(color_mask)) / float(valid_count)
        for color, color_mask in masks.items()
    }
    observed_color = max(scores, key=scores.get) if scores else ""
    if scores and scores[observed_color] < 0.08:
        observed_color = ""
    return {"observed_color": observed_color, "scores": scores}
