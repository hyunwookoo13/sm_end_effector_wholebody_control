import numpy as np

from sm_florence_2_vlm.yoloe_utils import (
    crop_mask_to_bbox,
    expand_yoloe_prompts,
    is_yoloe_geometry_valid,
    match_yoloe_detection_to_target,
    score_mask_colors,
    yoloe_candidate_reliability,
)


def test_expand_yoloe_prompts_adds_color_and_can_aliases():
    prompts = expand_yoloe_prompts(["blue can"])

    assert prompts[:2] == ["blue can", "blue soda can"]
    assert "can" not in prompts
    assert "soda can" not in prompts


def test_expand_yoloe_prompts_keeps_plain_aliases_for_uncolored_targets():
    prompts = expand_yoloe_prompts(["box"])

    assert prompts[:3] == ["box", "tray", "plastic tray"]


def test_score_mask_colors_counts_only_masked_pixels():
    image = np.zeros((6, 6, 3), dtype=np.uint8)
    image[:, :] = (255, 0, 0)  # blue background in BGR
    image[1:5, 1:5] = (0, 255, 255)  # yellow object in BGR

    mask = np.zeros((6, 6), dtype=bool)
    mask[1:5, 1:5] = True

    color_info = score_mask_colors(image, mask)

    assert color_info["observed_color"] == "yellow"
    assert color_info["scores"]["yellow"] > 0.95
    assert color_info["scores"]["blue"] < 0.05


def test_crop_mask_to_bbox_returns_bbox_local_mask():
    mask = np.zeros((8, 10), dtype=bool)
    mask[2:6, 3:8] = True
    bbox = {"xmin": 2, "ymin": 1, "xmax": 8, "ymax": 6}

    cropped = crop_mask_to_bbox(mask, bbox, width=10, height=8)

    assert cropped.shape == (6, 7)
    assert int(np.count_nonzero(cropped)) == 20


def test_match_yoloe_detection_requires_requested_color_score():
    match = match_yoloe_detection_to_target(
        "blue soda can",
        ["blue can"],
        {"observed_color": "blue", "scores": {"blue": 0.82}},
        min_color_score=0.08,
    )

    assert match == {
        "target_name": "blue can",
        "semantic_class": "can",
        "requested_color": "blue",
    }

    mismatch = match_yoloe_detection_to_target(
        "pink box",
        ["yellow box"],
        {"observed_color": "pink", "scores": {"yellow": 0.02, "pink": 0.91}},
        min_color_score=0.08,
    )

    assert mismatch is None

    weak_wrong_color = match_yoloe_detection_to_target(
        "box",
        ["yellow box"],
        {"observed_color": "red", "scores": {"yellow": 0.13, "red": 0.85}},
        min_color_score=0.08,
    )

    assert weak_wrong_color is None


def test_yoloe_geometry_rejects_tiny_box_candidates_and_large_table_masks():
    image_area = 960 * 720

    assert not is_yoloe_geometry_valid(
        {"xmin": 100, "ymin": 100, "xmax": 180, "ymax": 150},
        mask_area_px=int(image_area * 0.006),
        image_width=960,
        image_height=720,
        semantic_class="box",
        min_mask_area_px=40,
        max_mask_area_ratio=0.22,
        max_bbox_area_ratio=0.25,
        box_min_mask_area_ratio=0.015,
    )

    assert not is_yoloe_geometry_valid(
        {"xmin": 0, "ymin": 80, "xmax": 700, "ymax": 640},
        mask_area_px=int(image_area * 0.30),
        image_width=960,
        image_height=720,
        semantic_class="box",
        min_mask_area_px=40,
        max_mask_area_ratio=0.22,
        max_bbox_area_ratio=0.25,
        box_min_mask_area_ratio=0.015,
    )

    assert is_yoloe_geometry_valid(
        {"xmin": 540, "ymin": 120, "xmax": 860, "ymax": 560},
        mask_area_px=int(image_area * 0.18),
        image_width=960,
        image_height=720,
        semantic_class="box",
        min_mask_area_px=40,
        max_mask_area_ratio=0.22,
        max_bbox_area_ratio=0.25,
        box_min_mask_area_ratio=0.015,
    )


def test_yoloe_candidate_reliability_prefers_plausible_box_over_tiny_false_positive():
    image_area = 960 * 720
    tiny_can_like_box = yoloe_candidate_reliability(
        confidence=0.30,
        color_info={"scores": {}},
        semantic_match={"semantic_class": "box", "requested_color": None},
        mask_area_px=int(image_area * 0.006),
        image_width=960,
        image_height=720,
    )
    actual_box = yoloe_candidate_reliability(
        confidence=0.18,
        color_info={"scores": {}},
        semantic_match={"semantic_class": "box", "requested_color": None},
        mask_area_px=int(image_area * 0.18),
        image_width=960,
        image_height=720,
    )

    assert actual_box > tiny_can_like_box
