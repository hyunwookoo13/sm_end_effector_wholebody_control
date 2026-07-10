from sm_grasping_ros2.grasping_inference_node import sanitize_candidate_openings


def test_sanitize_candidate_openings_clamps_oversize_roi_noise():
    sanitized, oversized = sanitize_candidate_openings([0.457], max_opening_m=0.15)

    assert sanitized == [0.15]
    assert oversized == [0.457]


def test_sanitize_candidate_openings_keeps_valid_values():
    sanitized, oversized = sanitize_candidate_openings([0.04, 0.08], max_opening_m=0.15)

    assert sanitized == [0.04, 0.08]
    assert oversized == []
