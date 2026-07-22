from pathlib import Path


def test_top_down_launch_uses_yoloe_grasping_and_validation_only_controller():
    launch_path = (
        Path(__file__).parents[1]
        / "launch"
        / "yoloe_top_down_grasp_test.launch.py"
    )
    source = launch_path.read_text()

    assert 'executable="sm_yoloe_vlm_node"' in source
    assert 'executable="fixed_camera_tf_publisher"' in source
    assert 'executable="grasping_inference_node"' in source
    assert 'executable="gripper_marker_node"' in source
    assert 'executable="pick_perception_snapshot"' in source
    assert 'executable="grasp_target_tf_bridge"' in source
    assert 'executable="arm_yaw_rho_z_position_controller"' in source
    assert 'executable="sm_florence_2_vlm_node"' not in source
    assert '"enable_base_motion": False' in source
    assert '"freeze_on_pick_command": True' in source
    assert (
        '"roi_pointcloud_topic": "/sm_florence_2_vlm/roi_pointcloud_live"'
        in source
    )
    assert (
        '"input_roi_pointcloud_topic": "/sm_florence_2_vlm/roi_pointcloud"'
        in source
    )
    assert '"output_grasp_best_topic": "/sm_grasping/grasp_best_live"' in source
    assert (
        '"output_grasp_openings_topic": "/sm_grasping/grasp_openings_live"'
        in source
    )
    assert '"input_grasp_best_topic": "/sm_grasping/grasp_best"' in source
    assert (
        '"input_grasp_openings_topic": "/sm_grasping/grasp_openings"'
        in source
    )
    assert '"gripper_close_position": 0.8' in source
    assert '"grasp_lift_height": 0.12' in source
    assert '"return_home_after_pick": True' in source
    assert '"enable_staged_top_down_approach": True' in source
    assert '"enable_precomputed_top_down_sequence": True' in source
    assert '"top_down_use_fixed_reachable_orientation": True' in source
    assert '"top_down_use_group_sequential_approach": False' in source
    assert '"top_down_use_group_blended_approach": True' in source
    assert '"top_down_blend_orientation_during_descent": False' in source
    assert '"top_down_minimum_orientation_fraction": 0.50' in source
    assert '"top_down_orientation_search_steps": 20' in source
    assert '"top_down_descend_max_xy_deviation": 0.015' in source
    assert '"top_down_descend_max_orientation_deviation": 0.035' in source
    assert '"top_down_transit_joint_tolerance": 0.050' in source
    assert '"top_down_segment_min_duration": 0.6' in source
    assert '"top_down_stage_acceleration": 1.6' in source
    assert '"top_down_lift_home_clearance": 0.06' in source
    assert '"top_down_alignment_velocity_limits": [0.70, 1.05, 1.20, 0.95, 0.95, 0.95]' in source
    assert '"top_down_path_velocity_limits": [0.42, 0.56, 0.68, 0.56, 0.42, 0.56]' in source
    assert '"top_down_radial_inward_offset": 0.025' in source
    assert '"top_down_waypoint_spacing": 0.01' in source
    assert '"top_down_approach_joint_step": 0.12' in source
    assert '"top_down_minimum_approach_clearance": 0.08' in source
    assert '"pick_max_joint_excursion": [0.65, 2.50, 2.30, 3.00, 3.00, 3.00]' in source
    assert '"top_down_pregrasp_clearance": ParameterValue(' in source
    assert '"require_descend_confirmation": False' in source
    assert 'default_value="0.18"' in source
    assert 'default_value="0.10"' in source
    assert 'default_value="0.0"' in source
    assert 'default_value="rsd455_color_optical_frame"' in source
    assert 'default_value="chassis_link"' in source
