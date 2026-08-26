from pathlib import Path


LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "semantic_db_existing_pick_place.launch.py"
)
RVIZ_LAUNCH = (
    Path(__file__).parents[1]
    / "launch"
    / "semantic_map_rviz.launch.py"
)
FAST_NAV2_PARAMS = (
    Path(__file__).parents[3]
    / "maps"
    / "warehouse_isaac_occupancy_20260813"
    / "nav2_static_map_mvp.yaml"
)


def test_launch_reuses_existing_pipeline_without_split_executors():
    text = LAUNCH.read_text()
    assert "natural_language_pick_place.launch.py" in text
    assert '"enable_natural_language_task_parser": "false"' in text
    assert '"enable_nav2": "false"' in text
    assert '"external_place_navigation": "true"' in text
    assert '"existing_task_topic": "/pick_place_task"' in text
    assert 'executable="semantic_mission_orchestrator"' in text
    assert "semantic_pick_executor" not in text
    assert "semantic_place_executor" not in text
    assert "update_object_location" not in text


def test_launch_has_one_request_driven_navigation_adapter_for_both_roles():
    text = LAUNCH.read_text()
    assert text.count('executable="semantic_nav2_adapter"') == 1
    assert '"target_role": "pick"' in text
    assert '"request_topic": "/semantic_navigation/request"' in text
    assert '"auto_start_on_resolved_task": False' in text
    assert 'LaunchConfiguration("handoff_distance_m")' in text
    assert '"target_objects_publish_period": "3600.0"' in text
    assert 'DeclareLaunchArgument("handoff_distance_m", default_value="0.35")' in text


def test_launch_uses_fast_static_map_without_replacing_validation_profile():
    text = LAUNCH.read_text()
    assert "nav2_static_map_mvp.yaml" in text
    assert "nav2_static_map_validation.yaml" not in text
    assert '"phase_command_topic": "/pick_place_phase_command"' in text


def test_rviz_visualization_is_observer_only_and_opt_in():
    text = LAUNCH.read_text()
    assert 'executable="semantic_map_marker_publisher"' in text
    assert '"marker_topic": "/semantic_map/markers"' in text
    assert 'DeclareLaunchArgument("use_rviz", default_value="false")' in text
    assert 'condition=IfCondition(LaunchConfiguration("use_rviz"))' in text


def test_two_mission_demo_sequence_is_explicitly_opt_in():
    text = LAUNCH.read_text()
    assert 'executable="demo_task_sequencer"' in text
    assert 'DeclareLaunchArgument("auto_demo_sequence", default_value="false")' in text
    assert 'condition=IfCondition(LaunchConfiguration("auto_demo_sequence"))' in text
    assert '"mission_status_topic": "/semantic_mvp/status"' in text


def test_lightweight_rviz_preview_does_not_start_perception_or_manipulation():
    text = RVIZ_LAUNCH.read_text()
    assert 'executable="semantic_map_marker_publisher"' in text
    assert 'executable="rviz2"' in text
    assert 'executable="map_server"' in text
    assert "natural_language_pick_place.launch.py" not in text
    assert "sm_yoloe_vlm" not in text


def test_demo_rviz_keeps_global_and_local_costmaps_visible():
    text = (
        Path(__file__).parents[1] / "config" / "semantic_mvp.rviz"
    ).read_text()
    assert "Value: /global_costmap/costmap" in text
    assert "Value: /local_costmap/costmap" in text
    assert "Name: Global Costmap" in text
    assert "Name: Local Costmap" in text


def test_fast_profile_restores_long_range_dynamics_and_keeps_static_map():
    text = FAST_NAV2_PARAMS.read_text()
    assert "controller_frequency: 20.0" in text
    assert "max_vel_x: 1.5" in text
    assert "max_speed_xy: 1.5" in text
    assert "max_vel_theta: 0.45" in text
    assert "acc_lim_x: 2.0" in text
    assert "decel_lim_x: -2.2" in text
    assert "global_frame: map" in text
    assert "rolling_window: false" in text
    assert "plugin: nav2_costmap_2d::StaticLayer" in text
