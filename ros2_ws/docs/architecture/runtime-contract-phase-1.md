# Modular Packaging Phase 1 Runtime Contract

This document records the verified runtime baseline before the Phase 1 package
layout changes. Tasks 2-5 use it as the compatibility checklist.

## Baseline verification

- Unit tests: **86 passed** in 0.54s.
- Build: `colcon build --executor sequential --packages-select ee_switch_debug sm_florence_2_vlm_ros2 sm_grasping_ros2` completed with **3 packages finished** in 1.80s.
- Launch argument capture command:

  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py --show-args
  ```

For this ROS 2 Humble workspace, the canonical test interpreter is
`/usr/bin/python3`. Source `/opt/ros/humble/setup.bash` and the workspace
overlay (`install/setup.bash`) before running `/usr/bin/python3 -m pytest` when
tests import in-workspace packages. The host `python3` resolves to
`/home/kiro/miniforge3/bin/python3`, which does not provide `pytest`.

## Launch argument contract

| Argument | Default |
|---|---|
| `pick_yoloe_config_file` | `PathJoinSubstitution('FindPackageShare(pkg='sm_florence_2_vlm_ros2'), 'config', 'sm_yoloe_vlm_rsd455.yaml'')` |
| `place_yoloe_config_file` | `PathJoinSubstitution('FindPackageShare(pkg='sm_florence_2_vlm_ros2'), 'config', 'sm_yoloe_vlm_rsd455_place.yaml'')` |
| `grasping_config_file` | `PathJoinSubstitution('FindPackageShare(pkg='sm_grasping_ros2'), 'config', 'sm_grasping_isaac.yaml'')` |
| `place_grasping_config_file` | `PathJoinSubstitution('FindPackageShare(pkg='sm_grasping_ros2'), 'config', 'sm_grasping_isaac_place.yaml'')` |
| `controller_config_file` | `PathJoinSubstitution('FindPackageShare(pkg='ee_switch_debug'), 'config', 'arm_position_target_in.yaml'')` |
| `nav2_params_file` | `PathJoinSubstitution('FindPackageShare(pkg='ee_switch_debug'), 'config', 'nav2_rolling_odom.yaml'')` |
| `pick_object` | `can` |
| `place_object` | `box` |
| `autostart` | `true` |
| `enable_base_motion` | `true` |
| `enable_nav2` | `false` |
| `pick_standoff_m` | `0.70` |
| `place_standoff_m` | `0.70` |
| `place_direct_approach_distance_m` | `1.20` |
| `enable_hybrid_handoff` | `true` |
| `hybrid_outer_distance_m` | `1.40` |
| `hybrid_inner_distance_m` | `0.85` |
| `mux_max_linear_velocity_mps` | `1.50` |
| `mux_max_angular_velocity_rps` | `1.40` |
| `mux_max_linear_acceleration_mps2` | `2.00` |
| `mux_max_angular_acceleration_rps2` | `2.00` |
| `publish_fixed_camera_tf` | `false` |
| `target_parent_frame` | `odom` |
| `pick_camera_frame` | `rsd455_color_optical_frame` |
| `place_camera_frame` | `rsd455_color_optical_frame2` |
| `place_detections_topic` | `/sm_florence_2_vlm_place/detections` |
| `grasp_offset_z` | `0.03` |
| `grasp_descend_depth` | `0.07` |
| `place_offset_z` | `0.10` |
| `place_descend_depth` | `0.03` |
| `place_open_duration` | `0.8` |
| `return_home_after_place` | `true` |
| `place_offset_x` | `0.0` |
| `place_offset_y` | `0.0` |
| `place_target_offset_z` | `0.0` |
| `task_manager_rate_hz` | `30.0` |
| `target_objects_publish_period` | `1.0` |
| `task_manager_tf_timeout_sec` | `0.1` |
| `fresh_grasp_delay_sec` | `0.35` |
| `enable_natural_language_task_parser` | `true` |
| `natural_language_task_topic` | `/natural_language_task` |
| `use_local_llm` | `true` |
| `use_rule_task_parser` | `false` |
| `local_llm_model` | `gemma3:4b` |
| `ollama_url` | `http://127.0.0.1:11434/api/chat` |

The following defaults are explicitly protected: `enable_nav2=false`,
`pick_standoff_m=0.70`, `place_standoff_m=0.70`,
`hybrid_outer_distance_m=1.40`, `hybrid_inner_distance_m=0.85`,
`local_llm_model=gemma3:4b`, `use_local_llm=true`, and
`use_rule_task_parser=false`.

## Runtime node and topic contract

| Capability | Node name | Executable owner | Output/interface |
|---|---|---|---|
| Language parser | `natural_language_task_parser` | `ee_switch_debug` | `/pick_place_task`, `/natural_language_task_status` |
| Navigation | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | Nav2 via `ee_switch_debug` launch | `/cmd_vel_navigation`, `navigate_to_pose` |
| Base arbitration | `navigation_cmd_mux` | `ee_switch_debug` | `/cmd_vel` |
| Task orchestration | `pick_place_task_manager` | `ee_switch_debug` | `/base_control_mode`, `/base_control_blend` |
| Manipulation | `arm_yaw_rho_z_position_controller` | `ee_switch_debug` | `/joint_position_command`, `/cmd_vel_manipulation` |

**Phase 1 may change only the `Executable owner` column.** Node names,
topics, action interfaces, defaults, QoS behavior, controller values, and the
existing perception and grasping package names are runtime compatibility
requirements.
