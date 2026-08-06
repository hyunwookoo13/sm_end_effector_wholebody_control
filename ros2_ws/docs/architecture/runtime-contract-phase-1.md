# Modular Packaging Phase 1 Runtime Contract

This document records both the verified runtime baseline before the Phase 1
package layout changes and the resolved Phase 1 ownership state. Tasks 2-5 use
the baseline as the compatibility checklist.

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

## Phase 1 integration verification (2026-08-06)

- Stale executable and Nav2-config reference scans returned no matches.
- `colcon list` reported all six workspace packages: `ee_switch_debug`,
  `sm_base_control_manager`, `sm_florence_2_vlm_ros2`, `sm_grasping_ros2`,
  `sm_natural_language_task`, and `sm_navigation_nav2`.
- The full source test command completed with **89 passed** in 0.57s.
- The isolated, sequential build completed with **6 packages finished** in
  3.29s:

  ```bash
  source /opt/ros/humble/setup.bash
  colcon --log-base log_modular_phase1 build --executor sequential \
    --build-base build_modular_phase1 \
    --install-base install_modular_phase1
  ```

- With `/opt/ros/humble/setup.bash` and
  `install_modular_phase1/setup.bash` sourced, the installed compatibility
  launch rendered successfully and the extracted executables were present:
  `sm_natural_language_task natural_language_task_parser`,
  `sm_natural_language_task natural_language_task_console`, and
  `sm_base_control_manager navigation_cmd_mux`.

`rosdep check --from-paths src --ignore-src` exited 2 because this host's
rosdep data cannot locate the `ament_python` key for each of the six
packages, while also reporting that all system dependencies have been
satisfied. This is recorded as an environment/rosdep-index warning; no
dependency declaration was changed to hide it.

### Isaac Sim end-to-end regression: externally pending

An Isaac Sim Kit process was present (`kit ./kit/kit ./apps/isaacsim.exp.full.kit
--ext-folder ./apps`), but `ros2 node list` discovered no ROS nodes. A safe
active ROS-bridge stage could therefore not be established, and neither Isaac
Sim nor its stage was started or modified for this verification.

When a baseline-equivalent Isaac Sim ROS-bridge stage is active, run:

```bash
cd "$(git rev-parse --show-toplevel)/ros2_ws"
source /opt/ros/humble/setup.bash
source install_modular_phase1/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py autostart:=false enable_nav2:=true
```

Capture the relevant ROS log lines while confirming: (1) a Korean
natural-language command produces the unchanged pick/place JSON; (2) YOLOE
selects the requested object; (3) Nav2 publishes through
`/cmd_vel_navigation`; (4) `navigation_cmd_mux` is the only final `/cmd_vel`
publisher; (5) red-can to yellow-box completes at baseline behavior; (6)
orange to yellow-box completes at baseline behavior; and (7) a consecutive
remote task performs the existing safe-retreat behavior. Apple behavior is
recorded but is not an improvement target for this packaging phase.

## Launch argument contract

### Pre-Phase-1 baseline defaults

The following table intentionally preserves the defaults captured before the
package move.

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

### Phase 1-resolved resource default

Phase 1 keeps every launch-argument value and runtime effect unchanged except
for the resource owner used to resolve `nav2_params_file`:

| State | Resolved `nav2_params_file` default |
|---|---|
| Pre-Phase-1 baseline | `FindPackageShare("ee_switch_debug")/config/nav2_rolling_odom.yaml` |
| Phase 1 final | `FindPackageShare("sm_navigation_nav2")/config/nav2_rolling_odom.yaml` |

This resource-path delta is intentional because the unchanged YAML moved to
its new owner. It does not change the file contents, parameter values, launch
argument name, or runtime behavior.

## Runtime node and topic contract

| Capability | Node name | Executable owner | Output/interface |
|---|---|---|---|
| Language parser | `natural_language_task_parser` | `sm_natural_language_task` | `/pick_place_task`, `/natural_language_task_status` |
| Navigation | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | `sm_navigation_nav2` | `/cmd_vel_navigation`, `navigate_to_pose` |
| Base arbitration | `navigation_cmd_mux` | `sm_base_control_manager` | `/cmd_vel` |
| Task orchestration | `pick_place_task_manager` | `ee_switch_debug` | `/base_control_mode`, `/base_control_blend` |
| Manipulation | `arm_yaw_rho_z_position_controller` | `ee_switch_debug` | `/joint_position_command`, `/cmd_vel_manipulation` |

Within the runtime node/topic table, Phase 1 may change only the `Executable
owner` column. Separately, the `nav2_params_file` default resolves the same
unchanged YAML through `sm_navigation_nav2`, as documented above. Node names,
topics, action interfaces, parameter defaults, QoS behavior, controller
values, and the existing perception and grasping package names remain runtime
compatibility requirements.
