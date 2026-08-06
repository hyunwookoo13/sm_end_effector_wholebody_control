# Modular Packaging Phase 2 Runtime Contract

This contract freezes the verified runtime immediately before splitting
`ee_switch_debug` into task orchestration, EE whole-body control, and bringup
packages. Phase 2 changes ownership only; runtime behavior remains protected.

## Rollback baseline

- Verified runtime baseline: `f8a7122`
- Approved design baseline: `1da97b0`
- Implementation-plan baseline: `1839705`
- DDS: preserve the untracked `ros2_ws/dds_setting/` directory unchanged.

## Baseline automated verification (2026-08-06)

The isolated worktree was built with:

```bash
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-up-to ee_switch_debug
```

Result: **6 packages finished in 3.20s**.

The complete source suite was run with `/usr/bin/python3` after sourcing the
isolated overlay. Result: **90 passed in 0.59s**.

## Baseline package and executable ownership

The six packages present before Phase 2 are:

- `ee_switch_debug`
- `sm_base_control_manager`
- `sm_florence_2_vlm_ros2`
- `sm_grasping_ros2`
- `sm_natural_language_task`
- `sm_navigation_nav2`

`ee_switch_debug` owns these baseline console scripts:

```text
arm_yaw_velocity_controller
arm_yaw_rho_z_position_controller
arm_yaw_rho_z_velocity_controller
arm_joint_effect_probe
arm_rho_z_velocity_controller
arm_z_velocity_controller
florence_target_tf_bridge
fixed_camera_tf_publisher
grasp_target_tf_bridge
l100_pointer_target
live_yaw_rho_z_monitor
mobile_target_controller
pick_place_task_manager
target_arm_ik_controller
target_base_controller
target_frame_debug
wholebody_yaw_rho_z_controller
target_wholebody_controller
wholebody_debug_logger
plot_wholebody_debug
plot_yaw_rho_z_debug
```

Its six baseline launches are:

```text
arm_position_target_in.launch.py
florence_grasp_position_control.launch.py
florence_long_range_pick_place_control.launch.py
florence_pick_place_control.launch.py
grasp_wholebody.launch.py
wholebody_monitor.launch.py
```

Protected baseline ownership:

```text
pick_place_task_manager -> ee_switch_debug (baseline only)
arm_yaw_rho_z_position_controller -> ee_switch_debug (baseline only)
florence_long_range_pick_place_control.launch.py -> ee_switch_debug (baseline only)
navigation_cmd_mux -> sm_base_control_manager
/cmd_vel final publisher -> navigation_cmd_mux only
```

## Protected launch defaults

The complete Phase 1 launch-argument table remains authoritative. The
following high-risk defaults are explicitly frozen for Phase 2:

| Argument | Baseline value |
|---|---|
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
| `task_manager_rate_hz` | `30.0` |
| `target_objects_publish_period` | `1.0` |
| `fresh_grasp_delay_sec` | `0.35` |
| `use_local_llm` | `true` |
| `use_rule_task_parser` | `false` |
| `local_llm_model` | `gemma3:4b` |

The full baseline launch renders with:

```bash
ros2 launch ee_switch_debug \
  florence_long_range_pick_place_control.launch.py --show-args
```

## Phase 2 target ownership

| Capability | Final owner |
|---|---|
| Pick/Place state machine and navigation geometry | `sm_task_orchestrator` |
| EE-targeted arm/base controllers and controller resources | `sm_ee_wholebody_control` |
| Full-system launch composition and operator launch docs | `sm_bringup` |

Only package/import/resource paths may change during extraction. Node names,
topics, actions, frames, QoS, parameters, controller values, and behavior must
match this baseline.
