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

## Final automated verification (2026-08-06)

The final source tree contains exactly eight ROS 2 packages:

```text
sm_base_control_manager
sm_bringup
sm_ee_wholebody_control
sm_florence_2_vlm_ros2
sm_grasping_ros2
sm_natural_language_task
sm_navigation_nav2
sm_task_orchestrator
```

A fresh prefix under `/tmp/sm-phase2-*` was used so the former package could
not resolve through stale files in the normal workspace overlay. The low-CPU
build completed with **8 packages finished in 4.27s**.

The final-owner source suite completed with **96 passed in 0.55s**. This is the
original 90 behavioral tests plus six package/bringup ownership-contract tests.

Installed ownership checks returned:

```text
sm_task_orchestrator pick_place_task_manager
sm_ee_wholebody_control arm_yaw_rho_z_position_controller
sm_ee_wholebody_control fixed_camera_tf_publisher
```

`sm_bringup/natural_language_pick_place.launch.py --show-args` retained the
protected defaults, including `enable_nav2=false`, pick/place standoff `0.70`,
hybrid distances `1.40`/`0.85`, and `local_llm_model=gemma3:4b`.

The isolated overlay could not resolve `ee_switch_debug`, and live source
scans found no Python import, launch package owner, or `FindPackageShare`
reference to it. Historical material is retained under
`ros2_ws/docs/archive/ee_switch_debug/` with a warning that its commands are
not current instructions.

`sm_bringup` is a launch-only `ament_python` metadata package with no runtime
node or console script. This matches the workspace's package pattern and
avoids the host Miniforge interpreter being selected by `ament_cmake` without
the ROS `catkin_pkg` module.

## Isaac Sim runtime regression (2026-08-07)

The final `sm_bringup` entry point was exercised against the baseline Isaac
Sim stage with `enable_nav2:=true`. Before bringup, `/clock`, `/joint_states`,
and `/odom` were all live at approximately 28--29 Hz. All four Nav2 lifecycle
nodes (`controller_server`, `planner_server`, `behavior_server`, and
`bt_navigator`) reached `active [3]`, and `/cmd_vel` had exactly one publisher:
`navigation_cmd_mux`.

The command `빨간색 캔을 노란색 박스에 넣어줘` parsed to `red can` and
`yellow box` and completed the full runtime sequence:

```text
NAVIGATE_PICK -> HANDOFF -> APPROACH -> DESCEND -> GRASP -> LIFT
-> FIND_PLACE -> PLACE:APPROACH -> PLACE:DESCEND -> RELEASE -> DONE
```

The consecutive command `오렌지를 노란색 박스에 넣어줘` parsed to `orange`
and `yellow box` and completed pick through `GRASP` and `LIFT`. Because the
yellow box was only 0.333 m away, the preserved baseline rule skipped Nav2
place (`0.333 m <= 1.200 m`) and used direct precision control. The subsequent
place approach exposed a pre-existing reachability boundary: the target
required approximately 0.798 rad at `joint1`, while the preserved upper limit
is 0.6981 rad (40 degrees), leaving a constant 0.100 rad yaw error.

This boundary was verified as unrelated to package extraction. The installed
controller had one `/joint_position_command` publisher and Isaac Sim had one
subscriber; command and feedback both stopped at 0.6981 rad. The extracted
controller's SHA-256 exactly matched the `f8a7122` source
(`983bb6a109e9d8ca5174946a7432f8adfab94554b94457956ded72ff32c43a37`),
and its joint-limit configuration was unchanged. Resolving the nearby-place
base-alignment/reachability behavior is therefore a later control change, not
part of this ownership-only refactor.

The first run also downloaded the 242 MB MobileCLIP asset and temporarily
blocked a Nav2 heartbeat. After the asset was cached and the ROS stack alone
was restarted, Nav2 remained active and the red-can task completed. This was
a first-run model warm-up condition rather than a package ownership failure.
