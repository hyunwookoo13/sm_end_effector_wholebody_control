# EE Whole-Body Control, Task Orchestration, and Bringup Split Design

## 1. Objective

Replace the overloaded `ee_switch_debug` package with role-based `sm_*`
packages while preserving the currently verified natural-language
Pick-and-Place behavior.

The final runtime architecture has three owners:

- `sm_task_orchestrator`: decides what phase the task is in and what capability
  should act next.
- `sm_ee_wholebody_control`: executes end-effector-targeted arm/base control.
- `sm_bringup`: assembles the complete system without containing runtime logic.

This is a packaging and ownership change. It must not tune velocities, gains,
offsets, timeouts, prompts, aliases, standoff distances, or task behavior.

## 2. Approaches Considered

### Rename only

Renaming `ee_switch_debug` to `sm_ee_wholebody_control` would remove the debug
name but would leave task sequencing and full-system launch composition mixed
with control code.

### Two-package split

Separating task orchestration and control would clarify runtime logic, but one
of those packages would still need to own launch composition and dependencies
for perception, grasping, language, Nav2, and base arbitration.

### Selected: three-package split

Separate task decisions, control execution, and system assembly. This provides
clear replaceable boundaries and makes the user-facing launch package free of
controller or state-machine logic.

## 3. Package Responsibilities

### `sm_task_orchestrator`

Owns:

- `pick_place_task_manager.py` and the Pick/Place state machine.
- `navigation_geometry.py` and the unchanged Nav2 goal/standoff calculation.
- Natural-language task consumption through `/pick_place_task`.
- Perception/grasp target selection and caching.
- Nav2 action requests, hybrid handoff decisions, safe retreat sequencing,
  `/base_control_mode`, and `/base_control_blend` publication.
- The current task-manager unit tests.

Does not own:

- Final `/cmd_vel` arbitration.
- Joint or gripper command generation.
- Controller gains or whole-body kinematics.
- Full-system launch composition.

The runtime node and executable names remain `pick_place_task_manager`.

### `sm_ee_wholebody_control`

Owns:

- `arm_yaw_rho_z_position_controller` and the other retained arm/whole-body
  controller executables.
- End-effector target tracking, arm/base switching, and manipulation velocity
  publication on `/cmd_vel_manipulation`.
- Joint and gripper command publication.
- Target and camera TF bridges used by control.
- Controller configurations including `arm_position_target_in.yaml` and
  `wholebody_paper_switching.yaml`.
- Controller-focused launch files and tests.
- Control-specific monitors and probes that are still referenced or useful for
  diagnosis.

Does not own:

- Pick/Place phase decisions.
- Nav2 server startup.
- Perception or grasp-model startup.
- Final `/cmd_vel` ownership.

Existing node names, executable names, topics, frames, and parameters remain
unchanged; only their ROS package owner changes.

### `sm_bringup`

Owns:

- The complete natural-language Pick-and-Place launch graph.
- Integration launch arguments, deployment defaults, parameter composition,
  and package-to-package wiring.
- Full-system operator documentation and launch-contract tests.
- Transitional launch filename compatibility inside the new package.

Contains no Python runtime node and no control or task-state logic.

The canonical operator command becomes:

```bash
ros2 launch sm_bringup natural_language_pick_place.launch.py
```

`sm_bringup/florence_long_range_pick_place_control.launch.py` remains as a
thin compatibility include during this migration so the familiar launch
filename is still available under the new package.

## 4. File Ownership Map

| Current content | Final owner |
|---|---|
| `pick_place_task_manager.py` | `sm_task_orchestrator` |
| `navigation_geometry.py` | `sm_task_orchestrator` |
| Task-manager and navigation-geometry tests | `sm_task_orchestrator` |
| Arm and whole-body controller modules | `sm_ee_wholebody_control` |
| Target/TF bridges used by controllers | `sm_ee_wholebody_control` |
| Controller YAML files | `sm_ee_wholebody_control` |
| Controller-only launches and tests | `sm_ee_wholebody_control` |
| Full perception/grasp/navigation/task/control launches | `sm_bringup` |
| Full-system launch arguments and integration tests | `sm_bringup` |
| Operator-facing Pick-and-Place README | `sm_bringup` |

Historical design and plan documents are retained as archived evidence. Any
executable-looking legacy paths receive an archive warning and a pointer to
the new package owner.

## 5. Runtime Data Flow

```text
sm_natural_language_task
        | /pick_place_task
        v
sm_task_orchestrator
        | task commands / targets / base mode and blend
        +----------------------+-----------------------+
                               |                       |
                               v                       v
                    sm_navigation_nav2     sm_ee_wholebody_control
                    /cmd_vel_navigation    /cmd_vel_manipulation
                               |                       |
                               +-----------+-----------+
                                           v
                              sm_base_control_manager
                                           |
                                           v
                                       /cmd_vel
```

`sm_bringup` creates this graph but does not participate in the data path.
`sm_base_control_manager` remains the only final `/cmd_vel` publisher.

## 6. Compatibility Contract

The following remain unchanged:

- Runtime node names.
- Topic and action names and message types.
- QoS behavior.
- Frame names and TF relationships.
- Launch argument names and defaults.
- Controller gains, limits, offsets, speed values, and timing values.
- Natural-language prompts, model choice, aliases, and fallback behavior.
- Nav2 configuration and lifecycle behavior.
- DDS configuration; the untracked `ros2_ws/dds_setting/` directory remains
  outside the migration scope.

Expected intentional changes are limited to package names, Python import
paths, `FindPackageShare` owners, package dependencies, documentation commands,
and launch composition location.

## 7. Legacy `ee_switch_debug` Removal

Removal occurs only after all three new packages build and the full runtime
regression passes.

Each remaining file is handled by one of these rules:

1. Move active task logic to `sm_task_orchestrator`.
2. Move active or retained controller/diagnostic logic to
   `sm_ee_wholebody_control`.
3. Move integration-only launch/documentation to `sm_bringup`.
4. Delete a confirmed unused experimental file only in a separate cleanup
   commit after source, launch, setup, test, and documentation reference scans.

No file is deleted merely because its name contains `debug`. The package is
removed only when no runtime or test reference remains.

## 8. Migration Sequence

1. Capture the current package, launch-argument, node, topic, and executable
   baseline.
2. Create `sm_task_orchestrator` and move the task manager plus navigation
   geometry without implementation edits.
3. Create `sm_ee_wholebody_control` and move controller code, configuration,
   retained diagnostics, and focused tests without tuning.
4. Create `sm_bringup`, move the full-system launch graph, and replace only
   package/resource owner references.
5. Build and test all packages sequentially.
6. Run the Isaac Sim red-can and orange consecutive-task regression and verify
   safe retreat and sole `/cmd_vel` ownership.
7. Audit remaining legacy files, perform separately reviewable cleanup, and
   remove `ee_switch_debug`.
8. Rebuild and rerun the full regression after legacy removal.

## 9. Verification Gates

- Focused tests pass immediately after each ownership move.
- The complete source suite remains at least the current 90 passing tests;
  moved tests are counted from their new owners rather than duplicated.
- `colcon build --executor sequential --packages-up-to sm_bringup` succeeds.
- Installed executables resolve from their intended new packages.
- `ros2 launch sm_bringup natural_language_pick_place.launch.py --show-args`
  preserves the recorded defaults.
- The active runtime graph contains no `ee_switch_debug` executable owner.
- `navigation_cmd_mux` is the sole final `/cmd_vel` publisher.
- Korean red-can and orange commands parse identically and both reach `DONE`.
- The consecutive task enters `SAFE_RETREAT`/`RETREAT` before remote travel.

## 10. Failure and Rollback

Each package extraction and legacy cleanup is committed separately. A failed
gate stops the migration at the last passing commit; behavior tuning is not
used to compensate for a packaging failure. The already verified `main`
commit `f8a7122` remains the rollback baseline.

## 11. Out of Scope

- Top-down grasp integration.
- Controller speed, gain, offset, or timing changes.
- New collision avoidance or grasp-success logic.
- Runtime hot-swapping.
- Renaming perception, grasping, language, navigation, or base-manager
  packages.
- DDS cleanup.
