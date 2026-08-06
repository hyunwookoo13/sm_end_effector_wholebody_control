# Modular ROS 2 Packaging and Orchestration Design

## 1. Objective

Reorganize the current ROS 2 workspace into replaceable capability packages without changing the behavior of the working natural-language pick-and-place system.

The first release supports module replacement by selecting a launch profile and restarting the system. Runtime hot-swapping is explicitly deferred. Existing node names, topics, actions, parameters, controller gains, QoS behavior, and DDS settings remain unchanged during the migration.

## 2. Design Principles

1. One package owns one capability boundary.
2. The task orchestrator depends on stable ROS interfaces, not implementation internals.
3. Navigation, perception, grasping, language, and manipulation implementations can be replaced independently.
4. Only one component publishes the final `/cmd_vel` command.
5. Package extraction and behavior changes are separate work. This migration does not tune control behavior.
6. Legacy files are deleted only after reference checks, tests, and an Isaac Sim end-to-end regression pass.
7. The untracked `dds_setting/` directory is preserved and remains outside cleanup scope unless separately requested.

## 3. Target Package Structure

### 3.1 Existing packages retained

#### `sm_florence_2_vlm_ros2`

- Owns Florence and YOLOE perception nodes and model-specific configuration.
- Publishes detections and ROI information.
- Keeps its current name during this migration to avoid unnecessary launch and dependency changes.
- A later design may split Florence and YOLOE into separate implementation packages.

#### `sm_grasping_ros2`

- Owns grasp inference, filtering, post-processing, and visualization.
- Consumes perception/depth data and publishes grasp candidates or the selected grasp pose.
- Keeps its current package name.

### 3.2 New packages extracted from `ee_switch_debug`

#### `sm_mobile_manipulation_interfaces`

- Owns project-specific messages, services, and actions only when standard ROS interfaces are insufficient.
- Provides versioned contracts for task commands, task state, target navigation, and manipulation execution.
- Contains no nodes or business logic.
- During the first extraction pass, current topic types remain available through compatibility adapters so behavior does not change abruptly.

#### `sm_natural_language_task`

- Owns `natural_language_task_parser` and `natural_language_task_console`.
- Converts free-form language into a structured pick/place task.
- Does not know which perception, navigation, or arm implementation will execute the task.
- Preserves the current Ollama/Gemma configuration and parser fallback behavior.

#### `sm_navigation_nav2`

- Is the Nav2 implementation adapter, not a copy or fork of Nav2.
- Owns Nav2 launch, `nav2_rolling_odom.yaml`, lifecycle startup, and Nav2-specific parameters.
- Accepts a navigation target through the stable navigation contract.
- Produces navigation state and `/cmd_vel_navigation`.
- Does not contain Pick/Place phase logic, grasp logic, or final `/cmd_vel` ownership.
- A future navigation implementation can replace this package by implementing the same contract.

#### `sm_wholebody_control`

- Owns the active arm and end-effector whole-body controllers.
- Produces joint position commands and `/cmd_vel_manipulation` for precision approach.
- Owns controller-specific safety limits, gains, switching functions, and TF target bridges.
- Does not start Nav2 or decide the global Pick/Place task phase.

#### `sm_base_control_manager`

- Owns `navigation_cmd_mux` and the final `/cmd_vel` publisher.
- Arbitrates navigation, manipulation, retreat, and stop commands.
- Consumes `/base_control_mode` and `/base_control_blend` from the orchestrator.
- Retains command timeouts, slew-rate limits, and the rule that stale input produces zero velocity.
- Remains unchanged when either the navigation or precision-control implementation is replaced.

#### `sm_pick_place_orchestrator`

- Owns `pick_place_task_manager` and the Pick/Place state machine.
- Coordinates perception targets, navigation requests, manipulation commands, safe retreat, and task completion.
- Depends only on declared ROS interfaces and topics.
- Does not launch model implementations or directly publish final base velocity.

#### `sm_system_bringup`

- Owns full-system launch files and deployment profiles.
- Selects implementations with launch arguments such as `navigation_backend:=nav2`, `perception_backend:=yoloe`, and `language_backend:=gemma`.
- Owns integration-level frame names, topic remaps, and deployment parameter composition.
- Does not contain control or task-state logic.

#### `sm_debug_tools` (conditional)

- Retains only diagnostic nodes that are still actively used.
- Candidate contents include TF inspection, joint-effect probes, monitors, loggers, and plotting utilities.
- Unreferenced experimental controllers and duplicate launch files are deleted instead of being preserved automatically.

## 4. Runtime Data Flow

```text
Natural-language input
        |
        v
sm_natural_language_task
        | structured task
        v
sm_pick_place_orchestrator
        |---------------------> perception target selection
        |---------------------> grasp request / grasp target
        |---------------------> navigation target
        |---------------------> arm Pick/Place command
        |---------------------> base mode and blend
                                  |
              +-------------------+-------------------+
              |                                       |
              v                                       v
      sm_navigation_nav2                     sm_wholebody_control
      /cmd_vel_navigation                    /cmd_vel_manipulation
              |                                       |
              +-------------------+-------------------+
                                  v
                       sm_base_control_manager
                                  |
                                  v
                              /cmd_vel
```

The orchestrator selects modes but never blends velocity itself. The base-control manager owns the command arbitration boundary, preventing multiple backends from competing for `/cmd_vel`.

## 5. Navigation Replacement Contract

The Nav2 adapter has one responsibility: move the robot to a requested planar target while maintaining the requested standoff and report progress or completion.

The contract includes:

- Target pose and target frame.
- Requested standoff distance.
- Start, cancel, success, failure, and timeout states.
- Navigation velocity output on `/cmd_vel_navigation`.
- No direct access to arm state-machine internals.

For the first migration, the existing `nav2_msgs/action/NavigateToPose` behavior and topic names are retained. A project-specific target-navigation action is introduced only through a compatibility adapter and only after the package move is proven behavior-equivalent.

During the Phase 1 file move, the existing standoff calculation remains in the orchestrator so the generated Nav2 goal is byte-for-byte equivalent to the baseline. Ownership moves behind the navigation adapter only in Phase 3, after trace comparison tests prove that the adapter produces the same goal pose.

## 6. Launch Profiles

Initial profiles are restart-based:

- `full_nav2_yoloe_gemma`: current complete system.
- `perception_only`: dual-camera YOLOE/Florence inspection.
- `grasping_only`: perception plus grasp generation.
- `manipulation_only`: fixed or supplied target with whole-body control.
- `navigation_only`: Nav2 adapter plus base-control manager.
- `simulation_full`: Isaac Sim integration with the current frames and simulation time.

Each profile starts only one implementation for each capability slot. Runtime backend replacement is not part of this phase.

## 7. Failure and Safety Behavior

- If a required action server or topic is unavailable, the orchestrator stays stopped and reports the missing capability.
- If an active command source becomes stale, `sm_base_control_manager` publishes zero velocity.
- Switching backends first publishes `STOP`; no profile starts two final velocity publishers.
- Navigation cancellation is completed or timed out before manipulation exclusively owns base motion.
- Existing rear-laser retreat checks and retreat limits are preserved.
- Package migration does not change object aliases, model prompts, grasp offsets, standoff values, joint gains, or timing values.

## 8. Migration Strategy

### Phase 0: Freeze the baseline

- Commit and tag the currently working state.
- Record the active node list, topic list, action list, parameters, and launch command.
- Record an Isaac Sim regression sequence for red can, orange, and apple tasks.

### Phase 1: Extract low-risk capability packages

1. Extract `sm_natural_language_task` while retaining topic names and parser behavior.
2. Extract `sm_navigation_nav2` launch/configuration without changing Nav2 parameters.
3. Extract `sm_base_control_manager` with the existing mux code and tests.

### Phase 2: Extract control and orchestration

1. Extract the active controller and target bridges into `sm_wholebody_control`.
2. Extract `pick_place_task_manager` into `sm_pick_place_orchestrator`.
3. Assemble the same runtime graph through `sm_system_bringup`.

### Phase 3: Establish versioned interfaces

- Introduce `sm_mobile_manipulation_interfaces` contracts and compatibility adapters under a separate interface-versioning design review.
- Keep existing public topics during a deprecation window.
- Add a second mock navigation backend to prove that Nav2 is replaceable without orchestrator changes.

### Phase 4: Remove legacy content

- Verify that no launch, test, setup entry point, or documentation references legacy files.
- Move retained diagnostics to `sm_debug_tools`.
- Delete obsolete controllers, duplicate launch files, caches, and the empty `ee_switch_debug` package.
- Do not delete `dds_setting/`.

## 9. Verification Gates

Every extraction step must pass all applicable gates before the next package is moved:

1. `colcon build` succeeds for the affected packages and their dependents.
2. Unit tests from the original package pass from their new package locations.
3. Launch contract tests confirm required nodes, topics, actions, and parameters.
4. There is exactly one final `/cmd_vel` publisher.
5. Recorded command/state traces are behaviorally equivalent to the baseline.
6. Isaac Sim completes the current natural-language red-can and orange Pick-and-Place sequences.
7. Apple behavior is not required to improve during packaging, but it must not regress relative to the recorded baseline.

## 10. Deletion Criteria

A file or entry point may be deleted only if all of the following are true:

- No source, launch, test, package manifest, or documentation references it.
- It is not part of the recorded working launch graph.
- An equivalent capability is not required by a retained diagnostic profile.
- The full workspace builds and tests successfully without it.
- The end-to-end simulation regression remains at baseline behavior.

Deletion happens in dedicated commits so it can be reverted independently from package extraction.

## 11. Out of Scope

- Runtime hot-swapping of navigation or perception backends.
- Controller tuning or speed changes.
- Top-down grasp integration.
- New collision-avoidance or grasp-success logic.
- Renaming `sm_grasping_ros2` or `sm_florence_2_vlm_ros2`.
- DDS configuration cleanup.
