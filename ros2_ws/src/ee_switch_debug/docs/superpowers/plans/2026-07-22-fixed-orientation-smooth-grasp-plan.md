# Fixed-Orientation Smooth Grasp Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the 183-second rotating waypoint grasp with a fixed-orientation, fixed-base semantic trajectory expected to complete in approximately 15-25 seconds.

**Architecture:** The pure kinematics planner searches for the closest-to-top-down rotation reachable at both pre-grasp and grasp, then validates one fixed-orientation descent. A separate pure trajectory module time-parameterizes APPROACH, DESCEND, LIFT, and HOME with quintic smoothstep segments; the ROS controller executes those endpoints without intermediate settle events or the separate fast RETURN_HOME controller.

**Tech Stack:** Python 3.10, NumPy, ROS 2 Humble `rclpy`, `pytest`, `colcon`

## Global Constraints

- The mobile base remains stopped for the complete sequence.
- The first valid post-`PICK` target remains immutable until completion or `RESET`.
- Pre-grasp clearance remains 0.10 m in world Z.
- One selected EE orientation is used at both pre-grasp and grasp and throughout DESCEND and LIFT.
- Selected orientation fraction must be at least 0.50 toward the requested orientation.
- Sampled descent XY deviation must be at most 0.015 m, orientation deviation at most 0.035 rad, and Z must decrease monotonically.
- Every endpoint and sample remains inside configured hard joint limits and PICK excursion bounds.
- Segment duration enforces configured velocity and acceleration limits using quintic derivative bounds.
- Any planning failure holds the open gripper in `PICK:PLAN_FAILED`.

---

### Task 1: Common fixed-orientation grasp planner

**Files:**
- Modify: `ee_switch_debug/top_down_kinematics.py`
- Modify: `test/test_top_down_kinematics.py`

**Interfaces:**
- Produces: `FixedOrientationCandidate`
- Produces: `find_common_reachable_orientation(...) -> FixedOrientationCandidate`
- Produces: `validate_fixed_orientation_descent(...) -> tuple[bool, str]`
- Produces: `plan_fixed_orientation_top_down_sequence(...) -> TopDownPlan`
- Extends: `TopDownPlan.grasp_joints`, `TopDownPlan.selected_rotation`, and `TopDownPlan.orientation_fraction`

- [ ] **Step 1: Write failing live-target tests.**

Add tests that require the live target to select a fraction at least `0.75`, require pre-grasp and grasp FK orientations to differ by at most `0.035 rad`, and require sampled joint interpolation to stay within `0.015 m` XY deviation with monotonically decreasing Z.

- [ ] **Step 2: Run the focused tests and verify RED.**

Run:

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_top_down_kinematics.py -k fixed_orientation
```

Expected: import or assertion failure because the fixed-orientation planner does not exist.

- [ ] **Step 3: Implement common-orientation search.**

Search fractions from `1.0` down to `minimum_orientation_fraction` using `orientation_search_steps + 1` samples. For each fraction, solve pre-grasp IK from the structured seed, solve grasp IK from the pre-grasp result with the same rotation, and select the first candidate whose endpoints converge.

- [ ] **Step 4: Implement dense descent validation and build the plan.**

Sample 21 joint-interpolated poses between endpoint solutions. Reject non-monotonic Z, XY deviation above `0.015 m`, orientation deviation above `0.035 rad`, or joint-limit violations. Store only semantic endpoints; do not create stop-and-settle descent waypoints.

- [ ] **Step 5: Run all kinematics tests and verify GREEN.**

Run:

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_top_down_kinematics.py
```

Expected: all tests pass.

### Task 2: Quintic semantic joint trajectories

**Files:**
- Create: `ee_switch_debug/semantic_joint_trajectory.py`
- Create: `test/test_semantic_joint_trajectory.py`

**Interfaces:**
- Produces: `quintic_smoothstep(fraction: float) -> float`
- Produces: `minimum_quintic_duration(start, goal, velocity_limits, acceleration_limit, minimum_duration) -> float`
- Produces: `sample_quintic_joint_positions(start, goal, elapsed, duration) -> tuple[np.ndarray, bool]`

- [ ] **Step 1: Write failing tests for endpoints, midpoint, duration, and speed bounds.**

Tests require exact endpoints, `s(0.5) == 0.5`, zero numerical endpoint velocity, a duration of at least `1.875 * abs(delta_i) / velocity_limit_i`, and at least `sqrt(5.7736 * abs(delta_i) / acceleration_limit)`.

- [ ] **Step 2: Run the new test file and verify RED.**

Run:

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_semantic_joint_trajectory.py
```

Expected: import failure because the module does not exist.

- [ ] **Step 3: Implement the minimal pure trajectory functions.**

Clamp fractions to `[0, 1]`; calculate duration from the exact quintic maximum normalized velocity `1.875` and conservative maximum normalized acceleration `5.7736`; return the endpoint with `complete=True` when elapsed is at least duration.

- [ ] **Step 4: Run the trajectory tests and verify GREEN.**

### Task 3: Semantic endpoint executor

**Files:**
- Modify: `ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `TopDownPlan.pregrasp_joints` and `TopDownPlan.grasp_joints`
- Consumes: trajectory functions from Task 2
- Produces state sequence: `PICK:APPROACH -> PICK:DESCEND -> PICK:GRASP -> PICK:LIFT -> PICK:HOME -> PICK:HOLD`

- [ ] **Step 1: Write failing controller tests.**

Require fixed planner selection, semantic stages to use all six joints, reset to clear segment start/goal/start-time/stage, LIFT to target pre-grasp rather than reversed waypoints, and HOME to use the same semantic segment executor.

- [ ] **Step 2: Run focused tests and verify RED.**

Run:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py -k 'fixed or semantic or top_down'
```

- [ ] **Step 3: Add fixed-orientation parameters and planner selection.**

Declare/read `top_down_use_fixed_reachable_orientation`, `top_down_minimum_orientation_fraction`, `top_down_orientation_search_steps`, `top_down_descend_max_xy_deviation`, `top_down_descend_max_orientation_deviation`, and `top_down_segment_min_duration`.

- [ ] **Step 4: Add reusable semantic segment state.**

Store `top_down_segment_stage`, `top_down_segment_start`, `top_down_segment_goal`, `top_down_segment_start_time`, and `top_down_segment_duration`. On first entry initialize from measured joints; on each tick sample the quintic trajectory and publish positions directly; after planned time hold the endpoint until all measured joints are within `top_down_joint_tolerance`.

- [ ] **Step 5: Replace waypoint execution with semantic endpoints.**

Use pre-grasp for APPROACH, grasp for DESCEND, pre-grasp for LIFT, and configured home joints for HOME. Transition to HOLD only after measured home arrival. Do not enter the separate `RETURN_HOME` controller for this mode.

- [ ] **Step 6: Run controller tests and verify GREEN.**

### Task 4: Faster safe launch configuration and documentation

**Files:**
- Modify: `config/arm_position_target_in.yaml`
- Modify: `launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `test/test_yoloe_top_down_grasp_launch.py`
- Modify: `README_PICK_PLACE.md`

**Interfaces:**
- Enables the fixed-orientation planner only in the dedicated test launch.
- Configures alignment limits `[0.55, 0.85, 1.00, 0.80, 0.80, 0.80]` rad/s.
- Configures path limits `[0.35, 0.45, 0.55, 0.45, 0.35, 0.45]` rad/s.
- Configures `top_down_stage_acceleration: 1.2` rad/s² and `top_down_segment_min_duration: 0.8` s.

- [ ] **Step 1: Write failing launch assertions for fixed mode and faster limits.**

- [ ] **Step 2: Run the launch test and verify RED.**

- [ ] **Step 3: Enable fixed mode and update limits.**

Keep the generic configuration disabled by default. The dedicated launch explicitly enables fixed orientation, disables the old orientation-blended descent, and supplies all validation thresholds and faster bounded motion values.

- [ ] **Step 4: Update the README state sequence and safety notes.**

Document the reachable tilt, constant descent/lift orientation, semantic trajectory stages, expected 15-25 second range, and the fact that visual naturalness still requires a new recording.

- [ ] **Step 5: Run all package tests.**

Run:

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug:src/sm_florence_2_vlm_ros2:$PYTHONPATH \
  /usr/bin/python3 -m pytest -q src/ee_switch_debug/test
```

- [ ] **Step 6: Build the package.**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ee_switch_debug --symlink-install
```

Expected: package build succeeds with no errors.

- [ ] **Step 7: Hand off simulator commands.**

Do not start the simulator launch automatically. Provide the launch, target, `PICK`, `RESET`, and state-monitor commands so the user can record the next run.
