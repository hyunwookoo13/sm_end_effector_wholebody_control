# Current-Distance Blended Top-Down Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Pick the live can at the current base distance by following one precomputed synchronized approach and an orientation-blended 10 cm descent.

**Architecture:** Extend the pure OMY kinematics planner with radial target compensation, quaternion interpolation, a synchronized joint-space approach, and Cartesian descent waypoints. The ROS controller selects this planner only in the top-down test launch and executes its immutable approach waypoints before the existing grasp, lift, and home phases.

**Tech Stack:** Python 3.10, NumPy, ROS 2 Humble `rclpy`, `pytest`, `colcon`

## Global Constraints

- The mobile base must not move.
- The pre-grasp TCP height remains 0.10 m above the compensated grasp target.
- The planner applies 0.025 m inward radial compensation to a grasp stream that already contains 0.04 m outward compensation.
- Position and orientation are blended only in a precomputed path; no target-dependent online correction is allowed after latching.
- Every waypoint must remain inside configured hard joint limits.
- Final position error must be at most 0.005 m and final orientation error at most 0.03 rad.
- Any planning failure must hold the arm in `PICK:PLAN_FAILED`.

---

### Task 1: Blended current-distance kinematics planner

**Files:**
- Modify: `ee_switch_debug/top_down_kinematics.py`
- Modify: `test/test_top_down_kinematics.py`

**Interfaces:**
- Produces: `interpolate_rotation(start, end, fraction) -> np.ndarray`
- Produces: `apply_inward_radial_offset(position, offset) -> np.ndarray`
- Produces: `structured_pregrasp_seed(current_joints, target_position) -> np.ndarray`
- Produces: `interpolate_joint_waypoints(start, end, maximum_step) -> list[np.ndarray]`
- Produces: `solve_blended_descent_waypoints(start_position, end_position, start_rotation, end_rotation, seed, lower_limits, upper_limits, waypoint_spacing) -> list[np.ndarray]`
- Produces: `validate_and_build_plan(grasp_position, approach_waypoints, pregrasp_joints, descent_waypoints, minimum_approach_clearance) -> TopDownPlan`
- Produces: `plan_blended_top_down_sequence(grasp_position, grasp_rotation, current_joints, lower_limits, upper_limits, clearance, radial_inward_offset, waypoint_spacing, approach_joint_step, minimum_approach_clearance) -> TopDownPlan`
- Extends: `TopDownPlan.approach_waypoints: list[np.ndarray]`

- [ ] **Step 1: Write a failing live-target regression test.**

```python
def test_live_current_distance_target_has_safe_blended_plan():
    plan = plan_blended_top_down_sequence(
        grasp_position=[0.399, -0.194, 0.295],
        grasp_rotation=LIVE_MARKER_ROTATION,
        current_joints=LIVE_HOME_JOINTS,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        waypoint_spacing=0.01,
    )
    assert plan.success, plan.message
    assert plan.approach_waypoints
    assert len(plan.descent_waypoints) == 10
```

- [ ] **Step 2: Run the test and verify import failure for `plan_blended_top_down_sequence`.**

Run: `PYTHONPATH=src/ee_switch_debug /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_top_down_kinematics.py -k live_current_distance`

- [ ] **Step 3: Add interpolation, compensation, structured pre-grasp seeding and waypoint validation.**

```python
def plan_blended_top_down_sequence(
    grasp_position, grasp_rotation, current_joints, lower_limits, upper_limits,
    *, clearance=0.10, radial_inward_offset=0.025,
    waypoint_spacing=0.01, approach_joint_step=0.12,
    minimum_approach_clearance=0.08,
) -> TopDownPlan:
    compensated = apply_inward_radial_offset(
        grasp_position, radial_inward_offset
    )
    _, current_rotation = forward_kinematics(current_joints)
    pregrasp_position = compensated + np.array([0.0, 0.0, clearance])
    seed = structured_pregrasp_seed(current_joints, compensated)
    pregrasp = solve_pose_ik(
        pregrasp_position, current_rotation, seed, lower_limits, upper_limits
    )
    if not pregrasp.success:
        return TopDownPlan(False, message=f"pregrasp {pregrasp.message}")
    approach = interpolate_joint_waypoints(
        current_joints, pregrasp.joints, approach_joint_step
    )
    descent = solve_blended_descent_waypoints(
        pregrasp_position,
        compensated,
        current_rotation,
        grasp_rotation,
        pregrasp.joints,
        lower_limits,
        upper_limits,
        waypoint_spacing,
    )
    return validate_and_build_plan(
        compensated,
        approach,
        pregrasp.joints,
        descent,
        minimum_approach_clearance,
    )
```

- [ ] **Step 4: Test the radial correction, rotation endpoints, final accuracy, joint bounds and minimum approach height.**

- [ ] **Step 5: Run all kinematics tests and verify PASS.**

### Task 2: Immutable synchronized waypoint executor

**Files:**
- Modify: `ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `TopDownPlan.approach_waypoints` and `TopDownPlan.descent_waypoints`
- Produces: `PICK:APPROACH -> PICK:DESCEND -> PICK:GRASP -> PICK:LIFT -> HOME`

- [ ] **Step 1: Write failing tests for the `APPROACH` all-joint mask and one-shot blended planner selection.**

```python
def test_blended_approach_moves_the_precomputed_six_joint_waypoint():
    assert top_down_stage_mask("APPROACH") == [True] * 6
```

- [ ] **Step 2: Run focused controller tests and verify the new expectation fails.**

- [ ] **Step 3: Add `top_down_blend_orientation_during_descent` and `top_down_radial_inward_offset` parameters.**

- [ ] **Step 4: Select the blended planner once, execute approach waypoints, then reuse the existing descent/grasp/lift/home handling.**

- [ ] **Step 5: Confirm a latched blended plan never looks up or recomputes a target TF.**

- [ ] **Step 6: Run focused controller tests and verify PASS.**

### Task 3: Enable the live current-distance test and verify

**Files:**
- Modify: `config/arm_position_target_in.yaml`
- Modify: `launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `test/test_yoloe_top_down_grasp_launch.py`
- Modify: `README_PICK_PLACE.md`

**Interfaces:**
- Produces: launch defaults for blended planning, 0.025 m inward compensation and a hard-limit-compatible PICK excursion envelope.

- [ ] **Step 1: Write failing launch assertions for blended mode and 0.025 m radial compensation.**

- [ ] **Step 2: Enable the parameters and set the launch excursion envelope to `[0.65, 2.50, 2.30, 3.00, 3.00, 3.00]`.**

- [ ] **Step 3: Document that `APPROACH` is synchronized and precomputed rather than reactive.**

- [ ] **Step 4: Run all package tests and compile checks.**

Run: `PYTHONPATH=src/ee_switch_debug:src/sm_florence_2_vlm_ros2:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test`

- [ ] **Step 5: Build the package.**

Run: `colcon build --packages-select ee_switch_debug --symlink-install`

- [ ] **Step 6: Restart the top-down launch, publish `red can`, send one `PICK`, and monitor `/arm_task_state` plus controller logs at low speed.**
