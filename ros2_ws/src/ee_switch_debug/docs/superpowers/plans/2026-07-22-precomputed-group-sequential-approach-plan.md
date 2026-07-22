# Precomputed Joint-Group Sequential Approach Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace simultaneous six-joint APPROACH with measured, ordered YAW → ARM_POSITION → WRIST_ALIGN execution while preserving the validated fixed-orientation DESCEND and all later stages.

**Architecture:** Keep the existing whole-arm fixed-orientation planner as a feasibility oracle. Build group-specific approach endpoints with a restricted Joint 2/3 rho-Z solver, densely validate the actual interpolated group paths, then execute each group as one bounded compact trajectory followed by measured pregrasp pose verification.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble (`rclpy`), pytest, colcon.

## Global Constraints

- Whole-arm six-axis interpolation must not execute during approach.
- Stage order is `YAW -> ARM_POSITION -> WRIST_ALIGN -> PREGRASP_VERIFY -> DESCEND`.
- YAW changes Joint 1 only; ARM_POSITION changes Joint 2/3 only; WRIST_ALIGN keeps Joint 1 fixed and changes Joint 2 through Joint 6.
- DESCEND, GRASP, LIFT, 6 cm continuous HOME handoff, frozen perception, limits, and tolerances remain unchanged.
- No automatic ROS launch during verification.

---

### Task 1: Restricted Joint 2/3 approach planner

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/top_down_kinematics.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_top_down_kinematics.py`

**Interfaces:**
- Extend `TopDownPlan` with `yaw_joints` and `arm_joints` six-element arrays.
- Produce `solve_joint23_rho_z(target_position, seed, lower_limits, upper_limits) -> IKResult`.
- Produce `build_group_sequential_approach(fixed_plan, current_joints, lower_limits, upper_limits, minimum_clearance) -> TopDownPlan`.

- [ ] **Step 1: Write failing restricted-solver tests**

Add tests proving that the restricted solver changes only indices 1 and 2,
converges to the pregrasp rho/Z generated from a known fixed plan, and rejects
an unreachable target without exceeding limits.

```python
def test_joint23_solver_keeps_yaw_and_wrist_fixed():
    seed = np.array([0.2, -1.2, 1.8, -0.4, 1.0, -0.7])
    target_joints = np.array([0.2, -0.8, 1.3, -0.4, 1.0, -0.7])
    target_position, _ = forward_kinematics(target_joints)
    result = solve_joint23_rho_z(
        target_position, seed, [-3.14] * 6, [3.14] * 6
    )
    assert result.success
    assert np.allclose(result.joints[[0, 3, 4, 5]], seed[[0, 3, 4, 5]])
    solved_position, _ = forward_kinematics(result.joints)
    assert abs(np.hypot(*solved_position[:2]) - np.hypot(*target_position[:2])) < 0.005
    assert abs(solved_position[2] - target_position[2]) < 0.005
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_top_down_kinematics.py -k 'joint23 or group_sequential'
```

Expected: import failure for the new interfaces.

- [ ] **Step 3: Implement damped restricted IK**

Use a numerical 2x2 Jacobian over rho and Z with only Joint 2/3 perturbations,
damped least squares, a maximum `0.12 rad` iteration step, line search, hard
clipping, and `0.005 m` rho/Z tolerances. Return a full six-joint result while
never altering Joint 1 or Joint 4 through Joint 6.

- [ ] **Step 4: Build and validate group endpoints**

Create `yaw_joints` by copying measured start and replacing Joint 1 with the
fixed-plan pregrasp Joint 1. Solve `arm_joints` from that yaw endpoint toward the
fixed-plan pregrasp rho/Z. Keep measured-start wrist angles fixed. Validate 31
samples of YAW, ARM_POSITION, and the actual WRIST_ALIGN interpolation from
`arm_joints` to fixed-plan `pregrasp_joints`; reject limit violations, Joint 1
motion after YAW, or EE height below grasp FK Z plus `minimum_clearance`.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run Step 2 again. Expected: all selected planner tests pass.

### Task 2: Ordered semantic group execution

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Produce `semantic_top_down_stage_goal` support for `YAW`, `ARM_POSITION`, and `WRIST_ALIGN`.
- Produce `measured_pregrasp_pose_valid(current_joints, plan, position_tolerance, orientation_tolerance) -> bool`.
- Execute ordered stages before existing DESCEND.

- [ ] **Step 1: Write failing stage-goal and mask tests**

```python
def test_group_sequential_stage_goals_and_masks():
    plan = SimpleNamespace(
        yaw_joints=np.arange(6),
        arm_joints=np.arange(6) + 10,
        pregrasp_joints=np.arange(6) + 20,
    )
    assert np.array_equal(semantic_top_down_stage_goal("YAW", plan, []), plan.yaw_joints)
    assert np.array_equal(semantic_top_down_stage_goal("ARM_POSITION", plan, []), plan.arm_joints)
    assert np.array_equal(semantic_top_down_stage_goal("WRIST_ALIGN", plan, []), plan.pregrasp_joints)
    assert top_down_stage_mask("YAW") == [True, False, False, False, False, False]
    assert top_down_stage_mask("ARM_POSITION") == [False, True, True, False, False, False]
    assert top_down_stage_mask("WRIST_ALIGN") == [False, True, True, True, True, True]
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -k 'group_sequential or measured_pregrasp'
```

Expected: failures for the new stage names and verification function.

- [ ] **Step 3: Add stage policy and measured pose verification**

Map YAW, ARM_POSITION, and WRIST_ALIGN to their plan endpoints and the compact
trajectory profile. Use `forward_kinematics` and `rotation_distance` to verify
the measured EE against the planned pregrasp position and selected rotation.
Unknown stages remain conservative.

- [ ] **Step 4: Write failing ordered-transition tests**

Construct a controller with a valid group plan and monkeypatch semantic command
completion. Assert exactly these transitions:

```text
YAW -> ARM_POSITION -> WRIST_ALIGN -> PREGRASP_VERIFY
PREGRASP_VERIFY(valid) -> DESCEND
PREGRASP_VERIFY(invalid) -> remains PREGRASP_VERIFY with open gripper
```

Also assert every normal transition clears velocity, preserving the existing
full stop before DESCEND.

- [ ] **Step 5: Route the group stages**

When group sequencing is enabled, initialize the plan at YAW. In the semantic
executor, run YAW, ARM_POSITION, and WRIST_ALIGN through the existing compact
bounded segment command. Advance only on measured completion. PREGRASP_VERIFY
does not command motion; it advances only when measured position/orientation
passes. Existing DESCEND and later code is reused without changes.

- [ ] **Step 6: Run controller tests and verify GREEN**

Run Step 2 again. Expected: selected tests pass.

### Task 3: Dedicated launch activation and regression protection

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/config/arm_position_target_in.yaml`
- Modify: `ros2_ws/src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Produce ROS parameter `top_down_use_group_sequential_approach`, default false and true in the dedicated launch.

- [ ] **Step 1: Write failing launch activation test**

```python
assert '"top_down_use_group_sequential_approach": True' in source
```

- [ ] **Step 2: Run launch test and verify RED**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py
```

Expected: failure because the parameter is absent.

- [ ] **Step 3: Add parameter and planner routing**

Declare/load the parameter, add it to YAML, enable it in the dedicated launch,
and call `build_group_sequential_approach` immediately after the successful
fixed-orientation plan. A failed group plan enters `PLAN_FAILED`; it must not
fall back to simultaneous APPROACH.

- [ ] **Step 4: Update README and run launch test GREEN**

Document active joint groups, the role of six-axis IK as validator only, and the
unchanged post-pregrasp sequence. Run Step 2 and expect PASS.

### Task 4: Full verification and PR update

**Files:**
- No additional source files.

- [ ] **Step 1: Run full tests and build**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test
colcon build --symlink-install --packages-select ee_switch_debug \
  --build-base build_codex --install-base install_codex
git diff --check
```

- [ ] **Step 2: Commit and push the scoped package changes**

Stage only `ros2_ws/src/ee_switch_debug`, commit with a terse description, and
push `agent/hybrid-wholebody-updates` so Draft PR #9 is updated. Leave all
workspace videos, build artifacts, models, and backups untracked.

- [ ] **Step 3: User-run simulator acceptance**

Require the task-state trace `YAW -> ARM_POSITION -> WRIST_ALIGN ->
PREGRASP_VERIFY -> DESCEND`, no motion of inactive joint groups, no diagonal
descent, equivalent grasp success, and the existing continuous LIFT-to-HOME
behavior. Do not claim equal physical performance before inspecting the video.
