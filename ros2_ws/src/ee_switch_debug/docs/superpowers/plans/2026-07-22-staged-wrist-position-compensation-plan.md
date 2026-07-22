# Staged Wrist Position Compensation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Break the WRIST-recovery deadlock by allowing slow wrist progress while link2 and link3 perform bounded position compensation.

**Architecture:** Preserve the existing `POSITION -> WRIST -> PREGRASP -> DESCEND` state machine. Give the wrist a small velocity allowance during `RECOVER`, and use absolute configured safety bounds for link2 and link3 from `WRIST` through the remaining PICK stages so recovery exit cannot snap commands back into the earlier envelope; all other joints retain the PICK-start-relative envelope.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, `pytest`, `colcon`.

## Global Constraints

- Keep the frozen grasp target unchanged throughout PICK.
- Never exceed `joint_lower_limits` or `joint_upper_limits`.
- Set the recovery wrist velocity limit to `0.08 rad/s`.
- Keep the existing position and wrist alignment gates before descent.
- Do not change perception, grasp generation or PLACE behavior.

---

### Task 1: Specify Nonblocking Recovery Motion

**Files:**
- Modify: `src/ee_switch_debug/test/test_wrist_orientation.py`
- Test: `src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `ArmYawRhoZPositionController.apply_stage_velocity_limits()` and `active_joint_bounds()`.
- Produces: regression expectations for slow wrist motion and post-POSITION link2/link3 range.

- [ ] **Step 1: Change the existing recovery-speed expectation**

Rename the recovery test to `test_wrist_recovery_keeps_slow_wrist_motion_with_moderate_position_speed` and expect:

```python
assert limited == [0.22, 0.30, 0.42, 0.08, 0.08, 0.08]
```

- [ ] **Step 2: Add recovery-bound tests**

```python
def test_wrist_stage_uses_hard_bounds_for_link2_and_link3_only():
    node = make_stage_limiter()
    assert node.active_joint_bounds(1) == (-1.0, 1.0)
    assert node.active_joint_bounds(2) == (-1.0, 1.0)
    assert node.active_joint_bounds(0) == (-0.8, 0.8)
    assert node.active_joint_bounds(3) == (-0.6, 0.6)
    node.pick_approach_stage = "POSITION"
    assert node.active_joint_bounds(1) == (-0.8, 0.8)
    assert node.active_joint_bounds(2) == (-0.8, 0.8)
```

- [ ] **Step 3: Run focused tests and verify RED**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -k 'wrist_recovery_keeps_slow or wrist_stage_uses_hard'
```

Expected: the speed test receives zero wrist limits and the bounds test
receives PICK-start-relative bounds for link2 and link3 in `WRIST`.

### Task 2: Implement the Recovery Motion Policy

**Files:**
- Modify: `src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `src/ee_switch_debug/config/arm_position_target_in.yaml`
- Test: `src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: recovery state and the configured joint bounds.
- Produces: `wrist_recovery_wrist_velocity` and nonblocking `RECOVER` commands.

- [ ] **Step 1: Add and load the recovery wrist parameter**

Declare `wrist_recovery_wrist_velocity` with default `0.08`, load it into the
fourth element of `stage_velocity_limits["RECOVER"]`, and set the YAML value to
`0.08`.

- [ ] **Step 2: Stop replacing recovery wrist commands with zeros**

Remove the branch that assigns `[0.0, 0.0, 0.0]` to `wrist_velocities` during
recovery. The `RECOVER` stage limiter now provides the required small cap.

- [ ] **Step 3: Select recovery-only link2/link3 bounds**

In `active_joint_bounds(index)`, skip the PICK-start-relative excursion clamp
for link2 and link3 after the controller leaves `POSITION`. Keep the absolute
configured hard lower and upper limits active through `WRIST`, `PREGRASP` and
`HOLD` so recovery exit cannot cause a command jump.

- [ ] **Step 4: Run focused tests and verify GREEN**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/ee_switch_debug/test/test_wrist_orientation.py
```

Expected: every wrist-orientation test passes.

### Task 3: Verify Regression Safety and Build

**Files:**
- Verify: `src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Verify: `src/ee_switch_debug/config/arm_position_target_in.yaml`
- Verify: `src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: the recovery policy from Task 2.
- Produces: package-level test and build evidence plus runtime instructions.

- [ ] **Step 1: Run the package test suite**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/ee_switch_debug/test
```

Expected: all tests pass with no new failures.

- [ ] **Step 2: Build the package**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ee_switch_debug
```

Expected: `ee_switch_debug` finishes successfully.

- [ ] **Step 3: Check the diff**

```bash
git diff --check
git diff -- \
  src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py \
  src/ee_switch_debug/config/arm_position_target_in.yaml \
  src/ee_switch_debug/test/test_wrist_orientation.py
```

Expected: no whitespace errors and only recovery-policy changes beyond the
existing uncommitted staged-controller work.
