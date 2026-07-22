# Continuous LIFT-to-HOME Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Transition from vertical LIFT to HOME as soon as measured EE clearance reaches 6 cm, without resetting the current lift velocity.

**Architecture:** A pure clearance predicate decides the handoff from measured forward-kinematics height. The semantic executor records the bounded velocity represented by consecutive LIFT position commands, and a specialized acceleration-limited HOME controller inherits that velocity only for the early LIFT-to-HOME path.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble (`rclpy`), pytest, colcon.

## Global Constraints

- Only `LIFT -> HOME` may preserve velocity.
- `APPROACH -> DESCEND` must continue clearing velocity and stopping completely.
- The handoff threshold is measured EE rise `0.06 m` above planned grasp EE height in `link0`.
- `return_home_after_pick == false` must preserve existing LIFT-to-HOLD behavior.
- Existing velocity limits, acceleration limit, joint-limit slowdown, gripper state, and HOME tolerance remain active.
- Do not start or stop the user's ROS launch during automated verification.

---

### Task 1: Measured-clearance policy and configuration

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`
- Modify: `ros2_ws/src/ee_switch_debug/config/arm_position_target_in.yaml`
- Modify: `ros2_ws/src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py`

**Interfaces:**
- Produces: `vertical_lift_clearance_reached(measured_ee_z, grasp_ee_z, required_clearance) -> bool`
- Produces: ROS parameter `top_down_lift_home_clearance`, default `0.06`.

- [ ] **Step 1: Write failing boundary and launch tests**

```python
def test_vertical_lift_clearance_uses_measured_height_boundary():
    assert not vertical_lift_clearance_reached(0.559, 0.500, 0.060)
    assert vertical_lift_clearance_reached(0.560, 0.500, 0.060)
    assert vertical_lift_clearance_reached(0.700, 0.500, 0.060)
```

Add this assertion to `test_yoloe_top_down_grasp_launch.py`:

```python
assert '"top_down_lift_home_clearance": 0.06' in source
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py \
  -k 'vertical_lift_clearance or lift_home_clearance'
```

Expected: import failure because the predicate does not exist.

- [ ] **Step 3: Implement the predicate and parameter**

```python
def vertical_lift_clearance_reached(
    measured_ee_z: float,
    grasp_ee_z: float,
    required_clearance: float,
) -> bool:
    clearance = max(0.0, float(required_clearance))
    return float(measured_ee_z) - float(grasp_ee_z) >= clearance - 1e-9
```

Declare `top_down_lift_home_clearance` with default `0.06`, load it with a
nonnegative clamp, add `top_down_lift_home_clearance: 0.06` to the YAML, and set
`"top_down_lift_home_clearance": 0.06` explicitly in the dedicated launch.

- [ ] **Step 4: Run focused tests and verify GREEN**

Run Step 2 again. Expected: selected tests pass.

### Task 2: Preserve bounded LIFT velocity and perform early handoff

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Produces: `advance_precomputed_top_down_stage(next_stage, preserve_velocity=False)`.
- Produces: `measured_lift_home_clearance_reached() -> bool` using existing `forward_kinematics`.
- Produces: `command_continuous_top_down_home(goal, dt) -> bool`.

- [ ] **Step 1: Write failing transition tests**

Add tests that construct the controller with `__new__`, stub the logger, and
verify the transition contract:

```python
def test_stage_transition_preserves_velocity_only_when_requested():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.top_down_stage = "LIFT"
    node.pick_approach_stage = "LIFT"
    node.previous_velocity = [0.1] * 6
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    node.clear_top_down_segment_state = lambda: None

    node.advance_precomputed_top_down_stage("HOME", preserve_velocity=True)
    assert node.previous_velocity == [0.1] * 6

    node.advance_precomputed_top_down_stage("DESCEND")
    assert node.previous_velocity == [0.0] * 6
```

Add a measured-clearance test by monkeypatching `controller.forward_kinematics`
to return `z=0.500` for planned grasp joints and `z=0.560` for measured joints.
Assert `measured_lift_home_clearance_reached()` is true at the boundary.

- [ ] **Step 2: Run transition tests and verify RED**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -k 'stage_transition_preserves_velocity or measured_lift_home_clearance'
```

Expected: failure because the preserve argument and measured helper do not exist.

- [ ] **Step 3: Record bounded trajectory velocity**

In `command_semantic_top_down_segment`, before replacing the six arm position
commands, copy their previous values. For every non-complete sampled point,
compute `(new_position - old_position) / max(dt, 1e-6)` and clamp each result to
the active joint velocity limit. Store that list in `previous_velocity` before
publishing the sampled positions.

- [ ] **Step 4: Add selective transition preservation and measured FK check**

Import `forward_kinematics`. Add `preserve_velocity: bool = False` to
`advance_precomputed_top_down_stage`; clear `previous_velocity` only when false.
Add `measured_lift_home_clearance_reached` which obtains current measured joints,
computes measured and planned-grasp EE positions with `forward_kinematics`, and
calls the pure boundary predicate.

- [ ] **Step 5: Add acceleration-limited continuous HOME control**

Implement `command_continuous_top_down_home` with
`fixed_target_joint_velocities`, all six active joints, alignment velocity
limits, `top_down_transit_joint_tolerance`, joint-limit slowdown, and
`limit_acceleration(..., max_acceleration=top_down_stage_acceleration)`. Integrate
the limited velocity into `position_command`; clear velocity only after measured
HOME alignment succeeds.

- [ ] **Step 6: Route only early LIFT-to-HOME through continuous control**

Add `top_down_continuous_home` state, clear it in RESET, and set it only when all
of these are true: stage is LIFT, `return_home_after_pick` is true, and measured
clearance is reached. At that point call
`advance_precomputed_top_down_stage("HOME", preserve_velocity=True)`. During HOME,
call `command_continuous_top_down_home` when the flag is true; otherwise keep the
existing semantic S-curve HOME behavior. Normal completed LIFT and every other
transition retain existing behavior.

- [ ] **Step 7: Run focused controller tests and verify GREEN**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -k 'lift_home or continuous_home or stage_transition_preserves_velocity'
```

Expected: selected tests pass.

### Task 3: Regression verification and operator documentation

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Documents the 6 cm measured-clearance handoff and simulator acceptance checks.

- [ ] **Step 1: Update README**

Document that LIFT remains vertical through 6 cm measured clearance, then HOME
inherits the bounded LIFT velocity; APPROACH-to-DESCEND still stops completely.

- [ ] **Step 2: Run full verification**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test
colcon build --symlink-install --packages-select ee_switch_debug \
  --build-base build_codex --install-base install_codex
git diff --check
```

Expected: zero test failures, successful `ee_switch_debug` build, and no diff
whitespace errors.

- [ ] **Step 3: User-run simulator acceptance**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ee_switch_debug
source install/setup.bash
ros2 launch ee_switch_debug yoloe_top_down_grasp_test.launch.py
```

The recording must show a vertical 6 cm lift followed by a curved, uninterrupted
HOME transition; no stop at clearance, no link1 snap, no collision, and no object
loss. Do not claim visual success until that recording is inspected.
