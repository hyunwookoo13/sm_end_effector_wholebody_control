# Group-Preserving Fully Blended Top-Down Approach Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace stop-gated YAW, ARM_POSITION, and WRIST_ALIGN execution with one safe, time-scaled trajectory that blends independently generated joint-group goals.

**Architecture:** A pure trajectory composer combines the existing Joint1 yaw endpoint, restricted Joint2/3 overhead endpoint, final Joint2/3 compensation, and Joint4/5/6 orientation endpoint on a shared clock. `top_down_kinematics` builds and validates that immutable path; the ROS controller only samples it, verifies measured pregrasp pose, and then reuses the unchanged DESCEND through HOME sequence.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble (`rclpy`), pytest, colcon.

## Global Constraints

- Target generation, joint ownership, and temporal blending remain separate.
- Joint1 owns index 0; the position group owns indices 1 and 2; the orientation group owns indices 3 through 5.
- No runtime whole-arm IK, Jacobian controller, live perception, or target-TF update may steer the approach after PICK latches its snapshot.
- Joint2/3 overhead contribution completes at normalized time `0.75`; compensation begins at `0.10` so it keeps pace with wrist motion.
- Validate the exact composed path at 101 samples and time-scale it against existing per-joint velocity limits and `top_down_stage_acceleration`.
- A measured start may use at most the existing `0.01 rad` limit tolerance; the path may not move farther outside and must end inside hard limits.
- `PREGRASP_VERIFY`, DESCEND, GRASP, LIFT, the measured 6 cm HOME handoff, frozen PICK target, RESET, and stopped mobile base remain unchanged.
- The dedicated launch must not fall back to simultaneous APPROACH or strict sequential execution.
- Do not start or stop a ROS launch during automated verification.

---

### Task 1: Pure blended trajectory composer

**Files:**
- Create: `ros2_ws/src/ee_switch_debug/ee_switch_debug/group_blended_trajectory.py`
- Create: `ros2_ws/src/ee_switch_debug/test/test_group_blended_trajectory.py`

**Interfaces:**
- Produces: immutable `GroupBlendedTrajectory(start, yaw, arm, pregrasp, duration, arm_completion=0.75, compensation_start=0.10)`.
- Produces: `compose_group_blended_path(start, yaw, arm, pregrasp, progress, arm_completion=0.75, compensation_start=0.10) -> tuple[np.ndarray, np.ndarray, np.ndarray]`; outputs are position, `dq/ds`, and `d2q/ds2`.
- Produces: `plan_group_blended_trajectory(start, yaw, arm, pregrasp, velocity_limits, acceleration_limit, minimum_duration, derivative_samples=1001) -> GroupBlendedTrajectory`.
- Produces: `sample_group_blended_trajectory(trajectory, elapsed) -> tuple[np.ndarray, bool]`.

- [ ] **Step 1: Write failing composition tests**

```python
import numpy as np

from ee_switch_debug.group_blended_trajectory import compose_group_blended_path

START = np.array([0.0, -1.3, 1.9, -0.5, 1.4, 0.0])
YAW = np.array([0.3, -1.3, 1.9, -0.5, 1.4, 0.0])
ARM = np.array([0.3, -0.8, 1.5, -0.5, 1.4, 0.0])
PREGRASP = np.array([0.3, -0.65, 1.35, -1.0, 1.8, 0.4])

def test_composed_path_has_exact_endpoints():
    start, _, _ = compose_group_blended_path(START, YAW, ARM, PREGRASP, 0.0)
    finish, _, _ = compose_group_blended_path(START, YAW, ARM, PREGRASP, 1.0)
    assert np.allclose(start, START)
    assert np.allclose(finish, PREGRASP)

def test_endpoint_perturbations_change_only_owned_joints():
    baseline, _, _ = compose_group_blended_path(START, YAW, ARM, PREGRASP, 0.6)
    changed_yaw = YAW.copy()
    changed_yaw[0] += 0.1
    changed, _, _ = compose_group_blended_path(START, changed_yaw, ARM, PREGRASP, 0.6)
    assert np.flatnonzero(np.abs(changed - baseline) > 1e-9).tolist() == [0]
    changed_arm = ARM.copy()
    changed_arm[1:3] += [0.1, -0.1]
    changed, _, _ = compose_group_blended_path(START, YAW, changed_arm, PREGRASP, 0.6)
    assert np.flatnonzero(np.abs(changed - baseline) > 1e-9).tolist() == [1, 2]

def test_joint23_compensation_overlaps_overhead_motion():
    progress = 0.60
    blended, _, _ = compose_group_blended_path(START, YAW, ARM, PREGRASP, progress)
    u = progress / 0.75
    arm_scale = u * u * (3.0 - 2.0 * u)
    arm_only = START[1:3] + arm_scale * (ARM[1:3] - START[1:3])
    assert not np.allclose(blended[1:3], arm_only)
```

- [ ] **Step 2: Run tests and verify RED**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_group_blended_trajectory.py
```

Expected: collection fails because the new module does not exist.

- [ ] **Step 3: Implement compact-window composition**

Use this progress primitive for each window:

```python
def _compact_window(progress: float, start: float, end: float):
    if not 0.0 <= start < end <= 1.0:
        raise ValueError("compact window must satisfy 0 <= start < end <= 1")
    if progress <= start:
        return 0.0, 0.0, 0.0
    if progress >= end:
        return 1.0, 0.0, 0.0
    width = end - start
    u = (progress - start) / width
    return (
        u * u * (3.0 - 2.0 * u),
        (6.0 * u - 6.0 * u * u) / width,
        (6.0 - 12.0 * u) / (width * width),
    )
```

Validate six finite values per endpoint and enforce:

```python
np.allclose(yaw[1:], start[1:])
np.allclose(arm[[0, 3, 4, 5]], yaw[[0, 3, 4, 5]])
np.isclose(pregrasp[0], yaw[0])
```

Compose Joint1 and Joint4/5/6 with `_compact_window(s, 0, 1)`. Compose
Joint2/3 as:

```python
q23 = start23 + A * (arm23 - start23) + C * (pregrasp23 - arm23)
```

where `A` uses `[0, 0.75]` and `C` uses `[0.10, 1]`. Apply the same linear
combination to the first and second normalized derivatives.

- [ ] **Step 4: Write failing time-scaling tests**

```python
from ee_switch_debug.group_blended_trajectory import (
    plan_group_blended_trajectory,
    sample_group_blended_trajectory,
)

def test_motion_has_no_internal_full_robot_stop():
    for progress in np.linspace(0.01, 0.99, 99):
        _, derivative, _ = compose_group_blended_path(START, YAW, ARM, PREGRASP, progress)
        assert np.linalg.norm(derivative) > 1e-6

def test_duration_respects_velocity_and_acceleration_limits():
    limits = np.array([0.70, 1.05, 1.20, 0.95, 0.95, 0.95])
    trajectory = plan_group_blended_trajectory(
        START, YAW, ARM, PREGRASP, limits, 1.6, 0.6
    )
    for progress in np.linspace(0.0, 1.0, 1001):
        _, first, second = compose_group_blended_path(START, YAW, ARM, PREGRASP, progress)
        assert np.all(np.abs(first) / trajectory.duration <= limits + 1e-9)
        assert np.all(np.abs(second) / trajectory.duration**2 <= 1.6 + 1e-9)
    finish, complete = sample_group_blended_trajectory(trajectory, trajectory.duration)
    assert complete
    assert np.allclose(finish, PREGRASP)
```

- [ ] **Step 5: Implement duration planning and trajectory sampling**

Sample 1001 normalized points. Choose duration as the maximum of
`minimum_duration`, `max(abs(dq_ds) / velocity_limits)`, and
`sqrt(max(abs(d2q_ds2) / acceleration_limit))`. Reject non-positive velocity
or acceleration limits. Sampling clamps `elapsed / duration` into `[0, 1]`.

- [ ] **Step 6: Run tests and commit**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_group_blended_trajectory.py
git add src/ee_switch_debug/ee_switch_debug/group_blended_trajectory.py src/ee_switch_debug/test/test_group_blended_trajectory.py
git commit -m "Add group-preserving blended trajectory"
```

Expected: all new trajectory tests pass.

### Task 2: Kinematic endpoints and path safety

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/top_down_kinematics.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_top_down_kinematics.py`

**Interfaces:**
- Consumes: Task 1 trajectory type, planner, and sampler.
- Produces: `TopDownPlan.blended_approach: GroupBlendedTrajectory | None`.
- Produces: `build_group_blended_approach(fixed_plan, current_joints, lower_limits, upper_limits, velocity_limits, acceleration_limit, minimum_duration, minimum_clearance, sample_count=101) -> TopDownPlan`.

- [ ] **Step 1: Write failing live-geometry tests**

```python
def _live_fixed_plan():
    return plan_fixed_orientation_top_down_sequence(
        LIVE_GRASP_POSITION,
        grasp_marker_rotation_to_ee_rotation(LIVE_MARKER_ROTATION),
        current_joints=LIVE_HOME,
        lower_limits=LOWER,
        upper_limits=UPPER,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.50,
        orientation_search_steps=20,
        maximum_xy_deviation=0.015,
        maximum_orientation_deviation=0.035,
    )

def test_group_blended_approach_accepts_live_start_and_ends_at_pregrasp():
    fixed = _live_fixed_plan()
    assert fixed.success, fixed.message
    plan = build_group_blended_approach(
        fixed, LIVE_HOME, LOWER, UPPER,
        [0.70, 1.05, 1.20, 0.95, 0.95, 0.95], 1.6, 0.6, 0.08,
    )
    assert plan.success, plan.message
    start, _ = sample_group_blended_trajectory(plan.blended_approach, 0.0)
    finish, complete = sample_group_blended_trajectory(
        plan.blended_approach, plan.blended_approach.duration
    )
    assert np.allclose(start, LIVE_HOME)
    assert complete
    assert np.allclose(finish, plan.pregrasp_joints)

def test_group_blended_approach_rejects_impossible_clearance():
    plan = build_group_blended_approach(
        _live_fixed_plan(), LIVE_HOME, LOWER, UPPER,
        [0.70, 1.05, 1.20, 0.95, 0.95, 0.95], 1.6, 0.6, 10.0,
    )
    assert not plan.success
    assert "clearance" in plan.message
```

- [ ] **Step 2: Run focused tests and verify RED**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_top_down_kinematics.py -k group_blended
```

Expected: imports fail because the builder and plan field do not exist.

- [ ] **Step 3: Extract restricted endpoints and build the blended plan**

Extract current yaw-copy and Joint2/3 solving into:

```python
def _solve_group_approach_endpoints(fixed_plan, current, lower, upper):
    yaw = current.copy()
    yaw[0] = fixed_plan.pregrasp_joints[0]
    target, _ = forward_kinematics(fixed_plan.pregrasp_joints)
    result = solve_joint23_rho_z(target, yaw, lower, upper)
    return yaw, result.joints.copy(), "" if result.success else result.message
```

Make the existing sequential builder call this helper without changing its
behavior. Add `blended_approach` to `TopDownPlan`. The new builder creates the
trajectory, rejects starts farther than `0.01 rad` outside hard limits, and
checks 101 exact trajectory samples for finite values, no movement farther
outside the measured-start envelope, and FK Z above grasp Z plus clearance.
The final sample must be inside hard limits and equal pregrasp.

- [ ] **Step 4: Add ownership and clearance regression test**

```python
def test_group_blended_live_path_preserves_ownership_and_clearance():
    plan = build_group_blended_approach(
        _live_fixed_plan(), LIVE_HOME, LOWER, UPPER,
        [0.70, 1.05, 1.20, 0.95, 0.95, 0.95], 1.6, 0.6, 0.08,
    )
    assert plan.success, plan.message
    assert np.allclose(plan.yaw_joints[1:], LIVE_HOME[1:])
    assert np.allclose(plan.arm_joints[[0, 3, 4, 5]], plan.yaw_joints[[0, 3, 4, 5]])
    grasp, _ = forward_kinematics(plan.grasp_joints)
    for elapsed in np.linspace(0.0, plan.blended_approach.duration, 101):
        joints, _ = sample_group_blended_trajectory(plan.blended_approach, elapsed)
        position, _ = forward_kinematics(joints)
        assert position[2] >= grasp[2] + 0.08 - 1e-9
```

- [ ] **Step 5: Run planner tests and commit**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_group_blended_trajectory.py src/ee_switch_debug/test/test_top_down_kinematics.py
git add src/ee_switch_debug/ee_switch_debug/top_down_kinematics.py src/ee_switch_debug/test/test_top_down_kinematics.py
git commit -m "Validate blended grasp approach path"
```

Expected: all selected tests pass, including recorded live geometry.

### Task 3: Controller execution and exclusive mode selection

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `build_group_blended_approach` and `sample_group_blended_trajectory`.
- Produces: `configured_group_approach_mode(sequential: bool, blended: bool) -> str`.
- Produces: `command_group_blended_approach(dt: float) -> bool`.
- Adds: `BLENDED_APPROACH -> PREGRASP_VERIFY`.

- [ ] **Step 1: Write failing mode and routing tests**

```python
def test_group_approach_modes_are_mutually_exclusive():
    assert controller.configured_group_approach_mode(False, False) == "disabled"
    assert controller.configured_group_approach_mode(True, False) == "sequential"
    assert controller.configured_group_approach_mode(False, True) == "blended"
    with pytest.raises(ValueError, match="mutually exclusive"):
        controller.configured_group_approach_mode(True, True)
```

Refactor the current fixed-plan test fixture, enable blended and disable
sequential, monkeypatch `build_group_blended_approach`, make the sequential
builder raise if called, then assert the blended builder runs once and initial
stage equals `BLENDED_APPROACH`.

- [ ] **Step 2: Run focused tests and verify RED**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_wrist_orientation.py -k 'group_approach_modes or blended_plan'
```

Expected: failures for missing mode helper, parameter, and route.

- [ ] **Step 3: Implement configuration and plan routing**

```python
def configured_group_approach_mode(sequential: bool, blended: bool) -> str:
    if sequential and blended:
        raise ValueError("sequential and blended group approaches are mutually exclusive")
    if blended:
        return "blended"
    if sequential:
        return "sequential"
    return "disabled"
```

Declare/load `top_down_use_group_blended_approach`, default false. Resolve mode
before building the approach. Invalid configuration or failed blended planning
sets `PLAN_FAILED` and never invokes another builder. Successful blended mode
starts at `BLENDED_APPROACH`.

- [ ] **Step 4: Write failing command and transition tests**

```python
def test_blended_approach_transitions_only_to_pregrasp_verify():
    node = make_group_sequence_node("BLENDED_APPROACH")
    node.command_group_blended_approach = lambda _dt: True
    transitions = []
    node.advance_precomputed_top_down_stage = lambda next_stage, preserve_velocity=False: transitions.append(next_stage)
    node.run_semantic_fixed_orientation_pick(0.1)
    assert transitions == ["PREGRASP_VERIFY"]
    assert node.gripper_position == node.gripper_open_position
```

Add a second test monkeypatching `sample_group_blended_trajectory` to return
`np.arange(6)` and incomplete, then assert `command_group_blended_approach`
publishes exactly those six positions without invoking group-specific live
controllers.

- [ ] **Step 5: Implement immutable sampling and final settling**

On the first call, record the approach start time. While incomplete, sample the
immutable trajectory, update `previous_velocity` from consecutive commands
within alignment limits, and publish the six sampled positions. At planned
completion retain exact pregrasp and use `fixed_target_joint_velocities` with
all joints active and strict tolerance until measured completion. Then advance
only to `PREGRASP_VERIFY`, clearing retained velocity before DESCEND.

- [ ] **Step 6: Run controller tests and commit**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_wrist_orientation.py
git add src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py src/ee_switch_debug/test/test_wrist_orientation.py
git commit -m "Execute blended joint-group grasp approach"
```

Expected: all controller tests pass and sequential-mode regressions stay green.

### Task 4: Launch activation, documentation, and verification

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/config/arm_position_target_in.yaml`
- Modify: `ros2_ws/src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Adds shared `top_down_use_group_blended_approach: false`.
- Dedicated launch sets blended true and sequential false.
- Documents `PICK:BLENDED_APPROACH -> PICK:PREGRASP_VERIFY -> PICK:DESCEND`.

- [ ] **Step 1: Write failing launch assertions**

```python
assert '"top_down_use_group_sequential_approach": False' in source
assert '"top_down_use_group_blended_approach": True' in source
```

Retain every existing launch assertion for fixed orientation, limits, frozen
perception, DESCEND, LIFT, and HOME.

- [ ] **Step 2: Run launch test and verify RED**

```bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py
```

Expected: failure because sequential is still true and blended is absent.

- [ ] **Step 3: Activate blended mode and update README**

Add the false default to YAML. Set sequential false and blended true in the
dedicated launch. Document:

```text
BLENDED_APPROACH(group-generated Joint1 + Joint2/3 + Joint4/5/6)
    -> PREGRASP_VERIFY -> DESCEND -> GRASP -> LIFT -> CONTINUOUS HOME -> HOLD
```

State that this is simultaneous playback of separately owned group targets,
not runtime six-axis IK, and that video acceptance remains required.

- [ ] **Step 4: Run full tests, isolated build, and diff check**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q src/ee_switch_debug/test
colcon build --symlink-install --packages-select ee_switch_debug --build-base build_codex --install-base install_codex
git diff --check
```

Expected: all tests pass, `ee_switch_debug` finishes, and diff check is empty.
Existing-install warnings for other workspace packages are acceptable.

- [ ] **Step 5: Commit, push, and verify Draft PR #10**

```bash
git add src/ee_switch_debug/config/arm_position_target_in.yaml src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py src/ee_switch_debug/README_PICK_PLACE.md
git commit -m "Enable fully blended top-down approach"
git push origin agent/hybrid-wholebody-updates
gh pr view 10 --json number,title,isDraft,state,url,headRefName
```

Expected: push succeeds and Draft PR #10 uses the current branch.

- [ ] **Step 6: User-run simulator acceptance**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install_codex/setup.bash
ros2 launch ee_switch_debug yoloe_top_down_grasp_test.launch.py
```

Accept only after video confirms continuous blended approach, no twist or
collision, accurate overhead pose, vertical descent, successful grasp, and
continuous LIFT-to-HOME. Automated tests alone cannot prove visual naturalness.
