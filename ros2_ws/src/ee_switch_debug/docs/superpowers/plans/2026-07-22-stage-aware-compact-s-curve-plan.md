# Stage-Aware Compact S-Curve Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce visible APPROACH, LIFT, and HOME endpoint pauses with a bound-aware compact S-curve while keeping DESCEND on the existing minimum-jerk quintic profile.

**Architecture:** Add a reusable cubic position S-curve and its exact duration bounds to the pure trajectory module. Add a pure stage-to-profile policy in the arm controller, then route semantic duration calculation and sampling through that policy without changing the state machine, endpoints, or completion tolerances.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble (`rclpy`), pytest, colcon.

## Global Constraints

- The state sequence remains `APPROACH -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD`.
- `DESCEND` must retain the quintic profile and strict `0.025 rad` measured-joint tolerance.
- APPROACH, LIFT, and HOME must retain exact endpoints, zero endpoint velocity, configured velocity/acceleration bounds, and non-overlapping execution.
- Dedicated-launch velocity limits, acceleration `1.6 rad/s^2`, and minimum segment duration `0.6 s` remain unchanged.
- Do not start or stop the user's ROS launch during automated verification.

---

### Task 1: Bound-aware compact cubic trajectory profile

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/semantic_joint_trajectory.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_semantic_joint_trajectory.py`

**Interfaces:**
- Produces: `compact_smoothstep(fraction: float) -> float`
- Produces: `minimum_compact_duration(start, goal, velocity_limits, acceleration_limit, minimum_duration) -> float`
- Produces: `sample_compact_joint_positions(start, goal, elapsed, duration) -> tuple[np.ndarray, bool]`

- [ ] **Step 1: Write failing compact-profile tests**

Add imports for the three interfaces above and these tests:

```python
def test_compact_smoothstep_has_exact_endpoints_and_moves_early():
    assert compact_smoothstep(-1.0) == 0.0
    assert compact_smoothstep(0.0) == 0.0
    assert compact_smoothstep(0.5) == 0.5
    assert compact_smoothstep(1.0) == 1.0
    assert compact_smoothstep(2.0) == 1.0
    assert compact_smoothstep(0.2) > quintic_smoothstep(0.2)


def test_compact_smoothstep_has_zero_endpoint_velocity():
    epsilon = 1e-6
    start_slope = (compact_smoothstep(epsilon) - compact_smoothstep(0.0)) / epsilon
    end_slope = (
        compact_smoothstep(1.0) - compact_smoothstep(1.0 - epsilon)
    ) / epsilon
    assert abs(start_slope) < 4e-6
    assert abs(end_slope) < 4e-6


def test_compact_duration_respects_exact_velocity_and_acceleration_bounds():
    duration = minimum_compact_duration(
        np.zeros(3),
        np.array([1.0, -0.4, 0.2]),
        np.array([0.5, 1.0, 2.0]),
        acceleration_limit=1.2,
        minimum_duration=0.8,
    )
    assert duration >= 1.5 * 1.0 / 0.5
    assert duration >= np.sqrt(6.0 * 1.0 / 1.2)
    assert duration >= 0.8


def test_compact_joint_trajectory_samples_exact_endpoints_and_completion():
    start = np.array([0.0, -1.0, 2.0])
    goal = np.array([1.0, 0.0, -1.0])
    before, complete_before = sample_compact_joint_positions(start, goal, -1.0, 4.0)
    midpoint, complete_midpoint = sample_compact_joint_positions(start, goal, 2.0, 4.0)
    after, complete_after = sample_compact_joint_positions(start, goal, 8.0, 4.0)
    assert np.array_equal(before, start)
    assert not complete_before
    assert np.allclose(midpoint, 0.5 * (start + goal))
    assert not complete_midpoint
    assert np.array_equal(after, goal)
    assert complete_after
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug pytest -q src/ee_switch_debug/test/test_semantic_joint_trajectory.py
```

Expected: collection fails because the compact-profile interfaces do not exist.

- [ ] **Step 3: Implement the minimal compact profile**

Add the constants and functions below. Factor the shared shape validation only if it keeps existing exception behavior unchanged.

```python
COMPACT_MAX_NORMALIZED_VELOCITY = 1.5
COMPACT_MAX_NORMALIZED_ACCELERATION = 6.0


def compact_smoothstep(fraction: float) -> float:
    value = float(np.clip(fraction, 0.0, 1.0))
    return value**2 * (3.0 - 2.0 * value)


def minimum_compact_duration(
    start: Sequence[float],
    goal: Sequence[float],
    velocity_limits: Sequence[float],
    acceleration_limit: float,
    minimum_duration: float,
) -> float:
    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    limits = np.asarray(velocity_limits, dtype=float)
    if start_array.shape != goal_array.shape or start_array.shape != limits.shape:
        raise ValueError("start, goal, and velocity limits must have matching shapes")
    if np.any(limits <= 0.0):
        raise ValueError("velocity limits must be positive")
    acceleration = float(acceleration_limit)
    if acceleration <= 0.0:
        raise ValueError("acceleration limit must be positive")
    delta = np.abs(goal_array - start_array)
    velocity_duration = float(
        np.max(COMPACT_MAX_NORMALIZED_VELOCITY * delta / limits)
    )
    acceleration_duration = float(
        np.max(np.sqrt(COMPACT_MAX_NORMALIZED_ACCELERATION * delta / acceleration))
    )
    return max(float(minimum_duration), velocity_duration, acceleration_duration)


def sample_compact_joint_positions(
    start: Sequence[float],
    goal: Sequence[float],
    elapsed: float,
    duration: float,
) -> tuple[np.ndarray, bool]:
    start_array = np.asarray(start, dtype=float)
    goal_array = np.asarray(goal, dtype=float)
    if start_array.shape != goal_array.shape:
        raise ValueError("start and goal must have matching shapes")
    safe_duration = max(float(duration), 1e-9)
    complete = float(elapsed) >= safe_duration
    fraction = float(np.clip(float(elapsed) / safe_duration, 0.0, 1.0))
    scale = compact_smoothstep(fraction)
    return start_array + scale * (goal_array - start_array), complete
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the command from Step 2. Expected: all trajectory tests pass.

- [ ] **Step 5: Commit the pure trajectory change**

```bash
git add ros2_ws/src/ee_switch_debug/ee_switch_debug/semantic_joint_trajectory.py \
  ros2_ws/src/ee_switch_debug/test/test_semantic_joint_trajectory.py
git commit -m "feat: add compact semantic trajectory profile"
```

### Task 2: Route transit stages through the compact profile

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Consumes: Task 1 compact duration and sampling functions.
- Produces: `semantic_stage_trajectory_profile(stage: str) -> str`
- Produces: controller routing where APPROACH/LIFT/HOME use `compact` and DESCEND/unknown use `quintic`.

- [ ] **Step 1: Write the failing stage-policy test**

Import `semantic_stage_trajectory_profile` from the controller and add:

```python
def test_semantic_stage_trajectory_profile_keeps_descend_conservative():
    for stage in ("APPROACH", "LIFT", "HOME"):
        assert semantic_stage_trajectory_profile(stage) == "compact"
    assert semantic_stage_trajectory_profile("DESCEND") == "quintic"
    assert semantic_stage_trajectory_profile("UNKNOWN") == "quintic"
```

- [ ] **Step 2: Run the policy test and verify RED**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -k semantic_stage_trajectory_profile
```

Expected: collection fails because `semantic_stage_trajectory_profile` does not exist.

- [ ] **Step 3: Add the conservative stage policy**

```python
def semantic_stage_trajectory_profile(stage: str) -> str:
    normalized = str(stage).strip().upper()
    if normalized in ("APPROACH", "LIFT", "HOME"):
        return "compact"
    return "quintic"
```

Import `minimum_compact_duration` and `sample_compact_joint_positions` beside the existing quintic imports.

- [ ] **Step 4: Run the policy test and verify GREEN**

Run the command from Step 2. Expected: one selected test passes.

- [ ] **Step 5: Write failing controller-routing tests**

Add the following fixture and tests. They isolate the first tick of a new
semantic segment, give the two profiles distinct durations, and count sampler
calls:

```python
def make_semantic_segment_node():
    class FakeTime:
        def __sub__(self, _other):
            return SimpleNamespace(nanoseconds=0)

    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.joint_names = [f"joint{index}" for index in range(1, 7)]
    node.current_joints = dict.fromkeys(node.joint_names, 0.0)
    node.position_command = [0.0] * 6
    node.top_down_segment_stage = ""
    node.top_down_segment_start = None
    node.top_down_segment_goal = None
    node.top_down_segment_start_time = None
    node.top_down_segment_duration = 0.0
    node.top_down_segment_planned_complete = False
    node.top_down_segment_min_duration = 0.6
    node.top_down_stage_acceleration = 1.6
    node.top_down_alignment_velocity_limits = [1.0] * 6
    node.top_down_path_velocity_limits = [0.5] * 6
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeTime())
    node.get_logger = lambda: SimpleNamespace(warn=lambda *_args: None)
    return node


def test_semantic_segment_uses_compact_profile_for_approach(monkeypatch):
    compact_samples = []
    quintic_samples = []
    monkeypatch.setattr(controller, "minimum_compact_duration", lambda *_args, **_kwargs: 2.0)
    monkeypatch.setattr(controller, "minimum_quintic_duration", lambda *_args, **_kwargs: 3.0)
    monkeypatch.setattr(
        controller,
        "sample_compact_joint_positions",
        lambda start, *_args: (compact_samples.append("APPROACH") or np.asarray(start), False),
    )
    monkeypatch.setattr(
        controller,
        "sample_quintic_joint_positions",
        lambda start, *_args: (quintic_samples.append("APPROACH") or np.asarray(start), False),
    )
    node = make_semantic_segment_node()

    assert not node.command_semantic_top_down_segment([0.2] * 6, "APPROACH", 0.1)
    assert node.top_down_segment_duration == 2.0
    assert compact_samples == ["APPROACH"]
    assert quintic_samples == []


def test_semantic_segment_keeps_quintic_profile_for_descend(monkeypatch):
    compact_samples = []
    quintic_samples = []
    monkeypatch.setattr(controller, "minimum_compact_duration", lambda *_args, **_kwargs: 2.0)
    monkeypatch.setattr(controller, "minimum_quintic_duration", lambda *_args, **_kwargs: 3.0)
    monkeypatch.setattr(
        controller,
        "sample_compact_joint_positions",
        lambda start, *_args: (compact_samples.append("DESCEND") or np.asarray(start), False),
    )
    monkeypatch.setattr(
        controller,
        "sample_quintic_joint_positions",
        lambda start, *_args: (quintic_samples.append("DESCEND") or np.asarray(start), False),
    )
    node = make_semantic_segment_node()

    assert not node.command_semantic_top_down_segment([0.2] * 6, "DESCEND", 0.1)
    assert node.top_down_segment_duration == 3.0
    assert compact_samples == []
    assert quintic_samples == ["DESCEND"]
```

- [ ] **Step 6: Run routing tests and verify RED**

Run:

```bash
PYTHONPATH=src/ee_switch_debug pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -k 'semantic_segment_uses_compact or semantic_segment_keeps_quintic'
```

Expected: the APPROACH test fails because the controller still always calls the quintic functions.

- [ ] **Step 7: Route duration and sampling by stage**

In `command_semantic_top_down_segment`, compute:

```python
profile = semantic_stage_trajectory_profile(normalized_stage)
duration_function = (
    minimum_compact_duration if profile == "compact" else minimum_quintic_duration
)
sample_function = (
    sample_compact_joint_positions
    if profile == "compact"
    else sample_quintic_joint_positions
)
```

Use `duration_function(...)` when initializing the segment and
`sample_function(...)` on every tick. Include `profile={profile}` in the
trajectory-start log. Do not alter limits, state transitions, target endpoints,
settling feedback, or tolerances.

- [ ] **Step 8: Run routing tests and verify GREEN**

Run the command from Step 6. Expected: both routing tests pass.

- [ ] **Step 9: Update operator documentation**

In `README_PICK_PLACE.md`, replace the claim that all four semantic stages use
bounded quintic trajectories. State that APPROACH/LIFT/HOME use the compact
cubic S-curve, DESCEND retains minimum-jerk quintic scaling, every stage reaches
zero endpoint velocity, and the launch speed/acceleration values are unchanged.

- [ ] **Step 10: Run package tests and build**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug pytest -q src/ee_switch_debug/test
colcon build --symlink-install --packages-select ee_switch_debug \
  --build-base build_codex --install-base install_codex
```

Expected: zero pytest failures and `ee_switch_debug` finishes successfully.

- [ ] **Step 11: Inspect the scoped diff and commit**

```bash
git diff --check
git diff -- ros2_ws/src/ee_switch_debug/ee_switch_debug/semantic_joint_trajectory.py \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py \
  ros2_ws/src/ee_switch_debug/test/test_semantic_joint_trajectory.py \
  ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py \
  ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md
git add ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py \
  ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py \
  ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md
git commit -m "feat: shorten semantic transit easing"
```

### Task 3: Simulator evidence handoff

**Files:**
- No source changes.

**Interfaces:**
- Consumes: completed package build from Task 2.
- Produces: exact user-run commands and observable acceptance criteria.

- [ ] **Step 1: Provide rebuild and launch commands**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ee_switch_debug
source install/setup.bash
ros2 launch ee_switch_debug yoloe_top_down_grasp_test.launch.py
```

- [ ] **Step 2: State recording acceptance criteria**

The user recording must show earlier visible motion after APPROACH, LIFT, and
HOME begin; a full stop before DESCEND-to-GRASP; no link1 snap; no object contact
during APPROACH; and the can retained through LIFT and HOME. Compare timestamps
against the 13:24:40 recording before claiming the three-second pause is reduced.
