# Long-Range Whole-Body Top-Down Pick Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an opt-in long-range mode that preserves the existing Nav2, hybrid, base/arm switching, and place behavior while using rolling perception and a coordinated six-joint top-down trajectory for pick.

**Architecture:** Keep the legacy controller path as the default and add a `rolling_top_down` pick execution mode inside the existing arm controller. Perception and asynchronous IK planning run during the existing whole-body approach; the newest valid plan guides pre-grasp motion and is latched only when the existing switching logic has reduced base motion enough to allow descent.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, `geometry_msgs`, `sensor_msgs`, TF2, NumPy, `concurrent.futures`, pytest, ROS 2 launch.

## Global Constraints

- The existing `florence_long_range_pick_place_control.launch.py` default behavior must remain unchanged.
- Do not change Nav2 configuration, hybrid thresholds, `navigation_cmd_mux`, base/arm switching equations, `pick_place_task_manager`, direct-place policy, safe-retreat policy, or legacy place control.
- Base deceleration and arm pre-grasp motion must overlap; do not add a stop-then-start handoff.
- Do not allow `DESCEND` until base contribution and final base command are below configured contact thresholds.
- Preserve `PICK:HOLD` as the completed-pick interface to the existing task manager.
- The new long-range mode must set `return_home_after_pick=false`.
- Planner work must not run in or block the 40 Hz control callback.
- Reject stale or drifted plans and never silently fall back to legacy descent.
- The local `ros2_ws/dds_setting/cyclonedds.xml` file is environment configuration and must not be staged or committed.

---

## File Structure

- Create `ros2_ws/src/ee_switch_debug/ee_switch_debug/rolling_top_down_pick.py`
  - Owns immutable candidate/request/result records, freshness and drift checks,
    terminal contact gating, coordinated joint blending, and the latest-only
    asynchronous planner.
- Create `ros2_ws/src/ee_switch_debug/test/test_rolling_top_down_pick.py`
  - Tests all pure lifecycle, gating, blending, and asynchronous planning
    behavior without starting ROS.
- Modify `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
  - Adds the opt-in mode, rolling grasp subscription, non-blocking plan
    submission, coordinated pre-grasp use, terminal latch, failure state, and
    planner cleanup.
- Modify `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`
  - Covers controller integration seams and proves legacy behavior is still the
    default.
- Create `ros2_ws/src/ee_switch_debug/launch/yoloe_long_range_top_down_pick_place.launch.py`
  - Copies the proven long-range composition and changes only the rolling
    top-down pick parameters.
- Create `ros2_ws/src/ee_switch_debug/test/test_yoloe_long_range_top_down_launch.py`
  - Locks the new launch configuration and compares protected behavior with the
    existing long-range launch.
- Modify `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`
  - Documents the new launch, state flow, monitoring topics, and failure state.

---

### Task 1: Pure Rolling-Plan Records, Validation, and Contact Gate

**Files:**
- Create: `ros2_ws/src/ee_switch_debug/ee_switch_debug/rolling_top_down_pick.py`
- Create: `ros2_ws/src/ee_switch_debug/test/test_rolling_top_down_pick.py`

**Interfaces:**
- Produces: `RollingPlanRequest`, `RollingPlanResult`, `TerminalGateInput`
- Produces: `candidate_is_fresh(...) -> bool`
- Produces: `rolling_plan_is_valid(...) -> bool`
- Produces: `terminal_gate_ready(...) -> bool`
- Produces: `coordinated_joint_velocity(...) -> list[float]`

- [ ] **Step 1: Write failing tests for freshness, drift, contact gating, and coordinated blending**

```python
# ros2_ws/src/ee_switch_debug/test/test_rolling_top_down_pick.py
from types import SimpleNamespace

from pytest import approx

from ee_switch_debug.rolling_top_down_pick import (
    RollingPlanRequest,
    RollingPlanResult,
    TerminalGateInput,
    candidate_is_fresh,
    coordinated_joint_velocity,
    rolling_plan_is_valid,
    terminal_gate_ready,
)


def make_result() -> RollingPlanResult:
    request = RollingPlanRequest(
        candidate_stamp_ns=900_000_000,
        target_position=(0.40, -0.10, 0.30),
        target_rotation=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        seed_joints=(0.0, -1.2, 1.8, -0.5, 1.4, 0.0),
        lower_limits=(-0.7, -1.6, -1.1, -3.2, -3.2, -3.2),
        upper_limits=(0.7, 1.3, 2.8, 3.2, 3.2, 3.2),
    )
    return RollingPlanResult(
        request=request,
        plan=SimpleNamespace(success=True, pregrasp_joints=[0.1] * 6),
        completed_ns=950_000_000,
        duration_sec=0.205,
    )


def test_candidate_freshness_uses_source_stamp():
    assert candidate_is_fresh(900_000_000, 1_000_000_000, 0.25)
    assert not candidate_is_fresh(700_000_000, 1_000_000_000, 0.25)
    assert not candidate_is_fresh(0, 1_000_000_000, 0.25)


def test_plan_validation_rejects_target_seed_and_age_drift():
    result = make_result()
    assert rolling_plan_is_valid(
        result,
        now_ns=1_000_000_000,
        target_position=(0.405, -0.10, 0.30),
        current_joints=(0.01, -1.2, 1.8, -0.5, 1.4, 0.0),
        max_candidate_age_sec=0.25,
        max_target_drift_m=0.02,
        max_seed_drift_rad=0.15,
    )
    assert not rolling_plan_is_valid(
        result,
        now_ns=1_000_000_000,
        target_position=(0.45, -0.10, 0.30),
        current_joints=(0.01, -1.2, 1.8, -0.5, 1.4, 0.0),
        max_candidate_age_sec=0.25,
        max_target_drift_m=0.02,
        max_seed_drift_rad=0.15,
    )


def test_terminal_gate_requires_existing_arm_ownership_and_low_base_motion():
    ready = TerminalGateInput(
        control_state="ARM_TRACK",
        base_scale=0.03,
        target_safe=True,
        base_linear_command=0.01,
        base_angular_command=0.02,
        stable_cycles=3,
        required_stable_cycles=3,
        plan_valid=True,
    )
    assert terminal_gate_ready(
        ready,
        max_base_scale=0.05,
        max_linear_command=0.02,
        max_angular_command=0.05,
    )
    assert not terminal_gate_ready(
        TerminalGateInput(**{**ready.__dict__, "base_scale": 0.20}),
        max_base_scale=0.05,
        max_linear_command=0.02,
        max_angular_command=0.05,
    )


def test_joint_groups_blend_as_one_six_joint_command():
    velocity = coordinated_joint_velocity(
        home_velocity=(0.0, -0.2, 0.2, 0.0, 0.0, 0.0),
        pregrasp_velocity=(0.4, 0.2, -0.2, 0.3, -0.3, 0.1),
        arm_weight=0.25,
    )
    assert velocity == approx([0.1, -0.1, 0.1, 0.075, -0.075, 0.025])
```

- [ ] **Step 2: Run the tests and verify they fail because the module does not exist**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/ee_switch_debug/test/test_rolling_top_down_pick.py -q
```

Expected: collection fails with
`ModuleNotFoundError: No module named 'ee_switch_debug.rolling_top_down_pick'`.

- [ ] **Step 3: Implement the immutable records and pure validation functions**

```python
# ros2_ws/src/ee_switch_debug/ee_switch_debug/rolling_top_down_pick.py
from dataclasses import dataclass
from math import sqrt
from typing import Any, Sequence


@dataclass(frozen=True)
class RollingPlanRequest:
    candidate_stamp_ns: int
    target_position: tuple[float, float, float]
    target_rotation: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    seed_joints: tuple[float, float, float, float, float, float]
    lower_limits: tuple[float, float, float, float, float, float]
    upper_limits: tuple[float, float, float, float, float, float]


@dataclass(frozen=True)
class RollingPlanResult:
    request: RollingPlanRequest
    plan: Any
    completed_ns: int
    duration_sec: float


@dataclass(frozen=True)
class TerminalGateInput:
    control_state: str
    base_scale: float
    target_safe: bool
    base_linear_command: float
    base_angular_command: float
    stable_cycles: int
    required_stable_cycles: int
    plan_valid: bool


def candidate_is_fresh(
    candidate_stamp_ns: int,
    now_ns: int,
    max_age_sec: float,
) -> bool:
    if int(candidate_stamp_ns) <= 0:
        return False
    age_ns = int(now_ns) - int(candidate_stamp_ns)
    return 0 <= age_ns <= int(max(0.0, float(max_age_sec)) * 1e9)


def _distance(a: Sequence[float], b: Sequence[float]) -> float:
    return sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))


def _max_abs_difference(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def rolling_plan_is_valid(
    result: RollingPlanResult | None,
    *,
    now_ns: int,
    target_position: Sequence[float],
    current_joints: Sequence[float],
    max_candidate_age_sec: float,
    max_target_drift_m: float,
    max_seed_drift_rad: float,
) -> bool:
    if result is None or not bool(getattr(result.plan, "success", False)):
        return False
    return (
        candidate_is_fresh(
            result.request.candidate_stamp_ns,
            now_ns,
            max_candidate_age_sec,
        )
        and _distance(result.request.target_position, target_position)
        <= float(max_target_drift_m)
        and _max_abs_difference(result.request.seed_joints, current_joints)
        <= float(max_seed_drift_rad)
    )


def terminal_gate_ready(
    gate: TerminalGateInput,
    *,
    max_base_scale: float,
    max_linear_command: float,
    max_angular_command: float,
) -> bool:
    return (
        gate.control_state == "ARM_TRACK"
        and gate.target_safe
        and gate.plan_valid
        and gate.base_scale <= float(max_base_scale)
        and abs(gate.base_linear_command) <= float(max_linear_command)
        and abs(gate.base_angular_command) <= float(max_angular_command)
        and gate.stable_cycles >= gate.required_stable_cycles
    )


def coordinated_joint_velocity(
    home_velocity: Sequence[float],
    pregrasp_velocity: Sequence[float],
    arm_weight: float,
) -> list[float]:
    weight = max(0.0, min(1.0, float(arm_weight)))
    return [
        (1.0 - weight) * float(home) + weight * float(pregrasp)
        for home, pregrasp in zip(home_velocity, pregrasp_velocity)
    ]
```

- [ ] **Step 4: Run the pure tests and verify they pass**

Run the command from Step 2.

Expected: `4 passed`.

- [ ] **Step 5: Commit the pure state and gate primitives**

```bash
git add \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/rolling_top_down_pick.py \
  ros2_ws/src/ee_switch_debug/test/test_rolling_top_down_pick.py
git commit -m "feat: add rolling top-down pick gates"
```

---

### Task 2: Latest-Only Asynchronous Top-Down Planner

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/rolling_top_down_pick.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_rolling_top_down_pick.py`

**Interfaces:**
- Consumes: `RollingPlanRequest`, `RollingPlanResult`
- Produces: `build_fixed_top_down_plan(request) -> TopDownPlan`
- Produces: `LatestOnlyTopDownPlanner.submit(request) -> None`
- Produces: `LatestOnlyTopDownPlanner.poll() -> RollingPlanResult | None`
- Produces: `LatestOnlyTopDownPlanner.close() -> None`

- [ ] **Step 1: Add failing tests proving planner work is non-blocking and pending work is latest-only**

```python
from threading import Event

from ee_switch_debug.rolling_top_down_pick import LatestOnlyTopDownPlanner


def test_latest_only_planner_replaces_pending_request():
    started = Event()
    release = Event()
    planned_stamps = []

    def fake_plan(request):
        planned_stamps.append(request.candidate_stamp_ns)
        if len(planned_stamps) == 1:
            started.set()
            assert release.wait(timeout=1.0)
        return SimpleNamespace(success=True, pregrasp_joints=[0.1] * 6)

    planner = LatestOnlyTopDownPlanner(fake_plan)
    first = make_result().request
    second = RollingPlanRequest(**{**first.__dict__, "candidate_stamp_ns": 910_000_000})
    newest = RollingPlanRequest(**{**first.__dict__, "candidate_stamp_ns": 920_000_000})
    try:
        planner.submit(first)
        assert started.wait(timeout=1.0)
        planner.submit(second)
        planner.submit(newest)
        assert planner.poll() is None
        release.set()

        results = []
        for _ in range(100):
            result = planner.poll()
            if result is not None:
                results.append(result)
            if len(results) == 2:
                break

        assert planned_stamps == [900_000_000, 920_000_000]
        assert [result.request.candidate_stamp_ns for result in results] == [
            900_000_000,
            920_000_000,
        ]
    finally:
        planner.close()
```

- [ ] **Step 2: Run the focused test and verify it fails**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_rolling_top_down_pick.py::test_latest_only_planner_replaces_pending_request \
  -q
```

Expected: import fails because `LatestOnlyTopDownPlanner` is undefined.

- [ ] **Step 3: Implement the actual planner wrapper and latest-only worker**

Add these imports and definitions to
`ee_switch_debug/rolling_top_down_pick.py`:

```python
from concurrent.futures import Future, ThreadPoolExecutor
from threading import Lock
from time import perf_counter, time_ns
from typing import Callable

from ee_switch_debug.top_down_kinematics import (
    grasp_marker_rotation_to_ee_rotation,
    plan_fixed_orientation_top_down_sequence,
)


def build_fixed_top_down_plan(request: RollingPlanRequest):
    rotation = grasp_marker_rotation_to_ee_rotation(request.target_rotation)
    return plan_fixed_orientation_top_down_sequence(
        request.target_position,
        rotation,
        current_joints=request.seed_joints,
        lower_limits=request.lower_limits,
        upper_limits=request.upper_limits,
        clearance=0.10,
        radial_inward_offset=0.025,
        minimum_orientation_fraction=0.50,
        orientation_search_steps=20,
        maximum_xy_deviation=0.015,
        maximum_orientation_deviation=0.035,
    )


class LatestOnlyTopDownPlanner:
    def __init__(
        self,
        plan_function: Callable[[RollingPlanRequest], Any] = build_fixed_top_down_plan,
    ) -> None:
        self._plan_function = plan_function
        self._executor = ThreadPoolExecutor(max_workers=1)
        self._lock = Lock()
        self._active_request: RollingPlanRequest | None = None
        self._active_future: Future | None = None
        self._pending_request: RollingPlanRequest | None = None

    def _start(self, request: RollingPlanRequest) -> None:
        def run() -> tuple[Any, float]:
            started = perf_counter()
            return self._plan_function(request), perf_counter() - started

        self._active_request = request
        self._active_future = self._executor.submit(run)

    def submit(self, request: RollingPlanRequest) -> None:
        with self._lock:
            if self._active_future is None:
                self._start(request)
            else:
                self._pending_request = request

    def poll(self) -> RollingPlanResult | None:
        with self._lock:
            if self._active_future is None or not self._active_future.done():
                return None
            request = self._active_request
            plan, duration_sec = self._active_future.result()
            result = RollingPlanResult(
                request=request,
                plan=plan,
                completed_ns=time_ns(),
                duration_sec=duration_sec,
            )
            self._active_request = None
            self._active_future = None
            pending = self._pending_request
            self._pending_request = None
            if pending is not None:
                self._start(pending)
            return result

    def close(self) -> None:
        with self._lock:
            self._pending_request = None
        self._executor.shutdown(wait=True, cancel_futures=True)
```

- [ ] **Step 4: Make the asynchronous test deterministic**

Add `from time import sleep` to the test and insert `sleep(0.005)` inside the
100-iteration polling loop after an empty poll. This gives the worker time to
finish without a busy-loop race.

- [ ] **Step 5: Run the rolling planner test file**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest src/ee_switch_debug/test/test_rolling_top_down_pick.py -q
```

Expected: `5 passed`.

- [ ] **Step 6: Commit the asynchronous planner**

```bash
git add \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/rolling_top_down_pick.py \
  ros2_ws/src/ee_switch_debug/test/test_rolling_top_down_pick.py
git commit -m "feat: plan rolling top-down picks asynchronously"
```

---

### Task 3: Feature-Gated Candidate Intake and Planner Lifecycle

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `/sm_grasping/grasp_best` as `geometry_msgs/PoseStamped`
- Consumes: `LatestOnlyTopDownPlanner`
- Produces: `pick_execution_mode=legacy|rolling_top_down`
- Produces: `on_rolling_grasp_candidate(msg: PoseStamped) -> None`
- Produces: `submit_rolling_plan_if_needed(now_ns: int) -> None`

- [ ] **Step 1: Add failing tests for default isolation and source timestamp preservation**

```python
from geometry_msgs.msg import PoseStamped


def test_legacy_pick_mode_is_the_controller_default():
    source = Path(controller.__file__).read_text()
    assert 'self.declare_parameter("pick_execution_mode", "legacy")' in source


def test_rolling_candidate_keeps_the_grasp_source_stamp():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.pick_execution_mode = "rolling_top_down"
    node.task_mode = "PICK"
    node.rolling_candidate = None

    msg = PoseStamped()
    msg.header.frame_id = "link0"
    msg.header.stamp.sec = 12
    msg.header.stamp.nanosec = 34
    msg.pose.position.x = 0.4
    msg.pose.position.y = -0.1
    msg.pose.position.z = 0.3
    msg.pose.orientation.w = 1.0

    node.on_rolling_grasp_candidate(msg)

    assert node.rolling_candidate.header.stamp.sec == 12
    assert node.rolling_candidate.header.stamp.nanosec == 34
```

- [ ] **Step 2: Run the two tests and verify they fail**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_wrist_orientation.py::test_legacy_pick_mode_is_the_controller_default \
  src/ee_switch_debug/test/test_wrist_orientation.py::test_rolling_candidate_keeps_the_grasp_source_stamp \
  -q
```

Expected: both fail because the parameter and callback do not exist.

- [ ] **Step 3: Add controller parameters, state, and the grasp subscription**

Add imports:

```python
from copy import deepcopy

from geometry_msgs.msg import PoseStamped, Twist

from ee_switch_debug.rolling_top_down_pick import (
    LatestOnlyTopDownPlanner,
    RollingPlanRequest,
    TerminalGateInput,
    candidate_is_fresh,
    coordinated_joint_velocity,
    rolling_plan_is_valid,
    terminal_gate_ready,
)
```

Declare these parameters with defaults:

```python
self.declare_parameter("pick_execution_mode", "legacy")
self.declare_parameter("rolling_grasp_topic", "/sm_grasping/grasp_best")
self.declare_parameter("rolling_grasp_frame", "link0")
self.declare_parameter("rolling_candidate_max_age_sec", 0.50)
self.declare_parameter("rolling_target_drift_m", 0.025)
self.declare_parameter("rolling_seed_drift_rad", 0.20)
self.declare_parameter("rolling_base_scale_threshold", 0.05)
self.declare_parameter("rolling_base_linear_threshold", 0.02)
self.declare_parameter("rolling_base_angular_threshold", 0.05)
self.declare_parameter("rolling_stable_cycles", 3)
self.declare_parameter("rolling_terminal_timeout_sec", 3.0)
```

Initialize the feature-gated state:

```python
self.pick_execution_mode = str(
    self.get_parameter("pick_execution_mode").value
).strip().lower()
if self.pick_execution_mode not in ("legacy", "rolling_top_down"):
    raise ValueError(
        "pick_execution_mode must be 'legacy' or 'rolling_top_down'"
    )
self.rolling_candidate = None
self.rolling_plan_result = None
self.rolling_plan_latched = False
self.rolling_stable_count = 0
self.rolling_terminal_start_ns = 0
self.rolling_last_submitted_stamp_ns = 0
self.rolling_planner = (
    LatestOnlyTopDownPlanner()
    if self.pick_execution_mode == "rolling_top_down"
    else None
)
self.create_subscription(
    PoseStamped,
    str(self.get_parameter("rolling_grasp_topic").value),
    self.on_rolling_grasp_candidate,
    10,
)
```

Add the callback:

```python
def on_rolling_grasp_candidate(self, msg: PoseStamped) -> None:
    if self.pick_execution_mode != "rolling_top_down":
        return
    if self.task_mode != "PICK" or self.rolling_plan_latched:
        return
    if not msg.header.frame_id:
        return
    self.rolling_candidate = deepcopy(msg)
```

- [ ] **Step 4: Add reset and cleanup hooks**

In `start_pick_sequence()`, reset all rolling state without changing legacy
state:

```python
if self.pick_execution_mode == "rolling_top_down":
    self.rolling_candidate = None
    self.rolling_plan_result = None
    self.rolling_plan_latched = False
    self.rolling_stable_count = 0
    self.rolling_terminal_start_ns = 0
    self.rolling_last_submitted_stamp_ns = 0
```

In `main()`, close the worker before destroying the node:

```python
finally:
    if node.rolling_planner is not None:
        node.rolling_planner.close()
    node.destroy_node()
    rclpy.shutdown()
```

- [ ] **Step 5: Run the focused tests**

Run the command from Step 2.

Expected: `2 passed`.

- [ ] **Step 6: Commit feature-gated intake**

```bash
git add \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py \
  ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py
git commit -m "feat: ingest rolling grasp candidates"
```

---

### Task 4: Non-Blocking Rolling Planning and Coordinated Whole-Body Pre-Grasp

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: latest `PoseStamped`, current measured joints, existing TF and
  switching weights
- Produces: newest valid rolling plan
- Produces: coordinated pre-grasp velocity blended by the existing arm weight
- Preserves: existing base twist and switching equations

- [ ] **Step 1: Add failing tests for non-blocking submission and simultaneous base/arm motion**

```python
def test_rolling_submission_returns_without_waiting_for_plan(monkeypatch):
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    node.joint_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
    ]
    node.pick_execution_mode = "rolling_top_down"
    node.rolling_candidate = PoseStamped()
    node.rolling_candidate.header.frame_id = "link0"
    node.rolling_candidate.header.stamp.sec = 1
    node.rolling_candidate.pose.position.x = 0.4
    node.rolling_candidate.pose.position.z = 0.3
    node.rolling_candidate.pose.orientation.w = 1.0
    node.rolling_last_submitted_stamp_ns = 0
    node.rolling_plan_result = None
    node.current_joints = dict(
        zip(node.joint_names, [0.0, -1.2, 1.8, -0.5, 1.4, 0.0])
    )
    node.joint_lower_limits = [-3.2] * 6
    node.joint_upper_limits = [3.2] * 6
    submitted = []
    node.rolling_planner = SimpleNamespace(
        submit=lambda request: submitted.append(request),
        poll=lambda: None,
    )

    node.submit_rolling_plan_if_needed(now_ns=1_100_000_000)

    assert len(submitted) == 1
    assert submitted[0].candidate_stamp_ns == 1_000_000_000


def test_existing_arm_weight_blends_base_deceleration_with_pregrasp_motion():
    blended = coordinated_joint_velocity(
        home_velocity=[0.0] * 6,
        pregrasp_velocity=[0.3, 0.2, -0.2, 0.1, -0.1, 0.05],
        arm_weight=0.5,
    )
    assert any(abs(value) > 0.0 for value in blended)
    assert controller.ArmYawRhoZPositionController.compute_switching(
        SimpleNamespace(),
        arm_target_rho=0.65,
        switching_rho=0.55,
        switching_alpha=20.0,
    ) < 0.5
```

Add this import to the test file with the new tests:

```python
from ee_switch_debug.rolling_top_down_pick import coordinated_joint_velocity
```

- [ ] **Step 2: Run the tests and verify the submission test fails**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_wrist_orientation.py::test_rolling_submission_returns_without_waiting_for_plan \
  src/ee_switch_debug/test/test_wrist_orientation.py::test_existing_arm_weight_blends_base_deceleration_with_pregrasp_motion \
  -q
```

Expected: the first test fails because
`submit_rolling_plan_if_needed` is undefined.

- [ ] **Step 3: Implement source-stamp extraction and request submission**

Add:

```python
@staticmethod
def message_stamp_ns(msg: PoseStamped) -> int:
    return (
        int(msg.header.stamp.sec) * 1_000_000_000
        + int(msg.header.stamp.nanosec)
    )

def submit_rolling_plan_if_needed(self, now_ns: int) -> None:
    if self.rolling_planner is None or self.rolling_candidate is None:
        return
    result = self.rolling_planner.poll()
    if result is not None:
        self.rolling_plan_result = result

    stamp_ns = self.message_stamp_ns(self.rolling_candidate)
    max_age = float(self.get_parameter("rolling_candidate_max_age_sec").value)
    if not candidate_is_fresh(stamp_ns, now_ns, max_age):
        return
    if stamp_ns <= self.rolling_last_submitted_stamp_ns:
        return
    if self.rolling_candidate.header.frame_id != self.arm_base_frame:
        self.last_debug = (
            f"rolling grasp frame {self.rolling_candidate.header.frame_id!r} "
            f"must equal {self.arm_base_frame!r}"
        )
        return

    pose = self.rolling_candidate.pose
    rotation = quaternion_to_matrix(pose.orientation)
    request = RollingPlanRequest(
        candidate_stamp_ns=stamp_ns,
        target_position=(
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z) + self.grasp_final_offset_z,
        ),
        target_rotation=tuple(tuple(float(value) for value in row) for row in rotation),
        seed_joints=tuple(
            float(self.current_joints[name]) for name in self.joint_names
        ),
        lower_limits=tuple(float(value) for value in self.joint_lower_limits),
        upper_limits=tuple(float(value) for value in self.joint_upper_limits),
    )
    self.rolling_last_submitted_stamp_ns = stamp_ns
    self.rolling_planner.submit(request)
```

- [ ] **Step 4: Use the rolling pre-grasp target inside the existing switching blend**

Add a helper that reuses measured joint feedback and existing limits:

```python
def rolling_pregrasp_velocity(self) -> list[float] | None:
    result = self.rolling_plan_result
    candidate = self.rolling_candidate
    if result is None or candidate is None:
        return None
    current = [float(self.current_joints[name]) for name in self.joint_names]
    pose = candidate.pose
    current_target = (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z) + self.grasp_final_offset_z,
    )
    if not rolling_plan_is_valid(
        result,
        now_ns=self.get_clock().now().nanoseconds,
        target_position=current_target,
        current_joints=current,
        max_candidate_age_sec=float(
            self.get_parameter("rolling_candidate_max_age_sec").value
        ),
        max_target_drift_m=float(
            self.get_parameter("rolling_target_drift_m").value
        ),
        max_seed_drift_rad=float(
            self.get_parameter("rolling_seed_drift_rad").value
        ),
    ):
        return None
    velocity, _ = fixed_target_joint_velocities(
        current=current,
        target=list(result.plan.pregrasp_joints),
        active_mask=[True] * 6,
        kp=self.top_down_joint_kp,
        velocity_limits=self.top_down_alignment_velocity_limits,
        tolerance=self.top_down_transit_joint_tolerance,
    )
    return self.apply_joint_limit_slowdown(velocity)
```

In the existing `RETURN_HOME` switching block, replace `tracking_velocity` with
the rolling pre-grasp velocity only for the opt-in mode:

```python
rolling_velocity = (
    self.rolling_pregrasp_velocity()
    if self.pick_execution_mode == "rolling_top_down"
    and self.task_mode == "PICK"
    else None
)
if rolling_velocity is not None:
    tracking_velocity = rolling_velocity
desired_velocity = coordinated_joint_velocity(
    home_velocity,
    tracking_velocity,
    arm_blend,
)
```

Do not change `mu`, `arm_mu`, `arm_blend`, `base_scale`, or
`compute_base_twist(...)`.

- [ ] **Step 5: Poll and submit once per control tick without blocking**

Immediately after the existing target transforms and measured joint state are
available in `on_timer()`, add:

```python
if (
    self.pick_execution_mode == "rolling_top_down"
    and self.task_mode == "PICK"
    and not self.rolling_plan_latched
):
    self.submit_rolling_plan_if_needed(self.get_clock().now().nanoseconds)
```

- [ ] **Step 6: Run focused and legacy controller tests**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_rolling_top_down_pick.py \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  -q
```

Expected: all tests pass.

- [ ] **Step 7: Commit coordinated rolling approach**

```bash
git add \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py \
  ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py
git commit -m "feat: blend rolling top-down pregrasp approach"
```

---

### Task 5: Terminal Latch, Descent Gate, Failure State, and Pick Handoff

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `TerminalGateInput`, newest valid rolling plan, existing control
  state and switching outputs
- Produces: immutable top-down plan latch
- Produces: `PICK:PLAN_FAILED` on terminal timeout
- Produces: `PICK:HOLD` after lift without an automatic home return

- [ ] **Step 1: Add failing tests for descent gating and no-home completion**

```python
def test_rolling_pick_cannot_latch_while_base_command_is_nonzero():
    gate = TerminalGateInput(
        control_state="ARM_TRACK",
        base_scale=0.01,
        target_safe=True,
        base_linear_command=0.10,
        base_angular_command=0.0,
        stable_cycles=10,
        required_stable_cycles=3,
        plan_valid=True,
    )
    assert not terminal_gate_ready(
        gate,
        max_base_scale=0.05,
        max_linear_command=0.02,
        max_angular_command=0.05,
    )


def test_rolling_pick_latches_existing_plan_without_replanning():
    node = controller.ArmYawRhoZPositionController.__new__(
        controller.ArmYawRhoZPositionController
    )
    result = SimpleNamespace(
        plan=SimpleNamespace(
            success=True,
            pregrasp_joints=[0.1] * 6,
            grasp_joints=[0.2] * 6,
            descent_waypoints=[],
        )
    )
    node.rolling_plan_result = result
    node.rolling_plan_latched = False
    node.top_down_plan = None
    node.top_down_stage = "WAIT_TARGET"
    node.pick_approach_stage = "POSITION"
    node.grasp_phase = "APPROACH"
    node.previous_velocity = [1.0] * 6
    node.get_logger = lambda: SimpleNamespace(warn=lambda message: None)

    node.latch_rolling_top_down_plan()

    assert node.top_down_plan is result.plan
    assert node.rolling_plan_latched
    assert node.top_down_stage == "APPROACH"
    assert node.previous_velocity == [0.0] * 6
```

Add this import to the test file with the new tests:

```python
from ee_switch_debug.rolling_top_down_pick import TerminalGateInput
```

- [ ] **Step 2: Run the tests and verify the latch test fails**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_wrist_orientation.py::test_rolling_pick_cannot_latch_while_base_command_is_nonzero \
  src/ee_switch_debug/test/test_wrist_orientation.py::test_rolling_pick_latches_existing_plan_without_replanning \
  -q
```

Expected: the second test fails because `latch_rolling_top_down_plan` is
undefined.

- [ ] **Step 3: Implement immutable latching**

```python
def latch_rolling_top_down_plan(self) -> None:
    self.top_down_plan = self.rolling_plan_result.plan
    self.top_down_plan_attempted = True
    self.rolling_plan_latched = True
    self.top_down_stage = "APPROACH"
    self.pick_approach_stage = "APPROACH"
    self.grasp_phase = "APPROACH"
    self.previous_velocity = [0.0] * 6
    self.clear_top_down_segment_state()
    self.get_logger().warn(
        "Rolling whole-body handoff latched top-down PICK plan"
    )
```

- [ ] **Step 4: Evaluate the contact gate from existing switching outputs**

Add:

```python
def update_rolling_terminal_gate(
    self,
    *,
    now_ns: int,
    base_scale: float,
    base_twist: Twist,
    target_safe: bool,
) -> bool:
    if self.rolling_plan_latched or self.rolling_candidate is None:
        return self.rolling_plan_latched
    current_joints = [
        float(self.current_joints[name]) for name in self.joint_names
    ]
    pose = self.rolling_candidate.pose
    current_target = (
        float(pose.position.x),
        float(pose.position.y),
        float(pose.position.z) + self.grasp_final_offset_z,
    )
    valid = rolling_plan_is_valid(
        self.rolling_plan_result,
        now_ns=now_ns,
        target_position=current_target,
        current_joints=current_joints,
        max_candidate_age_sec=float(
            self.get_parameter("rolling_candidate_max_age_sec").value
        ),
        max_target_drift_m=float(
            self.get_parameter("rolling_target_drift_m").value
        ),
        max_seed_drift_rad=float(
            self.get_parameter("rolling_seed_drift_rad").value
        ),
    )
    low_motion = (
        base_scale <= float(
            self.get_parameter("rolling_base_scale_threshold").value
        )
        and abs(float(base_twist.linear.x))
        <= float(self.get_parameter("rolling_base_linear_threshold").value)
        and abs(float(base_twist.angular.z))
        <= float(self.get_parameter("rolling_base_angular_threshold").value)
    )
    self.rolling_stable_count = (
        self.rolling_stable_count + 1 if low_motion else 0
    )
    ready = terminal_gate_ready(
        TerminalGateInput(
            control_state=str(self.control_state),
            base_scale=float(base_scale),
            target_safe=bool(target_safe),
            base_linear_command=float(base_twist.linear.x),
            base_angular_command=float(base_twist.angular.z),
            stable_cycles=self.rolling_stable_count,
            required_stable_cycles=int(
                self.get_parameter("rolling_stable_cycles").value
            ),
            plan_valid=valid,
        ),
        max_base_scale=float(
            self.get_parameter("rolling_base_scale_threshold").value
        ),
        max_linear_command=float(
            self.get_parameter("rolling_base_linear_threshold").value
        ),
        max_angular_command=float(
            self.get_parameter("rolling_base_angular_threshold").value
        ),
    )
    if ready:
        self.latch_rolling_top_down_plan()
    return ready
```

Before calling the gate, derive diagnostic gate values with the exact existing
switching and base-command functions. These values are observed by the gate;
they do not replace the command that the legacy branch publishes:

```python
rolling_mu = self.compute_switching(
    target_rho,
    self.switching_rho,
    self.switching_alpha,
)
rolling_base_scale = 1.0 - rolling_mu
rolling_base_twist = self.compute_base_twist(
    base_target.x,
    base_target.y,
    target_rho,
    rolling_base_scale,
)
self.update_rolling_terminal_gate(
    now_ns=now.nanoseconds,
    base_scale=rolling_base_scale,
    base_twist=rolling_base_twist,
    target_safe=self.is_transition_target_safe(wc_target_z, target_yaw),
)
```

Do not alter `compute_switching`, `compute_base_twist`, or the command selected
by the existing control-state branch.

- [ ] **Step 5: Prevent legacy phase advancement before the latch**

Guard `_update_task_phase(...)`:

```python
rolling_precontact = (
    self.pick_execution_mode == "rolling_top_down"
    and self.task_mode == "PICK"
    and not self.rolling_plan_latched
)
if not rolling_precontact:
    self._update_task_phase(pos_aligned, wrist_aligned, dt)
```

At the top of the next timer tick, if a rolling plan is latched, call the
existing fixed semantic executor and return:

```python
if (
    self.pick_execution_mode == "rolling_top_down"
    and self.task_mode == "PICK"
    and self.rolling_plan_latched
):
    self.run_semantic_fixed_orientation_pick(dt)
    return
```

- [ ] **Step 6: Add timeout failure without a legacy fallback**

Start `rolling_terminal_start_ns` when `ARM_TRACK` is first reached. If elapsed
time exceeds `rolling_terminal_timeout_sec` without a latch:

```python
self.top_down_stage = "PLAN_FAILED"
self.grasp_phase = "PLAN_FAILED"
self.previous_velocity = [0.0] * 6
self.cmd_pub.publish(Twist())
self.publish_position_command()
self.publish_task_state()
self.last_debug = "rolling top-down terminal timeout; descent prohibited"
```

`publish_task_state()` already emits `PICK:PLAN_FAILED` for a precomputed
top-down stage.

- [ ] **Step 7: Run controller, kinematics, task-manager, and mux regressions**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_rolling_top_down_pick.py \
  src/ee_switch_debug/test/test_wrist_orientation.py \
  src/ee_switch_debug/test/test_top_down_kinematics.py \
  src/ee_switch_debug/test/test_navigation_geometry.py \
  src/ee_switch_debug/test/test_navigation_cmd_mux.py \
  src/ee_switch_debug/test/test_pick_place_precache.py \
  -q
```

Expected: all selected tests pass.

- [ ] **Step 8: Commit the terminal handoff**

```bash
git add \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py \
  ros2_ws/src/ee_switch_debug/test/test_wrist_orientation.py
git commit -m "feat: latch rolling top-down pick after whole-body handoff"
```

---

### Task 6: Isolated Long-Range Top-Down Launch

**Files:**
- Create: `ros2_ws/src/ee_switch_debug/launch/yoloe_long_range_top_down_pick_place.launch.py`
- Create: `ros2_ws/src/ee_switch_debug/test/test_yoloe_long_range_top_down_launch.py`
- Verify unchanged: `ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py`

**Interfaces:**
- Produces: `ros2 launch ee_switch_debug yoloe_long_range_top_down_pick_place.launch.py`
- Preserves: all existing long-range topics, nodes, Nav2/hybrid parameters, task
  manager behavior, and place behavior

- [ ] **Step 1: Write a failing launch-isolation test**

```python
# ros2_ws/src/ee_switch_debug/test/test_yoloe_long_range_top_down_launch.py
from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_new_launch_preserves_long_range_stack_and_enables_only_top_down_pick():
    source = (
        ROOT / "launch" / "yoloe_long_range_top_down_pick_place.launch.py"
    ).read_text()
    for executable in (
        "sm_yoloe_vlm_node",
        "grasping_inference_node",
        "pick_place_task_manager",
        "navigation_cmd_mux",
        "controller_server",
        "planner_server",
        "bt_navigator",
        "arm_yaw_rho_z_position_controller",
    ):
        assert f'executable="{executable}"' in source
    assert '"pick_execution_mode": "rolling_top_down"' in source
    assert '"rolling_grasp_topic": "/sm_grasping/grasp_best"' in source
    assert '"return_home_after_pick": False' in source
    assert '"return_home_after_place": ParameterValue(' in source
    assert '"enable_hybrid_handoff", default_value="true"' in source
    assert '"hybrid_outer_distance_m", default_value="1.40"' in source
    assert '"hybrid_inner_distance_m", default_value="0.85"' in source
    assert source.count('executable="arm_yaw_rho_z_position_controller"') == 1


def test_existing_long_range_launch_does_not_enable_rolling_pick():
    source = (
        ROOT / "launch" / "florence_long_range_pick_place_control.launch.py"
    ).read_text()
    assert '"pick_execution_mode": "rolling_top_down"' not in source
    assert '"return_home_after_pick": False' not in source
```

- [ ] **Step 2: Run the launch test and verify the new file is missing**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test/test_yoloe_long_range_top_down_launch.py \
  -q
```

Expected: `FileNotFoundError` for the new launch file.

- [ ] **Step 3: Create the new launch from the proven long-range composition**

Copy the current content of
`florence_long_range_pick_place_control.launch.py` into
`yoloe_long_range_top_down_pick_place.launch.py`, then add only these controller
parameter overrides inside the existing controller parameter dictionary:

```python
"pick_execution_mode": "rolling_top_down",
"rolling_grasp_topic": "/sm_grasping/grasp_best",
"rolling_grasp_frame": "link0",
"enable_precomputed_top_down_sequence": False,
"top_down_use_fixed_reachable_orientation": True,
"top_down_blend_orientation_during_descent": False,
"top_down_minimum_orientation_fraction": 0.50,
"top_down_orientation_search_steps": 20,
"top_down_descend_max_xy_deviation": 0.015,
"top_down_descend_max_orientation_deviation": 0.035,
"top_down_segment_min_duration": 0.6,
"top_down_transit_joint_tolerance": 0.050,
"top_down_stage_acceleration": 1.6,
"top_down_lift_home_clearance": 0.06,
"top_down_alignment_velocity_limits": [0.70, 1.05, 1.20, 0.95, 0.95, 0.95],
"top_down_path_velocity_limits": [0.42, 0.56, 0.68, 0.56, 0.42, 0.56],
"top_down_radial_inward_offset": 0.025,
"pick_max_joint_excursion": [0.65, 2.50, 2.30, 3.00, 3.00, 3.00],
"return_home_after_pick": False,
```

Override the pick grasping node's target frame without changing its topics:

```python
parameters=[
    LaunchConfiguration("grasping_config_file"),
    {"target_frame": "link0"},
],
```

Do not alter the place grasping node or controller place parameters.

- [ ] **Step 4: Load both launch files and run the isolation tests**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ee_switch_debug --symlink-install
source install/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py --show-args >/dev/null
ros2 launch ee_switch_debug yoloe_long_range_top_down_pick_place.launch.py --show-args >/dev/null
python3 -m pytest \
  src/ee_switch_debug/test/test_yoloe_long_range_top_down_launch.py \
  src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py \
  -q
```

Expected: both launch loads exit 0 and all tests pass.

- [ ] **Step 5: Commit the isolated launch**

```bash
git add \
  ros2_ws/src/ee_switch_debug/launch/yoloe_long_range_top_down_pick_place.launch.py \
  ros2_ws/src/ee_switch_debug/test/test_yoloe_long_range_top_down_launch.py
git commit -m "feat: add long-range whole-body top-down launch"
```

---

### Task 7: Documentation, Full Regression, and Isaac Sim Acceptance

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`
- Verify: all files under `ros2_ws/src/ee_switch_debug/test/`

**Interfaces:**
- Documents the new launch and observable state flow
- Produces final automated verification evidence
- Defines the manual Isaac Sim acceptance run

- [ ] **Step 1: Add exact operating instructions**

Append:

```markdown
## Long-Range Whole-Body Top-Down Pick

This opt-in launch preserves the existing Nav2, hybrid handoff, direct-place,
safe-retreat, and place behavior. During final pick approach, base contribution
decreases while the arm simultaneously moves toward a rolling top-down
pre-grasp. Contact descent starts only after the existing switching logic has
reduced base motion below the configured contact thresholds.

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug yoloe_long_range_top_down_pick_place.launch.py \
  autostart:=false \
  enable_nav2:=true
```

Start a task:

```bash
ros2 topic pub --once /pick_place_task \
  std_msgs/msg/String "{data: 'can,box'}"
```

Monitor the handoff:

```bash
ros2 topic echo /base_control_mode
ros2 topic echo /base_control_blend
ros2 topic echo /arm_task_state
ros2 topic echo /pick_place_task_state
```

Expected pick states end with:

```text
PICK:APPROACH -> PICK:DESCEND -> PICK:GRASP -> PICK:LIFT -> PICK:HOLD
```

`PICK:PLAN_FAILED` means no fresh, valid, reachable top-down path was available
before the terminal timeout. The controller holds outside contact and does not
fall back to legacy descent.
```

- [ ] **Step 2: Run the entire package test suite**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ee_switch_debug --symlink-install
source install/setup.bash
python3 -m pytest src/ee_switch_debug/test -q
```

Expected: all tests pass with 0 failures.

- [ ] **Step 3: Verify protected source files and local configuration**

Run:

```bash
git diff origin/main -- \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/pick_place_task_manager.py \
  ros2_ws/src/ee_switch_debug/ee_switch_debug/navigation_cmd_mux.py \
  ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py
git status --short
```

Expected:

- no diff for the three protected files;
- `ros2_ws/dds_setting/` remains untracked and unstaged;
- only intended feature and documentation files are changed.

- [ ] **Step 4: Run Isaac Sim acceptance with recording**

With Isaac Sim publishing `/clock`, `/joint_states`, `/tf`,
`/rsd455/rgb`, `/rsd455/depth`, and `/rsd455/camera_info`, run the new launch
with `autostart:=false`. Place an obstacle on the direct route, start a pick
task, and record:

```bash
ros2 bag record \
  /joint_states \
  /joint_position_command \
  /cmd_vel_navigation \
  /cmd_vel_manipulation \
  /cmd_vel \
  /base_control_mode \
  /base_control_blend \
  /arm_task_state \
  /pick_place_task_state
```

Acceptance requires:

1. Nav2 routes around the obstacle.
2. `NAVIGATION` changes to `HYBRID`, then `MANIPULATION`.
3. Nonzero arm motion overlaps decreasing base velocity.
4. `PICK:DESCEND` starts only after base contact thresholds are satisfied.
5. Pick reaches `PICK:HOLD`.
6. A nearby place skips home and safe retreat.
7. A distant place still uses transport, safe retreat, Nav2, and legacy place.

- [ ] **Step 5: Commit documentation**

```bash
git add ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md
git commit -m "docs: explain long-range whole-body top-down pick"
```

- [ ] **Step 6: Run final verification and inspect the complete diff**

Run:

```bash
cd /home/kiro/Desktop/hw_ws
git diff --check origin/main...HEAD
git status --short --branch
git log --oneline --decorate origin/main..HEAD
```

Expected:

- `git diff --check` exits 0;
- no tracked changes remain uncommitted;
- the only untracked path is `ros2_ws/dds_setting/`;
- the commit list contains the design, plan, focused implementation, launch,
  and documentation commits.
