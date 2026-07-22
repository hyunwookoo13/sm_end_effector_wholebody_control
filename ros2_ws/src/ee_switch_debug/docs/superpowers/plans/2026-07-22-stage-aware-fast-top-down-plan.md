# Stage-Aware Fast Top-Down Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove multi-second semantic-stage settle delays and reduce trajectory time without relaxing final DESCEND accuracy.

**Architecture:** A pure stage-policy function selects strict or transit measured-joint tolerance. The existing semantic executor consumes that value while retaining bounded endpoint feedback, and the dedicated simulator launch supplies faster but bounded trajectory parameters.

**Tech Stack:** Python 3.10, ROS 2 Humble `rclpy`, NumPy, `pytest`, `colcon`

## Global Constraints

- State order remains `APPROACH -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD`.
- `DESCEND` completion tolerance remains exactly `0.025 rad`.
- `APPROACH`, `LIFT`, and `HOME` completion tolerance is `0.050 rad`.
- Fixed target snapshot, fixed descent/lift orientation, mobile-base stop, joint limits, and `0.10 rad` correction cap remain unchanged.
- Faster values are enabled only by `yoloe_top_down_grasp_test.launch.py`.
- Do not start or stop the user's simulator launch.

---

### Task 1: Stage-specific semantic completion policy

**Files:**
- Modify: `src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Test: `src/ee_switch_debug/test/test_wrist_orientation.py`

**Interfaces:**
- Produces: `semantic_stage_joint_tolerance(stage: str, strict_tolerance: float, transit_tolerance: float) -> float`
- Consumes: `top_down_joint_tolerance` and new `top_down_transit_joint_tolerance`

- [ ] **Step 1: Write the failing policy test.**

```python
def test_semantic_stage_tolerance_keeps_descend_strict():
    assert semantic_stage_joint_tolerance("DESCEND", 0.025, 0.050) == 0.025
    for stage in ("APPROACH", "LIFT", "HOME"):
        assert semantic_stage_joint_tolerance(stage, 0.025, 0.050) == 0.050
    assert semantic_stage_joint_tolerance("UNKNOWN", 0.025, 0.050) == 0.025
```

- [ ] **Step 2: Run the focused test and verify RED.**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_wrist_orientation.py -k semantic_stage_tolerance
```

Expected: import failure because `semantic_stage_joint_tolerance` does not exist.

- [ ] **Step 3: Implement the pure policy.**

```python
def semantic_stage_joint_tolerance(stage, strict_tolerance, transit_tolerance):
    normalized = str(stage).strip().upper()
    if normalized in ("APPROACH", "LIFT", "HOME"):
        return max(0.0, float(transit_tolerance))
    return max(0.0, float(strict_tolerance))
```

Declare/read `top_down_transit_joint_tolerance` with a conservative generic
default of `0.025`. In `command_semantic_top_down_segment`, pass the selected
stage tolerance to `fixed_target_joint_velocities` instead of the global strict
tolerance.

- [ ] **Step 4: Run the focused test and verify GREEN.**

Run the Step 2 command. Expected: PASS.

---

### Task 2: Faster dedicated launch configuration

**Files:**
- Modify: `src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `src/ee_switch_debug/config/arm_position_target_in.yaml`
- Test: `src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py`

**Interfaces:**
- Produces dedicated overrides for transit tolerance, velocity, acceleration, and minimum duration.

- [ ] **Step 1: Write failing launch-source assertions.**

Require the launch source to contain:

```python
assert '"top_down_transit_joint_tolerance": 0.050' in source
assert '"top_down_segment_min_duration": 0.6' in source
assert '"top_down_stage_acceleration": 1.6' in source
assert '[0.70, 1.05, 1.20, 0.95, 0.95, 0.95]' in source
assert '[0.42, 0.56, 0.68, 0.56, 0.42, 0.56]' in source
```

- [ ] **Step 2: Run the launch test and verify RED.**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug:$PYTHONPATH /usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py
```

Expected: assertions fail against the previous conservative values.

- [ ] **Step 3: Apply the dedicated overrides.**

Set `top_down_transit_joint_tolerance: 0.050`, minimum duration `0.6`, stage
acceleration `1.6`, alignment limits `[0.70, 1.05, 1.20, 0.95, 0.95, 0.95]`,
and path limits `[0.42, 0.56, 0.68, 0.56, 0.42, 0.56]`. Keep the YAML generic
default for transit tolerance at `0.025`.

- [ ] **Step 4: Run the launch test and verify GREEN.**

Run the Step 2 command. Expected: PASS.

---

### Task 3: Documentation and full verification

**Files:**
- Modify: `src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Documents the strict DESCEND/transit tolerance split and the expected need for visual validation.

- [ ] **Step 1: Update the fixed-orientation test section.**

Document that the dedicated test uses `0.050 rad` for non-contact transit stages,
retains `0.025 rad` for DESCEND, and still uses zero-velocity semantic endpoints.

- [ ] **Step 2: Run all package tests.**

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/ee_switch_debug:src/sm_florence_2_vlm_ros2:$PYTHONPATH \
  /usr/bin/python3 -m pytest -q src/ee_switch_debug/test
```

Expected: all tests pass.

- [ ] **Step 3: Build and check the diff.**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ee_switch_debug --symlink-install
git diff --check
```

Expected: build and whitespace check exit successfully.

- [ ] **Step 4: Hand off restart and PICK commands.**

Do not manipulate the running launch. Tell the user to restart it so the running
Python process loads the new controller, then send only `PICK`; no separate
`DESCEND` command is required.
