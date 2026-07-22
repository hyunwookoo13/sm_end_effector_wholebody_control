# First-Post-PICK Grasp Snapshot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make each PICK task consume and permanently lock exactly the first valid `/sm_grasping/grasp_best` received after the PICK command.

**Architecture:** `GraspTargetTfBridge` remains the single owner of the task snapshot. A PICK command invalidates the previous transform and enters a waiting state; the next successfully transformed grasp becomes an immutable position-and-orientation snapshot until task completion or reset. The existing controller timestamp gate prevents stale pre-command TF data from moving the arm while the bridge is waiting.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, `geometry_msgs`, `tf2_ros`, `pytest`, `colcon`.

## Global Constraints

- Never reuse a transform captured before the current PICK command.
- Freeze both translation and quaternion from the first valid post-command grasp.
- Ignore all later grasp updates until `RESET`, `PICK:HOLD`, or `PICK:DONE`.
- Do not consume the snapshot on an invalid frame or failed TF conversion.
- Preserve existing non-PICK live-preview and `freeze_on_arm_track` behavior.

---

### Task 1: Specify the PICK Snapshot State Transitions

**Files:**
- Modify: `src/ee_switch_debug/test/test_grasp_target_tf_bridge.py`
- Test: `src/ee_switch_debug/test/test_grasp_target_tf_bridge.py`

**Interfaces:**
- Consumes: `GraspTargetTfBridge.on_task_command(String)`, `on_grasp_best(PoseStamped)`, `on_task_state(String)`, and `on_timer()`.
- Produces: tests defining `pick_freeze_requested`, `pick_snapshot_active`, `latest_transform`, `target_frozen`, and broadcast behavior.

- [ ] **Step 1: Replace the stale-cache test with a failing fresh-boundary test**

```python
def test_pick_command_discards_existing_grasp_and_waits_for_fresh_sample():
    bridge, _ = make_bridge()
    stale = bridge.latest_transform

    bridge.on_task_command(String(data="PICK"))

    assert bridge.latest_transform is None
    assert bridge.pick_freeze_requested
    assert not bridge.pick_snapshot_active
    assert not bridge.target_frozen
    assert stale is not bridge.latest_transform
```

- [ ] **Step 2: Add a failing first-sample lock test**

```python
def test_first_valid_grasp_after_pick_is_locked_and_later_grasps_are_ignored():
    bridge, _ = make_bridge()
    bridge.on_task_command(String(data="PICK"))

    first = make_pose(x=0.40, y=0.10, z=0.70, qz=0.1, qw=0.995)
    second = make_pose(x=0.20, y=-0.30, z=0.55, qz=0.7, qw=0.714)
    bridge.on_grasp_best(first)
    locked = bridge.latest_transform
    bridge.on_grasp_best(second)

    assert bridge.latest_transform is locked
    assert bridge.latest_transform.transform.translation.x == pytest.approx(0.40)
    assert bridge.latest_transform.transform.rotation.z == pytest.approx(
        first.pose.orientation.z
    )
    assert bridge.pick_snapshot_active
    assert bridge.target_frozen
```

- [ ] **Step 3: Add a failing no-broadcast-while-waiting test**

```python
def test_timer_does_not_broadcast_old_target_while_waiting_for_first_pick_grasp():
    bridge, _ = make_bridge()
    bridge.on_task_command(String(data="PICK"))

    bridge.on_timer()

    assert bridge.broadcasts == []
```

- [ ] **Step 4: Add reset, completion and repeated-PICK coverage**

```python
@pytest.mark.parametrize("state", ["PICK:HOLD", "PICK:DONE"])
def test_pick_completion_releases_snapshot(state):
    bridge, _ = make_bridge()
    lock_first_post_pick_grasp(bridge)
    bridge.on_task_state(String(data=state))
    assert not bridge.pick_snapshot_active
    assert not bridge.target_frozen


def test_repeated_pick_discards_previous_locked_snapshot():
    bridge, _ = make_bridge()
    lock_first_post_pick_grasp(bridge)
    previous = bridge.latest_transform
    bridge.on_task_command(String(data="PICK"))
    assert bridge.latest_transform is None
    assert previous is not bridge.latest_transform
    assert bridge.pick_freeze_requested
```

- [ ] **Step 5: Run focused tests and verify RED**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/ee_switch_debug/test/test_grasp_target_tf_bridge.py
```

Expected: failures show that `PICK` currently freezes the pre-existing transform instead of invalidating it.

---

### Task 2: Implement First-Post-PICK Snapshot Ownership

**Files:**
- Modify: `src/ee_switch_debug/ee_switch_debug/grasp_target_tf_bridge.py`
- Test: `src/ee_switch_debug/test/test_grasp_target_tf_bridge.py`

**Interfaces:**
- Consumes: the state expectations from Task 1.
- Produces: `begin_pick_snapshot() -> None` and existing callbacks with fresh-boundary semantics.

- [ ] **Step 1: Add a single PICK-boundary helper**

```python
def begin_pick_snapshot(self) -> None:
    self.latest_transform = None
    self.target_latched = False
    self.target_frozen = False
    self.pick_snapshot_active = False
    self.pick_freeze_requested = True
    self.latest_update_time = self.get_clock().now()
    self.last_debug = "waiting for first valid grasp after PICK"
    self.get_logger().warn(
        f"Invalidated previous {self.target_frame}; "
        "waiting for first valid grasp after PICK"
    )
```

- [ ] **Step 2: Route every PICK command through the new boundary**

```python
def on_task_command(self, msg: String) -> None:
    command = msg.data.strip().upper()
    if command == "PICK":
        self.begin_pick_snapshot()
    elif command == "RESET":
        self.release_pick_snapshot("RESET command")
```

- [ ] **Step 3: Lock the first successfully converted full transform**

After assigning `self.latest_transform`, set:

```python
if self.pick_freeze_requested:
    self.target_frozen = True
    self.pick_snapshot_active = True
    self.pick_freeze_requested = False
    rotation = self.latest_transform.transform.rotation
    self.get_logger().info(
        f"Locked first post-PICK {self.target_frame}: {self.last_debug}, "
        f"q=[{rotation.x:.4f}, {rotation.y:.4f}, "
        f"{rotation.z:.4f}, {rotation.w:.4f}]"
    )
```

The existing early return for `target_frozen` then rejects all later position
and orientation updates.

- [ ] **Step 4: Keep `ARM_TRACK` subordinate to PICK ownership**

When `pick_freeze_requested` is true, `on_control_state("ARM_TRACK")` must only
log that it is waiting. It must not freeze an absent or pre-command target.
When `pick_snapshot_active` is true, state changes must not replace or release
the snapshot.

- [ ] **Step 5: Run focused tests and verify GREEN**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/ee_switch_debug/test/test_grasp_target_tf_bridge.py
```

Expected: all bridge tests pass.

---

### Task 3: Verify Launch Integration and Regression Safety

**Files:**
- Verify: `src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py`
- Verify: `src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py`
- Verify: `src/ee_switch_debug/ee_switch_debug/arm_yaw_rho_z_position_controller.py`

**Interfaces:**
- Consumes: `freeze_on_pick_command=True`, `/arm_task_command`, `/arm_task_state`, and controller `target_accept_after_ns`.
- Produces: evidence that the launch activates the new bridge behavior and the controller rejects pre-command TF.

- [ ] **Step 1: Run launch and bridge tests**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q \
  src/ee_switch_debug/test/test_grasp_target_tf_bridge.py \
  src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py
```

Expected: all selected tests pass.

- [ ] **Step 2: Run the complete package test suite**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q src/ee_switch_debug/test
```

Expected: all tests pass with no new failures.

- [ ] **Step 3: Build the affected package**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ee_switch_debug
```

Expected: `ee_switch_debug` finishes successfully.

- [ ] **Step 4: Confirm the installed launch and module expose the new behavior**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
rg -n "freeze_on_pick_command|begin_pick_snapshot|Locked first post-PICK" \
  build/ee_switch_debug/ee_switch_debug/grasp_target_tf_bridge.py \
  install/ee_switch_debug/share/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py
```

Expected: the bridge implementation and launch parameter are present.

- [ ] **Step 5: Review the final diff**

```bash
git diff --check
git diff -- \
  src/ee_switch_debug/ee_switch_debug/grasp_target_tf_bridge.py \
  src/ee_switch_debug/test/test_grasp_target_tf_bridge.py
```

Expected: no whitespace errors and only first-post-PICK snapshot changes.
