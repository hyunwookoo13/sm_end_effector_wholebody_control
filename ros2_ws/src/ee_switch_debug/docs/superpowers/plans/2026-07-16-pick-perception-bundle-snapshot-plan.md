# PICK Perception Bundle Snapshot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Freeze the first post-PICK ROI point cloud, grasp pose and gripper opening so RViz visualization and arm control consume one immutable perception bundle.

**Architecture:** Add an orchestration-layer relay that sits between YOLOE and `sm_grasping_ros2`, then between `sm_grasping_ros2` and its marker/TF consumers. The relay passes data through while idle, captures the first fresh ROI after PICK, rejects in-flight stale grasps during a settle window, then locks and republishes the resulting grasp bundle.

**Tech Stack:** ROS 2 Humble, Python 3, `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `pytest`, `colcon`.

## Global Constraints

- Preserve the existing canonical topic names used by RViz and downstream nodes.
- Put raw producer output on `_live` topics only in the top-down test launch.
- Never let pre-PICK ROI or grasp data own a new task.
- Keep YOLOE and `sm_grasping_ros2` mission-state agnostic.
- Release only on `RESET`, `PICK:HOLD`, `PICK:DONE`, or a repeated `PICK`.

---

### Task 1: Define Relay State Behavior with Failing Tests

**Files:**
- Create: `src/ee_switch_debug/test/test_pick_perception_snapshot.py`
- Create: `src/ee_switch_debug/ee_switch_debug/pick_perception_snapshot.py`

**Interfaces:**
- Consumes: raw `PointCloud2`, raw `PoseStamped`, raw `Float32MultiArray`, `/arm_task_command`, and `/arm_task_state`.
- Produces: canonical locked ROI, grasp and opening messages.

- [ ] Write tests for LIVE pass-through, PICK invalidation, first non-empty ROI
  capture, settle-window rejection, first grasp/opening lock, later-update
  rejection, timer republish, completion release, and repeated PICK.
- [ ] Run the new test file and verify it fails because
  `PickPerceptionSnapshot` does not exist.
- [ ] Implement the minimum relay state machine and message-copy behavior.
- [ ] Run the focused tests and verify they pass.

### Task 2: Connect the Top-Down Launch Through the Relay

**Files:**
- Modify: `src/ee_switch_debug/setup.py`
- Modify: `src/ee_switch_debug/package.xml`
- Modify: `src/ee_switch_debug/launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `src/ee_switch_debug/test/test_yoloe_top_down_grasp_launch.py`

**Interfaces:**
- Consumes: `pick_perception_snapshot` console entry point.
- Produces: canonical `/sm_florence_2_vlm/roi_pointcloud`,
  `/sm_grasping/grasp_best`, and `/sm_grasping/grasp_openings`.

- [ ] Add failing launch assertions for the relay executable and `_live` raw
  topic overrides.
- [ ] Run the launch test and verify RED.
- [ ] Register the relay executable and add `visualization_msgs` only if the
  implementation actually consumes marker messages.
- [ ] Start the relay in the launch.
- [ ] Override YOLOE `roi_pointcloud_topic` to
  `/sm_florence_2_vlm/roi_pointcloud_live`.
- [ ] Override grasp inference input to the canonical ROI and its best/opening
  outputs to `/sm_grasping/grasp_best_live` and
  `/sm_grasping/grasp_openings_live`.
- [ ] Keep the gripper marker and TF bridge on canonical locked topics.
- [ ] Run focused relay and launch tests and verify GREEN.

### Task 3: Regression Verification

**Files:**
- Verify: `src/ee_switch_debug/test`
- Verify: `src/sm_grasping_ros2/test`
- Verify: `src/sm_florence_2_vlm_ros2/test`

**Interfaces:**
- Consumes: completed relay and launch integration.
- Produces: test and build evidence.

- [ ] Run all three package test suites.
- [ ] Build `ee_switch_debug`, `sm_grasping_ros2`, and
  `sm_florence_2_vlm_ros2` with `--symlink-install`.
- [ ] Confirm installed launch and module contain the relay and `_live`
  remappings.
- [ ] Run `git diff --check` on all modified files.
