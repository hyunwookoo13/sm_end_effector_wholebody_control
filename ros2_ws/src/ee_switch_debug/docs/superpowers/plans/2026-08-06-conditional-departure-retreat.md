# Conditional Departure Retreat Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reuse the existing rear-LiDAR-safe retreat before Nav2 departs for a remote pick in a consecutive task.

**Architecture:** Mark a one-shot departure retreat when a new task starts from `DONE`. Let the existing navigation skip decision clear it for close picks; otherwise enter the existing `SAFE_RETREAT` with a configurable continuation phase of `FIND_PICK`, then request Nav2 normally.

**Tech Stack:** Python 3.10, ROS 2 Humble, Nav2, pytest

## Global Constraints

- Modify only `pick_place_task_manager.py` and focused tests/documentation.
- Preserve first-task, close-pick, existing post-pick retreat, Nav2, precision control, and arm behavior.
- Reuse retreat distance `0.40 m`, speed `0.25 m/s`, and rear scan safety.
- Never send Nav2 motion while departure retreat is active or failed.

---

### Task 1: Add Consecutive-Task Retreat Decisions

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/pick_place_task_manager.py`
- Create: `ros2_ws/src/ee_switch_debug/test/test_consecutive_task_retreat.py`

**Interfaces:**
- Produces: `departure_retreat_for_new_task(previous_phase, enable_navigation) -> bool`
- Produces: `maybe_start_departure_retreat(kind, skip_navigation) -> bool`

- [ ] Write failing tests proving only `DONE` plus enabled navigation marks a
  retreat, a close pick clears the flag without retreat, and a remote pick
  commands `TRANSPORT` and starts retreat with continuation `FIND_PICK`.
- [ ] Run the new test file and verify failures are caused by missing behavior.
- [ ] Implement the two small decision helpers and initialize/reset
  `departure_retreat_pending` in `__init__` and `start_task`.
- [ ] Integrate the helper into `request_navigation` after the existing skip
  decision and Nav2 readiness check. Return `retreating` instead of sending a
  goal.
- [ ] Handle `retreating` in `FIND_PICK` without falling through to pick
  manipulation.
- [ ] Run the new tests and commit.

---

### Task 2: Add Retreat Continuation Phase

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/pick_place_task_manager.py`
- Test: `ros2_ws/src/ee_switch_debug/test/test_consecutive_task_retreat.py`

**Interfaces:**
- Changes: `start_safe_retreat(next_phase: str = "FIND_PLACE") -> None`
- Produces state: `retreat_next_phase`

- [ ] Write failing tests proving departure retreat completes in `FIND_PICK`
  while the default existing retreat completes in `FIND_PLACE`.
- [ ] Run the focused tests and verify RED.
- [ ] Store the continuation in `start_safe_retreat` and use it when traveled
  distance reaches `retreat_distance_m`.
- [ ] Preserve blocked-rear behavior and dynamic log/debug messages.
- [ ] Run the focused and existing task-manager/navigation tests and commit.

---

### Task 3: Full Verification

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

- [ ] Document that a remote consecutive pick performs the existing 0.40 m
  safe retreat, while close consecutive picks skip it.
- [ ] Build `sm_florence_2_vlm_ros2`, `sm_grasping_ros2`, and
  `ee_switch_debug`.
- [ ] Run all `ee_switch_debug`, YOLOE, and grasp tests plus `ament_flake8` and
  `git diff --check`.
- [ ] Confirm changed runtime scope contains only the task manager and no Nav2,
  perception, mux, or arm-controller files.
- [ ] Commit documentation and report the restart command with
  `enable_nav2:=true autostart:=false`.
