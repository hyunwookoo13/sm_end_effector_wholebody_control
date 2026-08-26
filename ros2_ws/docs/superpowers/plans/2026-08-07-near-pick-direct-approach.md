# Near Pick Direct Approach Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent unnecessary retreat and re-approach for a nearby consecutive pick while retaining the existing safe retreat for remote picks.

**Architecture:** Extend the pure navigation geometry decision with an independent pick direct-approach threshold. Wire that parameter through the task manager and bringup launch while leaving all motion controllers unchanged.

**Tech Stack:** Python 3, pytest, ROS 2 Humble launch and parameters

## Global Constraints

- Default pick direct-approach distance is `1.20 m`.
- Remote picks retain the existing rear-LiDAR-safe retreat.
- Do not modify arm, base controller, perception, or Nav2 implementations.

---

### Task 1: Add the nearby-pick decision

**Files:**
- Modify: `src/sm_task_orchestrator/test/test_navigation_geometry.py`
- Modify: `src/sm_task_orchestrator/sm_task_orchestrator/navigation_geometry.py`

**Interfaces:**
- Consumes: `should_skip_navigation(...)`
- Produces: pick-specific direct-range skip decision and reason

- [ ] **Step 1: Write a failing test for a pick at 1.10 m with a 1.20 m limit.**
- [ ] **Step 2: Run the focused test and confirm it fails because Pick lacks a direct range.**
- [ ] **Step 3: Add `pick_direct_approach_distance_m` to the pure decision function.**
- [ ] **Step 4: Run navigation geometry tests and confirm they pass.**

### Task 2: Wire the parameter through orchestration and bringup

**Files:**
- Modify: `src/sm_task_orchestrator/sm_task_orchestrator/pick_place_task_manager.py`
- Modify: `src/sm_task_orchestrator/test/test_consecutive_task_retreat.py`
- Modify: `src/sm_bringup/launch/natural_language_pick_place.launch.py`
- Modify: `src/sm_bringup/README_PICK_PLACE.md`

**Interfaces:**
- Consumes: ROS parameter and launch argument `pick_direct_approach_distance_m`
- Produces: nearby Pick skips pending retreat; remote Pick still retreats

- [ ] **Step 1: Add a failing manager test showing a 1.10 m consecutive Pick does not retreat.**
- [ ] **Step 2: Run the focused test and confirm the missing parameter path fails.**
- [ ] **Step 3: Declare, load, pass, and expose the new parameter with default `1.20`.**
- [ ] **Step 4: Update the operator note with near and remote behavior.**
- [ ] **Step 5: Run package tests and build affected packages.**
