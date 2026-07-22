# Precomputed Top-Down Sequence Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Execute one stable top-down pick from a frozen grasp pose using a precomputed six-joint plan and grouped joint motion.

**Architecture:** A pure kinematics module models the actual OMY-F3M chain and computes the pre-grasp plus vertical descent waypoints. The ROS controller latches this plan once, executes joint1, joints2-3, and joints4-6 in order, then follows the fixed descent and lift paths without reactive Cartesian compensation.

**Tech Stack:** Python 3, NumPy, ROS 2 Humble `rclpy`, `pytest`, `colcon`

## Global Constraints

- Pre-grasp clearance is exactly 0.10 m for this top-down test.
- The target pose and computed joint plan are immutable during an active pick.
- IK failure stops the pick instead of enabling the old reactive fallback.
- Existing PLACE and return-home behavior remain available.
- Joint hard limits and PICK-start excursion limits are enforced.

---

### Task 1: Six-axis kinematics and fixed Cartesian path planner

**Files:**
- Create: `ee_switch_debug/top_down_kinematics.py`
- Create: `test/test_top_down_kinematics.py`
- Modify: `package.xml`
- Modify: `setup.py`

**Interfaces:**
- Produces: `forward_kinematics(joints) -> (position, rotation)`
- Produces: `solve_pose_ik(position, rotation, seed, lower, upper) -> IKResult`
- Produces: `plan_top_down_sequence(position, rotation, current, lower, upper, clearance, spacing) -> TopDownPlan`

- [ ] **Step 1: Write failing FK, IK, clearance, continuity and failure tests.**

- [ ] **Step 2: Run `pytest -q test/test_top_down_kinematics.py` and verify the missing-module failure.**

- [ ] **Step 3: Implement the USD-derived FK, damped least-squares IK, and fixed vertical waypoint planner.**

- [ ] **Step 4: Add the NumPy runtime dependency and run the focused tests to PASS.**

### Task 2: Grouped fixed-target PICK executor

**Files:**
- Modify: `ee_switch_debug/arm_yaw_rho_z_position_controller.py`
- Modify: `test/test_wrist_orientation.py`

**Interfaces:**
- Consumes: `TopDownPlan.pregrasp_joints`, `TopDownPlan.descent_waypoints`
- Produces: one-shot stages `YAW -> ARM -> WRIST -> DESCEND -> GRASP -> LIFT -> HOME`

- [ ] **Step 1: Write failing tests for group masks, measured-joint completion, one-shot plan latching, and reset clearing.**

- [ ] **Step 2: Run the focused controller tests and verify they fail for the new behavior.**

- [ ] **Step 3: Add parameters and plan state, latch the plan on the first fresh target, and bypass reactive PICK control.**

- [ ] **Step 4: Implement fixed joint-target motion, vertical waypoint descent, grasp, reversed lift, and safe IK-failure hold.**

- [ ] **Step 5: Run focused controller tests to PASS.**

### Task 3: Enable and verify the top-down demonstration

**Files:**
- Modify: `config/arm_position_target_in.yaml`
- Modify: `launch/yoloe_top_down_grasp_test.launch.py`
- Modify: `README_PICK_PLACE.md`

**Interfaces:**
- Produces: launch defaults that enable one-shot precomputed top-down planning at 0.10 m clearance.

- [ ] **Step 1: Add a failing launch/config test for precomputed mode and the 0.10 m clearance.**

- [ ] **Step 2: Enable the new mode in the top-down launch and document observable states and failure behavior.**

- [ ] **Step 3: Run all `ee_switch_debug` tests.**

- [ ] **Step 4: Build with `colcon build --packages-select ee_switch_debug --symlink-install`.**

- [ ] **Step 5: Run the launch, issue one `PICK`, and record a result video for visual verification.**
