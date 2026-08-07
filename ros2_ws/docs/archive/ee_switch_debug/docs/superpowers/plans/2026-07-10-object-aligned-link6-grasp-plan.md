# Object-Aligned Link6 Grasp Implementation Plan

## Scope

Replace the ineffective link6 Euler-component extraction with continuous axis
alignment. Keep all mobile-base control behavior unchanged.

## Implementation Steps

1. Add controller unit tests for a pure axis-alignment helper:
   - parallel object major axis requires a 90-degree link6 correction toward
     the object minor axis;
   - diagonal geometry produces a continuous non-cardinal correction;
   - reversing the undirected object axis selects the same grasp;
   - the nearest equivalent target is selected within joint limits;
   - invalid or unreachable projections report alignment failure.
2. Run the new tests and confirm that they fail because the helper does not yet
   exist.
3. Implement vector projection, signed angle calculation, undirected-axis
   normalization, and feasible joint-target selection in
   `arm_yaw_rho_z_position_controller.py`.
4. Integrate the helper into PICK control using the current end-effector TF and
   frozen target orientation. Preserve the existing fixed posture for joints 4
   and 5 and preserve the existing full/off modes.
5. Add YAML parameters for the USD-verified local axes:
   - joint6 rotation axis: `Y`;
   - gripper closing axis: `X`;
   - target object minor axis: `Y`.
6. Require both numerical wrist tolerance and valid axis alignment before
   entering `DESCEND`. Never modify base commands for orientation correction.
7. Extend controller diagnostics with current/desired joint6, correction angle,
   and alignment validity.
8. Run focused tests, all package tests, and a three-package ROS2 build.

## Verification Evidence

- Unit tests demonstrate 90-degree and arbitrary-angle behavior.
- Existing perception, task-cache, wrist, and grasp tests remain green.
- `colcon build` succeeds for `ee_switch_debug`, `sm_grasping_ros2`, and
  `sm_florence_2_vlm_ros2`.

