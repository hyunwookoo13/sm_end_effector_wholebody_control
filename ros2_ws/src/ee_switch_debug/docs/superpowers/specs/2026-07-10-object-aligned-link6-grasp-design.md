# Object-Aligned Link6 Grasp Design

## Goal

Align the gripper closing direction with the detected object's short axis by
rotating `joint6` continuously. The mobile base must not rotate to compensate
for grasp orientation. A horizontal can with a horizontally closing gripper
therefore requires approximately 90 degrees of `joint6` rotation, while a
diagonal approach may require any continuous angle such as 35, 60, or 70
degrees.

## Root Cause

The current controller converts the YOLOE mask orientation into a full target
quaternion, decomposes the relative wrist rotation as Y-Z-Y Euler angles, and
uses only the final Euler component for `joint6`. In the recorded run this
produced a `joint6` error of about `0.001 rad` from the start of `ARM_TRACK`, so
the controller issued no meaningful rotation command even though the object and
gripper axes were misaligned.

## Considered Approaches

1. Keep the Y-Z-Y decomposition and tune offsets. This is rejected because the
   discarded joint 4 and joint 5 components contain part of the object angle,
   so an offset cannot recover continuous alignment reliably.
2. Convert the image angle directly with camera yaw and robot yaw arithmetic.
   This is simple but fragile because it duplicates TF conventions and depends
   on camera mounting assumptions.
3. Project the object and gripper axes into a common TF frame and calculate a
   signed angle about the actual `joint6` axis. This is selected because it
   naturally accounts for camera mounting, robot heading, and diagonal arm
   approaches without changing base control.

## Data Flow

1. YOLOE estimates the undirected major axis of the selected object mask.
2. The task manager freezes that orientation together with the selected grasp
   target and transforms it into the target parent frame.
3. The arm controller reads the target orientation and current end-effector
   orientation in `link0`.
4. The target object's minor axis becomes the desired gripper closing axis.
5. The current closing axis and desired closing axis are projected onto the
   plane perpendicular to the current `joint6` rotation axis.
6. Their signed angular difference is added to the current `joint6` position.
   Since both are undirected axes, the equivalent solution requiring the least
   joint travel is selected.
7. The existing wrist-alignment gate blocks `DESCEND` until the `joint6` error
   is within tolerance.

## Controller Behavior

- `joint4` and `joint5` retain their existing grasp posture.
- `joint6` receives a continuous target rather than a fixed 90-degree target.
- Base linear and angular commands are not modified by orientation alignment.
- Configurable local-axis parameters describe the `joint6` rotation axis and
  gripper closing axis. The defaults follow the controller's existing Y-Z-Y
  wrist convention, and the existing link6 offset remains available for model
  calibration.
- If either projected axis is degenerate, the controller uses the existing
  fixed grasp posture and reports the fallback in the debug log.
- If no equivalent target is reachable within the `joint6` limits, the target
  is limited to the nearest valid position and `DESCEND` remains blocked rather
  than rotating the base or attempting a knowingly misaligned grasp.

## Diagnostics

The controller debug output will include the current `joint6`, desired
`joint6`, signed alignment delta, and whether object-axis alignment is valid.
This makes it possible to distinguish perception/TF errors from actuator or
Isaac Sim command errors.

## Tests

- Parallel object major axis and gripper closing axis produces approximately a
  90-degree rotation toward the object minor axis.
- A diagonal relative pose produces the corresponding continuous angle rather
  than snapping to 0 or 90 degrees.
- Reversing an undirected object axis does not change the selected grasp.
- The nearest equivalent solution is selected within joint limits.
- An unreachable target remains alignment-failed and cannot enter `DESCEND`.
- Existing pick/place, perception cache, and wrist tests continue to pass.

