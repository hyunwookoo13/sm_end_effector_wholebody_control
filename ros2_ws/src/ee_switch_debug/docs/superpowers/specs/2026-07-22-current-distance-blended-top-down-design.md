# Current-Distance Blended Top-Down Grasp Design

## Goal

Pick the currently detected can without moving the mobile base, while retaining
a 10 cm pre-grasp clearance and finishing in the requested top-down grasp pose.

## Confirmed Failure

The frozen grasp target is approximately `[0.399, -0.194, 0.295]` in `link0`,
or 44.4 cm away in the horizontal plane.  Requiring the final top-down wrist
orientation already at 10 cm above that target produces no valid IK solution.
The controller correctly entered `PICK:PLAN_FAILED` rather than moving toward
an invalid solution.

Moving joint2 and joint3 to the pre-grasp solution before moving the wrist is
also unsafe at this distance.  Forward-kinematics replay places the EE at only
2.3 cm above `link0` during that intermediate stage.  By contrast, interpolating
all six planned joints together keeps the EE between approximately 39.5 cm and
52.4 cm during the same approach.

## Considered Approaches

### Selected: precomputed synchronized approach and blended descent

Compute the complete trajectory before moving.  Use a reachable orientation at
the 10 cm pre-grasp, then interpolate both position and orientation during the
descent so the final waypoint has the requested top-down orientation.  This
keeps the current base pose and avoids reactive controllers fighting each
other.

### Rejected: reduce clearance to 0-2 cm

This makes the final pose nearly reachable but removes the clearance required
to align safely above the object.

### Deferred: move or rotate the mobile base

A collision-aware base pose could improve arm reach, but it requires the robot
footprint, table geometry and Nav2 goal selection.  It is outside this focused
arm-controller demonstration and a straight 24 cm approach would collide with
the table.

## Target Convention and Radial Compensation

`sm_grasping_ros2` currently adds a 4 cm outward radial offset to the detected
object centre.  The current-distance top-down solution applies a configurable
2.5 cm inward compensation in the arm planner, leaving a net 1.5 cm outward
offset from the measured centroid.  Offline IK reproducing the live target is
exact at this compensated target while remaining inside the hard joint limits.

The existing marker-to-EE axis conversion remains unchanged:

- marker `+X` approach maps to OMY EE `-Y`;
- marker `+Y` closing maps to OMY EE `+X`.

## Planning Pipeline

1. Accept the first fresh post-`PICK` grasp target and freeze it.
2. Apply the configured 2.5 cm inward radial TCP compensation.
3. Convert marker orientation to the OMY EE convention.
4. Compute a reachable pre-grasp pose 10 cm above the compensated target while
   retaining the current EE orientation.
5. Generate a synchronized joint-space approach from the current joints to the
   pre-grasp joints.  Sample every segment with forward kinematics before
   accepting the plan.
6. Generate 1 cm Cartesian descent waypoints.  Position moves vertically from
   pre-grasp to grasp while orientation uses shortest-path quaternion
   interpolation from the reachable pre-grasp orientation to top-down.
7. Solve each waypoint from the previous solution to retain branch continuity.
8. Reject the entire plan before motion if any waypoint violates a hard joint
   limit, exceeds tolerance, or drops below the configured EE clearance.
9. Execute the immutable waypoint sequence with velocity and acceleration
   limits, close the gripper, replay the descent in reverse, then return home.

## Control Behaviour

The approach trajectory moves multiple joints together, but it is not the old
simultaneous reactive controller.  Every joint target is computed once before
motion, all joints follow the same waypoint clock, and no joint recomputes its
goal from a changing TF.  This removes the feedback conflict that previously
twisted the arm.

After the plan is latched, perception and marker updates are ignored until the
sequence ends or a new command resets it.

## Safety and Failure Handling

- No base motion is commanded.
- Configured hard joint limits always apply.
- The current-distance launch expands only the PICK-start excursion envelope
  enough to reach the already configured hard limits.
- The gripper remains open throughout approach and descent.
- Planning failure produces `PICK:PLAN_FAILED` and zero arm/base motion.
- A new `PICK` or `RESET` clears the failed plan and requests a fresh target.
- Isaac collision physics remains the final integration guard; the first run
  must be observed at low speed before increasing trajectory speed.

## Test Criteria

- The recorded live target produces a valid 10 cm pre-grasp and descent plan.
- The approach never drops the EE below the configured clearance.
- The final compensated pose error is at most 5 mm and orientation error is at
  most 0.03 rad.
- The first and final descent orientations match the reachable pre-grasp and
  requested top-down orientations respectively.
- The active target and waypoint list remain unchanged after planning.
- Existing target snapshot, task transition, PLACE and return-home tests pass.
