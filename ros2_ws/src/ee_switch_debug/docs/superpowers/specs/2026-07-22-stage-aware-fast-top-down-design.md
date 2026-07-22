# Stage-Aware Fast Top-Down Sequence Design

## Goal

Reduce the successful top-down PICK sequence from the recorded approximately
58 seconds while preserving the fixed wrist orientation, immutable grasp target,
vertical descent geometry, and successful grasp shown in the 13:04:42 recording.

## Evidence

The recording shows a complete successful sequence, but the motion contains
three visible settle intervals of roughly four to six seconds. Motion begins at
about 16 seconds and ends at about 74 seconds. The current executor applies the
same `0.025 rad` measured-joint completion tolerance to APPROACH, DESCEND, LIFT,
and HOME. It also uses conservative velocity arrays for every fixed-orientation
segment. Earlier live inspection showed a stationary joint3 tracking residual
of approximately `0.043 rad`, so non-contact stages can spend several seconds
correcting an error that is visually and operationally acceptable.

## Considered Approaches

### Increase one global tolerance

This is the smallest change, but it would also relax the final DESCEND pose and
could close the gripper before the grasp endpoint is accurate. It is rejected.

### Stage-aware completion and bounded speed increase

Use a transit tolerance for APPROACH, LIFT, and HOME while preserving the strict
DESCEND tolerance. Increase semantic trajectory velocity and acceleration limits
by a bounded amount. This directly addresses the observed delays without
changing planning, target latching, or grasp geometry. This is selected.

### Continuous velocity blending between all stages

This could eliminate every zero-velocity endpoint, but it overlaps semantic
stages and weakens the sequential behavior required for safe top-down grasping.
It is deferred until collision-aware trajectory execution is available.

## Execution Design

The state sequence remains unchanged:

```text
APPROACH -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD
```

The semantic segment executor selects completion tolerance by stage:

- `DESCEND`: `0.025 rad`, preserving the existing grasp accuracy;
- `APPROACH`, `LIFT`, and `HOME`: `0.050 rad`, allowing safe transit stages to
  advance through the measured actuator residual seen in the simulator.

The bounded endpoint feedback remains active whenever a stage exceeds its own
tolerance. It remains capped at `0.10 rad`, so changing the completion policy
does not introduce unbounded commands.

The dedicated top-down test launch increases limits to:

- alignment/HOME velocity: `[0.70, 1.05, 1.20, 0.95, 0.95, 0.95] rad/s`;
- DESCEND/LIFT velocity: `[0.42, 0.56, 0.68, 0.56, 0.42, 0.56] rad/s`;
- semantic stage acceleration: `1.6 rad/s^2`;
- minimum segment duration: `0.6 s`.

These values are only enabled in `yoloe_top_down_grasp_test.launch.py`. Generic
mission defaults remain conservative.

## Safety and Failure Behavior

- The mobile base remains disabled for the full test sequence.
- The first post-`PICK` target and fixed-orientation IK plan remain immutable.
- DESCEND retains the existing strict measured-joint tolerance.
- Fixed-orientation descent validation, hard joint limits, and PICK excursion
  bounds remain unchanged.
- Every segment still uses quintic time scaling with zero endpoint velocity and
  acceleration; this change shortens waits but does not overlap stages.
- If measured error exceeds the stage tolerance, bounded endpoint feedback still
  runs instead of advancing blindly.
- `RESET` continues to clear all planned and settling state.

## Verification

Automated tests must prove that stage tolerance selection is strict for DESCEND,
relaxed only for APPROACH/LIFT/HOME, and rejects unknown stages conservatively.
Launch tests must verify the dedicated faster limits. All `ee_switch_debug`
tests and the package build must pass.

The simulator launch will not be started automatically. A new recording must
confirm that descent remains centered and accurate, the can remains held during
lift and HOME, and visible settle intervals are substantially reduced.
