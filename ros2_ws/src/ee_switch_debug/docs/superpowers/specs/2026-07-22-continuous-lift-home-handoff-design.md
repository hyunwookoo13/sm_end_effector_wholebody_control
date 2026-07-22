# Continuous LIFT-to-HOME Handoff Design

## Goal

After a successful grasp, lift the object vertically by a measured 6 cm and
transition toward HOME without stopping. Preserve the existing full stop and
alignment check between APPROACH and DESCEND.

## Selected Behavior

The semantic sequence remains:

```text
APPROACH -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD
```

APPROACH, DESCEND, and GRASP are unchanged. During LIFT, the controller compares
the measured end-effector height against the planned grasp-pose height. Once the
measured EE has risen by `0.06 m`, it changes the semantic stage to HOME without
zeroing the current commanded joint velocity.

HOME then uses the existing measured-joint proportional target controller and
acceleration limiter rather than starting a new zero-velocity time-scaled
segment. The acceleration limiter gradually bends the velocity vector from the
vertical lift direction toward the HOME direction. HOME still finishes only
when all six measured joints meet the existing transit tolerance.

## Why This Is Not a Raw Interrupt

Calling the existing stage transition directly would set every stored velocity
to zero and start HOME from rest, recreating the visible pause. The continuous
handoff therefore has two explicit responsibilities:

1. retain the most recent bounded LIFT command velocity;
2. clear only the old time-scaled segment state while keeping that velocity as
   the initial condition for the acceleration-limited HOME controller.

The time-scaled semantic segment executor records its latest bounded commanded
velocity from consecutive position samples. Normal semantic transitions still
clear that velocity. Only the verified LIFT-to-HOME transition preserves it.

## Clearance Decision

The threshold is based on measured robot state, not trajectory time or command
percentage:

```text
measured_ee_z >= planned_grasp_ee_z + 0.06 m
```

Both heights are computed in `link0` using the existing forward kinematics. A
configurable parameter named `top_down_lift_home_clearance` defaults to `0.06`.
If the threshold is not reached, LIFT continues to the existing pregrasp target
and then transitions to HOME normally, so unreachable or lagging measurements
cannot deadlock the sequence.

## Safety and Scope

- The gripper remains closed throughout LIFT, HOME, and HOLD.
- The mobile base remains disabled for the top-down test.
- Joint velocity limits, joint-limit slowdown, acceleration limiting, and HOME
  measured-joint tolerance remain active.
- APPROACH-to-DESCEND continues to stop completely before vertical descent.
- DESCEND retains the minimum-jerk quintic profile and strict `0.025 rad`
  completion tolerance.
- The handoff is enabled only for `LIFT -> HOME`; unknown stage changes and RESET
  clear stored velocity conservatively.
- If `return_home_after_pick` is false, LIFT retains its existing completion and
  HOLD behavior; no early HOME handoff occurs.

## Verification

Pure tests must prove that clearance is decided from measured EE height and not
elapsed time, including values just below and at the 6 cm boundary. Controller
tests must prove that the early handoff occurs only during LIFT with
`return_home_after_pick`, preserves the current velocity, and leaves
APPROACH-to-DESCEND zero-velocity behavior unchanged.

All `ee_switch_debug` tests and the package build must pass. The ROS launch will
not be started automatically. A simulator recording must confirm a vertical
6 cm lift, no visible stop at the handoff, no table/object collision, no link1
snap, and successful arrival at HOME while retaining the object.
