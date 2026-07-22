# Stage-Aware Compact S-Curve Design

## Goal

Reduce the approximately three-second visually stationary interval around the
APPROACH, LIFT, and HOME segment boundaries without overlapping semantic stages
or weakening the precise DESCEND-to-GRASP stop.

## Observed Cause

Every semantic motion segment currently uses the same minimum-jerk quintic time
scaling. That profile intentionally has zero velocity and zero acceleration at
both endpoints. It travels only about 5.8 percent of the commanded distance in
the first 20 percent of a segment, so a half-second video sample can make a
moving arm appear stationary for several consecutive frames. The same slow tail
is repeated before the next segment begins from rest.

The GRASP dwell is only 0.1 seconds and is not the main source of the visible
pause. Measured-joint endpoint feedback remains a separate safeguard and is not
removed by this change.

## Considered Approaches

### Increase every velocity and acceleration limit

This is easy but makes DESCEND and link1 more abrupt, and it does not directly
remove the long, low-motion portion of the current quintic profile. It is not
selected.

### Use a compact transit S-curve while retaining quintic DESCEND

APPROACH, LIFT, and HOME use a cubic position S-curve,
`s(u) = 3u^2 - 2u^3`. It retains exact endpoints and zero endpoint velocity but
moves 10.4 percent of the distance in the first 20 percent of the duration. Its
duration is calculated from its exact normalized peak velocity `1.5` and peak
acceleration `6.0`, so the configured joint velocity and acceleration bounds
remain enforced. DESCEND retains the existing minimum-jerk quintic scaling.
This is selected because it addresses the visible pause without stage overlap.

### Blend nonzero velocity across semantic boundaries

This is potentially fastest but would make APPROACH-to-DESCEND and
LIFT-to-HOME overlap. It conflicts with the required sequential behavior and is
deferred until collision-aware Cartesian trajectory execution is available.

## Execution Design

The state machine remains unchanged:

```text
APPROACH -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD
```

The pure trajectory module adds a compact S-curve scaler, its bound-aware
duration calculation, and a sampler. A pure stage-policy function selects the
profile:

- `APPROACH`, `LIFT`, `HOME`: compact cubic S-curve;
- `DESCEND`: existing minimum-jerk quintic;
- unknown stages: existing minimum-jerk quintic as the conservative fallback.

The semantic segment executor uses the selected duration and sampler. It still
starts each segment from measured joint positions, commands one immutable goal,
and waits for measured-joint tolerance after planned time. No stage starts with
nonzero velocity and no stage is allowed to overlap another.

Existing dedicated-launch velocity limits, `1.6 rad/s^2` acceleration, and
`0.6 s` minimum duration remain unchanged for the first simulator validation.
This isolates the effect of the profile change instead of combining it with a
second speed increase.

## Safety and Failure Behavior

- DESCEND keeps quintic zero-velocity and zero-acceleration endpoints and the
  strict `0.025 rad` completion tolerance before GRASP.
- APPROACH, LIFT, and HOME retain zero endpoint velocity, bounded acceleration,
  hard joint limits, and bounded endpoint feedback.
- The grasp target, fixed wrist orientation, precomputed IK endpoints, mobile
  base lock, and RESET behavior are unchanged.
- Unknown or malformed stage names use the conservative quintic profile.
- The cubic transit profile has a finite acceleration step at its endpoints;
  this initial change is restricted to the existing position-command simulator
  path. Hardware deployment requires validation through the robot trajectory
  controller or a later jerk-limited profile.

## Verification

Unit tests must prove exact endpoints, midpoint symmetry, zero numerical
endpoint velocity, and duration compliance with the `1.5` velocity and `6.0`
acceleration constants. Stage-policy tests must prove that only APPROACH, LIFT,
and HOME select the compact profile and DESCEND remains quintic.

Controller tests must prove that transit stages call the compact sampler while
DESCEND calls the quintic sampler. The full `ee_switch_debug` test suite and
package build must pass. The ROS launch will not be started automatically; a
new simulator recording must verify that the visible pause is reduced without
causing link1 snapping, object contact during APPROACH, or loss of the can
during LIFT and HOME.
