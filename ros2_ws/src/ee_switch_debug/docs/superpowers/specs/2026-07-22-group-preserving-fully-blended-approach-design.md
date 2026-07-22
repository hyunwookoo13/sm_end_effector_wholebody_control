# Group-Preserving Fully Blended Top-Down Approach Design

## Goal

Remove the stop-and-start appearance of the current
`YAW -> ARM_POSITION -> WRIST_ALIGN` approach without returning to a runtime
whole-arm IK or Jacobian controller. Joint-group goals remain independently
defined, but their precomputed trajectories execute together on one shared
timeline.

The validated contact sequence remains unchanged:

```text
BLENDED_APPROACH -> PREGRASP_VERIFY -> DESCEND -> GRASP
    -> LIFT -> HOME -> HOLD
```

## Separation of Responsibilities

Target generation, joint ownership, and time blending are separate concerns:

- **Joint 1 direction group:** owns only Joint 1 and targets the selected
  pregrasp yaw.
- **Joint 2/3 position group:** owns only Joint 2 and Joint 3. Its restricted
  rho/Z solver supplies the safe overhead-position guide point, and its final
  value includes the position compensation required by wrist alignment.
- **Joint 4/5/6 orientation group:** owns only Joint 4 through Joint 6 and
  targets the selected grasp orientation.
- **Trajectory composer:** combines the three precomputed group trajectories
  into one six-joint command. It does not solve IK and cannot change a group's
  endpoint.

The existing whole-arm fixed-orientation planner remains a final-pose
feasibility and vertical-descent oracle. It does not command live approach
motion. No live perception, target TF, whole-arm Jacobian, or competing joint
controllers participate after the PICK snapshot is latched.

## Blended Path Construction

Let normalized global approach time be `s` from 0 to 1. The composer uses the
existing compact cubic easing function for each progress curve.

### Joint 1

Joint 1 moves from the measured PICK-start value to `yaw_joints[0]` over the
full global interval. Joint 2 through Joint 6 receive no contribution from this
group.

### Joint 2/3

Joint 2/3 use two overlapping contributions:

```text
q23(s) = q23_start
       + A(s) * (q23_arm - q23_start)
       + C(s) * (q23_pregrasp - q23_arm)
```

`A(s)` is the overhead-position contribution. It starts at `s=0` and reaches
one at `s=0.75`. `C(s)` is wrist-position compensation. It starts at `s=0.10`
and reaches one at `s=1.0`. Both use compact easing within their windows.

Because the contribution windows overlap, Joint 2/3 do not stop at
`arm_joints`; that point shapes the path rather than becoming a mandatory
dwell. At `s=1`, the result is exactly the validated pregrasp Joint 2/3 target.

### Joint 4/5/6

Joint 4/5/6 move from their measured PICK-start values to the validated
pregrasp values over the full global interval. They remain independently owned
by the orientation group even though their motion occurs at the same time as
Joint 1 and Joint 2/3.

All primary groups therefore begin on the same shared trajectory, and the
robot has no artificial zero-velocity boundary between direction, reach, and
orientation alignment. This is temporal blending of group-generated targets,
not simultaneous runtime six-axis IK.

## Time Scaling

The trajectory duration is computed from the composed path, not selected by an
arbitrary dwell timer. The planner samples normalized joint derivatives and
chooses a duration that satisfies every configured per-joint velocity limit and
the existing approach acceleration limit. A configurable minimum duration
remains as a lower bound.

If the composed Joint 2/3 contributions momentarily add velocity or
acceleration, the entire shared timeline is stretched uniformly. Relative
group timing and the geometric path remain unchanged.

## Pre-Execution Safety Validation

Before any motion, at least 101 samples of the exact composed path are checked
for:

- hard joint limits and PICK-start excursion limits. A measured start may use
  the existing `0.01 rad` encoder/limit tolerance, but the path may never move
  farther outside the limit and its endpoint must be inside the hard limits;
- configured per-joint velocity and acceleration limits after time scaling;
- EE height above grasp FK Z plus the configured minimum approach clearance;
- finite joint values and a continuous path from measured start to pregrasp;
- exact final agreement with the validated pregrasp joint target.

Failure enters `PLAN_FAILED` with the gripper open and the base stopped. There
is no silent fallback to the simultaneous whole-arm APPROACH or the old strict
sequential executor.

## Controller Execution

The controller exposes one approach state, `PICK:BLENDED_APPROACH`. Each control
tick samples the immutable composed trajectory and publishes one position
command. Individual live controllers never write the same joint; the composer
is the sole command publisher.

At planned completion, residual measured-joint error is closed against the
same final pregrasp endpoint. `PREGRASP_VERIFY` then checks measured EE position
and orientation. DESCEND cannot begin until that verification succeeds.

DESCEND retains the fixed orientation, strict joint tolerance, monotonic-Z
validation, XY deviation limit, and orientation deviation limit. GRASP, LIFT,
the measured 6 cm LIFT-to-HOME handoff, continuous HOME motion, frozen PICK
snapshot, RESET behavior, and stopped mobile base remain unchanged.

## Configuration

Add `top_down_use_group_blended_approach`, defaulting to false in the shared
configuration and true only in the dedicated YOLOE top-down test launch. The
existing sequential-group mode remains available for comparison but is false
in that launch. Enabling both modes is an invalid configuration and must not
silently choose one.

The initial timing constants are fixed to:

- overhead-position completion: `s=0.75`;
- Joint 2/3 compensation start: `s=0.10`, selected because the recorded live
  geometry loses clearance when wrist motion outruns Joint 2/3 compensation;
- path validation samples: 101.

These become named ROS parameters only if simulator evidence shows that target
geometry requires tuning; the first implementation avoids unnecessary runtime
knobs.

## Verification

Pure trajectory tests must prove:

- the composed path starts at measured joints and ends exactly at pregrasp;
- each group changes only the joints it owns;
- Joint 2/3 compensation overlaps the overhead-position contribution;
- the full robot has no internal planned stop;
- time scaling respects velocity and acceleration limits;
- unsafe clearance, invalid limits, and non-finite paths are rejected.

Controller tests must prove the transition
`BLENDED_APPROACH -> PREGRASP_VERIFY -> DESCEND`, that invalid measured pose
blocks DESCEND, and that invalid or mutually enabled modes cannot fall back to
another executor. All existing target-freeze, descent, grasp, lift, HOME,
joint-limit, launch, and RESET regression tests must continue to pass.

Automated checks cannot prove visual naturalness. Simulator acceptance requires
a new video showing continuous direction/reach/orientation motion, no arm
twisting, no collision or clearance loss, accurate pregrasp placement, vertical
descent, successful grasp, and the existing continuous return home.
