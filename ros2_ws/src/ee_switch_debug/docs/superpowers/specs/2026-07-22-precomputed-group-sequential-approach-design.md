# Precomputed Joint-Group Sequential Approach Design

## Goal

Replace the active simultaneous six-joint APPROACH with the intended sequential
joint-group controller while preserving the current frozen target, reachable
top-down orientation, validated vertical descent, motion speed, grasp, lift, and
continuous LIFT-to-HOME behavior.

## Architectural Boundary

Whole-arm six-axis IK is retained only as a feasibility and safety oracle. It
may select a reachable final pregrasp/grasp orientation and validate the final
vertical descent, but it must not command the approach motion.

The approach executor commands these groups in order:

```text
YAW              Joint 1
ARM_POSITION     Joint 2 and Joint 3
WRIST_ALIGN      Joint 4, Joint 5, and Joint 6
                 plus bounded Joint 2 and Joint 3 position compensation
DESCEND          validated fixed-orientation descent
GRASP -> LIFT -> HOME -> HOLD
```

No stage may activate the next primary joint group before its own measured
completion condition succeeds.

## Planning

The first fresh post-PICK perception bundle remains immutable. The existing
fixed-orientation planner continues to determine a final pregrasp pose 10 cm
above the object and a grasp pose with one common reachable orientation. These
two whole-arm solutions are references and descent validators, not approach
commands.

Before motion, a new staged approach planner produces three path sections:

1. **YAW:** interpolate only Joint 1 from measured start to the selected
   pregrasp Joint 1 value. Joint 2 through Joint 6 remain at measured start.
2. **ARM_POSITION:** keep Joint 1 and Joint 4 through Joint 6 fixed. Solve only
   Joint 2 and Joint 3 for the pregrasp radial distance and height. The planar
   two-joint solver minimizes rho/Z error and rejects joint-limit violations.
3. **WRIST_ALIGN:** interpolate Joint 4 through Joint 6 toward the selected
   grasp orientation. At each wrist sample, re-solve only Joint 2 and Joint 3
   to keep the EE at the pregrasp rho/Z position. Joint 1 remains fixed.

Every section is fully generated before execution. A section is rejected if a
restricted solve fails, a hard joint bound is exceeded, EE clearance drops
below the configured pregrasp safety region, or consecutive joint steps exceed
their configured limits. There is no live perception or target-TF feedback
after planning.

## Execution

The state machine becomes:

```text
YAW -> ARM_POSITION -> WRIST_ALIGN -> PREGRASP_VERIFY
    -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD
```

Each stage follows only its precomputed waypoints using existing bounded
velocity, acceleration, soft-limit slowdown, and measured-joint completion
checks. `PREGRASP_VERIFY` checks the final EE position and orientation from
measured joints. Failure holds the open gripper above the object; it never
advances to DESCEND.

DESCEND keeps the current fixed-orientation plan, strict `0.025 rad` endpoint
tolerance, monotonic-Z validation, `0.015 m` XY deviation limit, and `0.035 rad`
orientation deviation limit. GRASP, vertical LIFT, the measured 6 cm clearance,
and velocity-preserving HOME handoff remain unchanged.

## Performance Strategy

The staged plan does not insert arbitrary dwell timers. A stage advances on
measured completion during the next control tick. YAW uses a Joint 1-specific
speed limit so it does not move conspicuously faster than the arm. ARM_POSITION
and WRIST_ALIGN use the existing compact transit S-curve, with Joint 2/3
compensation included in the wrist waypoints rather than performed by competing
live controllers.

The expected total approach duration may be slightly longer than simultaneous
whole-arm APPROACH because the primary groups execute sequentially. The design
prioritizes matching current smoothness, target accuracy, and grasp success;
speed will be tuned only within the existing velocity and acceleration bounds.

## Safety and Failure Behavior

- The mobile base remains stopped for the complete sequence.
- Joint 1 is immutable after YAW until the validated descent requires otherwise.
- ARM_POSITION cannot command Joint 4 through Joint 6.
- WRIST_ALIGN can command Joint 4 through Joint 6 and compensation on Joint 2
  and Joint 3 only; it cannot command Joint 1.
- APPROACH-to-DESCEND retains a complete measured pose verification and stop.
- RESET clears the target snapshot, staged plan, waypoint indices, trajectory
  state, and retained velocities.
- Whole-arm approach interpolation is removed from the dedicated test launch;
  no silent fallback to simultaneous six-axis APPROACH is allowed.

## GPD Extension

The planner consumes a generic grasp position and rotation after the existing
marker-to-EE frame conversion. Replacing the heuristic top-down pose with a GPD
pose changes the requested final pose but not the staged execution contract.
Joint 4 through Joint 6 follow the requested grasp orientation, while Joint 2
and Joint 3 compensate the position displaced by wrist alignment.

## Verification

Unit tests must prove each stage's active-joint mask, Joint 1 immutability after
YAW, Joint 2/3-only ARM_POSITION, Joint 2/3 compensation during WRIST_ALIGN, and
rejection of invalid restricted solutions. Controller tests must prove ordered
stage transitions and prevent DESCEND before measured pregrasp verification.

Regression tests must preserve target snapshot behavior, fixed-orientation
vertical descent, strict grasp tolerance, joint limits, compact transit
profiles, and continuous LIFT-to-HOME. All `ee_switch_debug` tests and the
package build must pass. Simulator video must then confirm equivalent grasp
success and smoothness; automated checks alone cannot establish equal physical
performance.
