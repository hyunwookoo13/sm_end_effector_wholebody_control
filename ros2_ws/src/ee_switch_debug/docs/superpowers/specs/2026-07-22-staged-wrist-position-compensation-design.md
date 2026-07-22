# Staged Wrist Position Compensation Design

## Goal

Prevent the PICK controller from deadlocking in `WRIST` when wrist alignment
moves the end effector away from the frozen pre-grasp position. Wrist alignment
must continue slowly while link2 and link3 make bounded position corrections,
then the existing state machine may enter `PREGRASP` and descend.

## Observed Failure

The 2026-07-22 recording and matching ROS log show this sequence:

1. The first post-PICK grasp pose is frozen correctly.
2. `POSITION` reaches the pre-grasp region and changes to `WRIST`.
3. Wrist movement creates more than 5 cm of rho or z error.
4. Recovery sets all wrist velocities to zero.
5. Link2 and link3 reach their PICK-start-relative excursion bounds before
   recovering the position.
6. Recovery cannot exit, wrist alignment cannot finish, and the controller
   never reaches `PREGRASP` or `DESCEND`.

## Considered Approaches

1. Remove recovery and command all six joints at their normal speeds. This is
   simple but can create the uncontrolled twisting the staged controller was
   introduced to prevent.
2. Keep stopping the wrist and enlarge every PICK excursion limit. This may
   postpone the deadlock, but it preserves the circular dependency between
   wrist completion and position recovery.
3. Keep the staged sequence, permit a very small wrist velocity during
   recovery, and let only link2 and link3 use their configured hard safety
   bounds while recovering. This is selected because it breaks the circular
   wait while preserving slow, ordered motion and absolute joint safety.

## Selected Control Behavior

- `POSITION` remains unchanged: link1, link2 and link3 first bring the EE over
  the frozen target while the wrist stays in its approach shape.
- `WRIST` remains the only stage that aligns link4, link5 and link6 to the
  grasp orientation.
- If rho or z error crosses the recovery threshold, the controller enters the
  existing `RECOVER` motion profile.
- In `RECOVER`, link2 and link3 continue position correction at their existing
  moderate speed limits while link4, link5 and link6 continue at a new small
  wrist limit of `0.08 rad/s` instead of stopping.
- From `WRIST` through the remaining PICK stages, link2 and link3 use the
  configured absolute joint lower and upper limits. Keeping this range active
  after recovery prevents a command jump when recovery exits. The
  PICK-start-relative envelope remains active for link1 and all wrist joints.
- The existing soft-limit slowdown remains active near every absolute bound.
- The state machine still requires both position and wrist alignment before
  entering `PREGRASP`; no tolerance or descent safety gate is bypassed.

## Safety and Failure Behavior

- No command may exceed `joint_lower_limits` or `joint_upper_limits`.
- Recovery does not grant extra range to link1 or link4 through link6.
- The wrist recovery speed is lower than the normal `WRIST` speed.
- If the pose is physically unreachable inside the hard bounds, the controller
  remains above the object and does not descend.
- Target snapshot, perception, grasp generation and place behavior are outside
  this change.

## Diagnostics

Existing logs continue to expose `approach_stage`, `position_recovery`,
position errors, wrist errors, orientation error and commanded joint velocity.
The expected successful trace is `POSITION -> WRIST`, optionally one or more
bounded recovery intervals, then `WRIST -> PREGRASP -> DESCEND`.

## Tests

- Recovery preserves a non-zero but capped wrist velocity.
- Recovery lets link2 and link3 use hard safety bounds.
- Recovery keeps PICK-start-relative bounds for link1 and link4 through link6.
- Normal WRIST limits and hard-bound integration behavior remain unchanged.
- The full `ee_switch_debug` test suite and package build pass.
