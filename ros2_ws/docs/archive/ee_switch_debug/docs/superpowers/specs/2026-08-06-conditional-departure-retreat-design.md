# Conditional Departure Retreat Design

## Context

In consecutive successful tasks, the robot finishes `PLACE` close to the place
table and enters `DONE`. A new natural-language task immediately resets the task
manager to `FIND_PICK`. When the next pick is remote, `request_navigation`
sends a Nav2 goal whose final yaw faces the object. Nav2 may therefore start by
rotating in place while the robot footprint is still too close to the table.

The existing `SAFE_RETREAT` does not cover this transition. It runs only after
`PICK` when a remote place target requires travel, then always continues to
`FIND_PLACE`. There is no retreat after `PLACE -> DONE` or before the next
task's `FIND_PICK -> NAVIGATE_PICK` transition.

## Goal

Before Nav2 starts a remote pick in a consecutive task, move the robot backward
using the existing rear-LiDAR-protected retreat. Preserve immediate operation
when the next pick is close enough that Nav2 is skipped.

## Non-Goals

- Do not change Nav2 planners, controllers, costmaps, speed limits, or goal
  geometry.
- Do not change YOLOE, grasping, natural-language parsing, precision control,
  base/arm switching, or manipulation trajectories.
- Do not retreat before the first task.
- Do not force a retreat for a close pick that uses direct precision control.
- Do not retreat when the rear scan reports insufficient clearance.

## Selected Design

Record whether a new task was started from the previous task's `DONE` phase.
This sets a one-shot `departure_retreat_pending` flag only when navigation is
enabled.

The existing navigation decision remains authoritative. In `request_navigation`
for a pick:

1. Compute the current standoff pose and existing `should_skip_navigation`
   result.
2. If navigation is skipped, clear the pending flag and continue directly; no
   retreat occurs.
3. If Nav2 is required and the pending flag is set, clear it, command the arm to
   `TRANSPORT`, stop the base, and enter `SAFE_RETREAT` instead of sending the
   Nav2 goal.
4. After retreat completion, return to `FIND_PICK`; the next state-machine tick
   requests the same Nav2 pick normally.

`request_navigation` returns a distinct `retreating` result so `FIND_PICK` does
not fall through into manipulation while the retreat is active.

## Reusing SAFE_RETREAT

Extend `start_safe_retreat` with a continuation phase, defaulting to the current
`FIND_PLACE` behavior. The existing post-pick call therefore stays unchanged.
The new consecutive-task call supplies `FIND_PICK`.

The retreat keeps all existing safety and timing values:

- distance: `0.40 m`;
- speed: `0.25 m/s` backward;
- minimum rear clearance: `0.50 m`;
- rear scan topic: `/laser_scan_2`;
- transport settling: `1.0 s`;
- timeout: `5.0 s`.

The rear scan must be fresh. If the rear path is blocked, the retreat fails to
`ERROR` and Nav2 is not started.

## State Flow

Remote consecutive pick:

```text
PLACE -> DONE
new task -> FIND_PICK [departure_retreat_pending]
fresh target -> SAFE_RETREAT [continuation=FIND_PICK]
retreat complete -> FIND_PICK -> NAVIGATE_PICK
```

Close consecutive pick:

```text
PLACE -> DONE
new task -> FIND_PICK [departure_retreat_pending]
existing Nav2 skip decision -> clear pending -> PICK/direct precision control
```

First task:

```text
IDLE -> FIND_PICK -> existing behavior unchanged
```

## Failure and Interruption Behavior

- A stale or missing rear scan holds the base stopped.
- Rear clearance at or below `0.50 m` produces `RETREAT_FAILED` and `ERROR`.
- A new task that does not originate from `DONE` does not inherit the departure
  retreat flag.
- The one-shot flag is cleared before starting retreat so it cannot loop after
  returning to `FIND_PICK`.

## Verification

Automated tests must prove:

- a task started from `DONE` with navigation enabled marks departure retreat;
- a first task does not mark departure retreat;
- a close pick clears the flag and skips retreat;
- a remote pick returns `retreating`, commands `TRANSPORT`, and does not submit
  a Nav2 goal;
- retreat completion continues to `FIND_PICK` for departure retreat;
- the existing post-pick retreat still continues to `FIND_PLACE`;
- blocked rear clearance still prevents motion and ends in `ERROR`;
- all existing navigation, task-manager, parser, YOLOE, and grasp tests remain
  green.
