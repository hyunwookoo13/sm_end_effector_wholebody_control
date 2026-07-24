# Long-Range Whole-Body Top-Down Pick Design

## Goal

Add an opt-in long-range pick-and-place mode that keeps the existing
obstacle-aware Nav2 approach, hybrid command blending, and continuous
base-to-arm whole-body handoff, but replaces only the terminal pick motion with
the validated precomputed top-down grasp sequence.

The system continues to control one end-effector objective with both the mobile
base and the six arm joints. As the robot approaches the object, the existing
switching functions reduce the base contribution while increasing the arm
contribution. The arm must start moving toward the object while the base is
still decelerating; the design must not introduce a stop-then-start handoff.

After the pick reaches `PICK:HOLD`, the existing task manager remains solely
responsible for choosing the next action:

- A nearby place target uses the existing direct-place path without returning
  home or performing a safe retreat.
- A distant place target uses the existing `TRANSPORT`, `SAFE_RETREAT`,
  Nav2, and place sequence.

## Non-Goals

- Replacing Nav2, its costmaps, or obstacle avoidance.
- Changing the existing hybrid handoff thresholds or command blending.
- Changing the existing base/arm switching equations.
- Changing place motion planning or the direct-place distance policy.
- Adding MoveIt or global arm collision planning in this iteration.
- Making the new mode the default for an existing launch.

Global arm collision planning can be added later above the same execution
interface. In that future design, the existing joint groups remain useful as
constraints, initial guesses, and local execution roles.

## Invariants

The following behavior is preserved:

1. `NAVIGATE_PICK` uses Nav2 for long-range obstacle-aware travel.
2. The existing outer threshold starts hybrid blending between navigation and
   manipulation commands.
3. The existing inner threshold transfers command ownership to manipulation.
4. One end-effector target drives both base and arm behavior.
5. The existing switching functions smoothly reduce base motion and increase
   arm motion as the target enters the arm workspace.
6. `PICK:HOLD` remains the task-manager contract for a completed pick.
7. All behavior after `PICK:HOLD` remains unchanged.
8. Existing launch files retain their current default behavior.

## Selected Approach

Extend the existing arm position controller with a feature-gated pick execution
mode:

```text
pick_execution_mode: legacy | rolling_top_down
```

`legacy` remains the default. Only a new
`yoloe_long_range_top_down_pick_place.launch.py` selects
`rolling_top_down`.

This approach was selected over a second arm controller because it avoids
arbitrating two publishers on `/joint_position_command`. It was selected over
adding orchestration states to `pick_place_task_manager` because the task
manager, navigation flow, direct-place policy, and retreat behavior are already
working and do not need to understand how the arm performs a pick.

## Architecture

### Existing whole-body approach

The task manager and navigation command mux continue their current flow:

```text
FIND_PICK
  -> NAVIGATE_PICK
  -> HYBRID_PICK
  -> PICK
```

During the transition, the existing controller continues to calculate the base
command from the end-effector target. The existing base switching factor and arm
switching factor remain unchanged:

```text
far target:
    base contribution high
    arm near home

transition region:
    base contribution decreases
    arm contribution increases

arm workspace:
    base contribution approaches zero
    arm approaches the top-down pre-grasp
```

The new mode must not publish a hard stop merely to begin arm motion. Base
deceleration and arm approach overlap.

### Rolling top-down preparation

YOLOE and grasp inference continue running during navigation and whole-body
approach. The new pick mode consumes the rolling grasp candidates independently
of the task manager's navigation target cache.

Each candidate is transformed into the arm planning frame using the current TF
state. A background planning worker computes the fixed-orientation top-down
pre-grasp and grasp joint targets from:

- the newest acceptable grasp candidate;
- the current measured six-joint state;
- the existing joint limits and top-down constraints.

Each plan is tagged with the grasp timestamp, the transform used for planning,
the planning completion time, and the measured joint state used as its seed.
Only one planning request may be active. If a newer candidate arrives while a
request is active, it replaces the pending request rather than creating an
unbounded queue.

The ROS control timer never performs the expensive IK search. It continues
publishing base and arm commands at its configured rate while the worker plans.

### Coordinated pre-grasp approach

The newest valid rolling plan supplies a six-joint pre-grasp target. The
existing arm switching weight blends motion toward that target with the current
home/whole-body behavior. Joint roles remain explicit:

- `joint1` supplies target yaw;
- `joint2` and `joint3` supply radial reach and height;
- `joint4`, `joint5`, and `joint6` supply the grasp orientation.

The groups execute as one coordinated six-joint motion. The new mode does not
replace the switching weight with a separate arm start event.

Because the base is still moving, a rolling plan is advisory until the terminal
gate. A plan that no longer matches the current target or joint state is
discarded and replaced asynchronously.

### Terminal latch and contact motion

The controller latches the newest valid plan only when all of these conditions
hold:

- the existing controller reports arm ownership of the target;
- the existing base contribution has fallen below a small configurable
  threshold;
- the target remains inside the existing arm safety envelope;
- the final base command is below configurable linear and angular limits for a
  small number of consecutive control cycles;
- the candidate and plan pass freshness and drift validation.

The gate does not start the arm; the arm is already approaching pre-grasp. It
only freezes the final contact path.

After the latch, the validated semantic sequence runs:

```text
APPROACH completion
  -> DESCEND
  -> GRASP
  -> LIFT
  -> HOLD
```

`DESCEND` is prohibited until the terminal gate is satisfied. This preserves
continuous whole-body approach while preventing contact during meaningful base
motion.

The fixed reachable orientation is shared by pre-grasp, descend, and lift.
All six joints cooperate so that `joint2`/`joint3` translation is compensated by
`joint4`/`joint5`/`joint6` orientation motion.

The long-range top-down launch sets `return_home_after_pick` to false. The
controller publishes `PICK:HOLD` after lift and waits for the task manager.

## Perception Freshness Without a Fixed Dwell

The new mode does not add a fixed `SNAPSHOT_WAIT` delay after base motion. It
uses a rolling, bounded candidate stream:

1. Perception and grasp inference run throughout the final approach.
2. Candidate timestamps establish when the underlying observation was
   produced.
3. The controller rejects candidates older than a configurable maximum age.
4. The controller rejects a prepared plan when target translation,
   orientation, or seed-joint drift exceeds configured tolerances.
5. The terminal gate latches the newest valid plan immediately.

Where the current grasp output does not preserve the originating ROI timestamp,
the new opt-in perception path must preserve that provenance. It must not change
the timestamps or topics used by existing launches.

This removes the unconditional 0.4-second settle from the new long-range mode
without accepting a pre-navigation grasp.

## Performance

The existing fixed-orientation planner was measured on the development machine
with the recorded reachable test geometry:

```text
10 successful plans
minimum: 198.96 ms
mean:    205.05 ms
maximum: 220.24 ms
```

That work must run outside the 40 Hz control callback. Planning overlaps the
final whole-body approach. When a valid rolling plan is ready at the terminal
gate, there is no additional planning wait after the base contribution reaches
zero. If no valid plan is ready, the controller remains in non-contact approach
until a valid plan arrives or the terminal timeout expires.

The implementation records:

- candidate age;
- plan computation duration;
- target and seed drift;
- time from hybrid entry to first valid rolling plan;
- time from terminal-gate readiness to plan latch;
- time from plan latch to descent.

There is no intentional dwell between terminal readiness and latching an
already valid plan.

## Failure Handling

The new mode fails closed:

- No fresh candidate: continue the existing non-contact whole-body approach
  until a configurable timeout.
- Planner still running: continue publishing normal base/arm approach commands;
  never block the control timer.
- Unreachable target: discard the failed rolling plan and allow a newer
  candidate to trigger another attempt.
- Candidate or plan becomes stale: discard it and keep the gripper open.
- Terminal timeout without a valid plan: reduce the base through the existing
  switching behavior, hold the arm outside contact, publish
  `PICK:PLAN_FAILED`, and do not fall back silently to legacy descent.
- TF loss: hold the last safe non-contact command, command no descent, and
  report the missing transform.

Once `DESCEND` starts, the target and plan remain immutable, matching the
validated top-down test behavior.

## Launch and Configuration Isolation

The new launch composes the same long-range perception, Nav2, task manager,
command mux, and controller nodes as the existing long-range launch. It only
adds or overrides settings required for rolling top-down pick.

The existing launch keeps:

```text
pick_execution_mode=legacy
```

The new launch uses:

```text
pick_execution_mode=rolling_top_down
return_home_after_pick=false
```

Top-down readiness, freshness, and drift thresholds are explicit launch
arguments or controller parameters. They are not applied to legacy pick or
place behavior.

## Testing

### Pure unit tests

- Freshness validation accepts a recent candidate and rejects stale input.
- Drift validation rejects plans whose target or seed has moved too far.
- The rolling planner keeps one active request and only the newest pending
  request.
- The terminal gate requires arm ownership, a low base contribution, a safe
  target, a low base command, and a valid plan.
- `DESCEND` cannot start while any terminal condition is false.
- A valid latch preserves one immutable plan through descend and lift.

### Controller tests

- In the switching region, base and arm commands are nonzero concurrently.
- As distance closes, base contribution decreases while arm contribution
  increases.
- Rolling plan computation does not block periodic command publication.
- The arm moves toward pre-grasp before base motion reaches zero.
- A new candidate invalidates a materially drifted rolling plan.
- Top-down pick ends in `PICK:HOLD` without returning home.
- A terminal timeout produces `PICK:PLAN_FAILED` and no descent.
- `PLACE` continues through the legacy controller path.

### Launch tests

- The existing long-range launch preserves its current nodes, topics, and
  default parameters.
- The new launch enables only the rolling top-down pick path.
- The new launch retains the existing Nav2, hybrid mux, task manager, and dual
  perception nodes.
- Only one node publishes `/joint_position_command`.

### Regression tests

- All existing navigation, mux, task-manager, wrist, kinematics, and launch
  tests pass unchanged.
- The existing `yoloe_top_down_grasp_test.launch.py` retains its current fixed
  base behavior.
- The existing direct-place threshold still skips transport and safe retreat.
- A distant place target still uses `TRANSPORT`, `SAFE_RETREAT`, navigation,
  and the existing place sequence.

### Isaac Sim acceptance

An acceptance recording must demonstrate:

1. Nav2 avoids an obstacle while approaching the pick target.
2. Hybrid command blending begins at the existing threshold.
3. The base visibly decelerates while the arm simultaneously moves toward
   top-down pre-grasp.
4. The base is below the contact-motion threshold before descent starts.
5. Pick completes through grasp and lift.
6. A nearby place target proceeds without home return or safe retreat.
7. A distant place target retains the existing transport and navigation flow.

## Future Collision Planning

A future collision-aware arm planner can replace the rolling top-down IK worker
without changing the task-manager contract or whole-body switching interface.
It should plan all six arm joints together while preserving the current joint
group roles as constraints and execution structure.
