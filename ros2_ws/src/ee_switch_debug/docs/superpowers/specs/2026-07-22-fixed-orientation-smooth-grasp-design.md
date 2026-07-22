# Fixed-Orientation Smooth Current-Distance Grasp Design

## Goal

At the current fixed mobile-base pose, grasp the detected object with a single
immutable plan that looks intentional: approach a reachable pre-grasp, descend
without changing wrist orientation, close, lift vertically with the same
orientation, and return home without a speed discontinuity.

## Evidence and Constraint

The 183.1 second recording shows four undesirable phases:

- joint-space approach curves up to 0.084 m away from a direct EE path;
- the 0.10 m descent rotates the wrist by about 120 degrees;
- lift reverses that rotation while the object remains close to the table;
- the final home controller changes from 0.16-0.28 rad/s path limits to a
  separate 1.0 rad/s home limit.

The complete requested top-down orientation is not reachable 0.10 m above the
live target at the current base pose. A 120-seed IK search produced no valid
solution. Therefore the design must not promise exact top-down orientation at
that pre-grasp. The planner instead finds the closest orientation to top-down
that is reachable at both the pre-grasp and final grasp positions. For the
recorded live geometry, approximately 80 percent of the current-to-top-down
rotation is feasible, leaving about 24 degrees of tilt.

## Considered Approaches

### Move the mobile base closer

This can make the exact top-down pre-grasp reachable, but the required inward
change is approximately 0.20 m and conflicts with the table/chassis collision
constraint. It is rejected for this fixed-base test.

### Fixed reachable orientation with semantic smooth segments

Search for the most top-down common orientation, keep it fixed during descent
and lift, and execute only semantic segment endpoints with quintic time scaling.
This directly removes the visible wrist sweep, reverse sweep, waypoint pauses,
and final speed jump. This is the selected approach.

### MoveIt or a general trajectory optimizer

This is appropriate for later collision-aware multi-object manipulation, but it
adds planning-scene and execution integration beyond the current demonstration.
It is deferred.

## Planner Architecture

The planner receives the immutable post-`PICK` target snapshot, current six
joint positions, joint limits, 0.10 m clearance, and the requested grasp
orientation.

1. Apply the existing 0.025 m inward radial perception compensation once.
2. Interpolate orientation from the current EE rotation toward the requested
   grasp rotation.
3. Search from the requested orientation backward for the greatest interpolation
   fraction whose single fixed rotation has valid IK at both:
   - compensated grasp position plus 0.10 m world-Z clearance;
   - compensated final grasp position.
4. Reject the plan if the best fraction is below 0.50 or either endpoint exceeds
   hard joint limits.
5. Validate a dense FK sample of the joint interpolation between pre-grasp and
   grasp:
   - Z must decrease monotonically;
   - XY deviation from the grasp axis must remain at most 0.015 m;
   - orientation deviation from the selected fixed rotation must remain at most
     0.035 rad.

The resulting plan stores endpoint joint vectors and the selected orientation
fraction. Perception and target TF are never consulted again for that `PICK`.

## Execution Architecture

The controller executes four semantic motion segments:

```text
APPROACH -> DESCEND -> GRASP -> LIFT -> HOME -> HOLD
```

- `APPROACH`: current joints to fixed-orientation pre-grasp.
- `DESCEND`: pre-grasp to grasp with the same EE orientation.
- `GRASP`: hold the endpoint and close the gripper.
- `LIFT`: grasp to pre-grasp, exactly preserving the selected orientation.
- `HOME`: pre-grasp to configured home through the same trajectory generator.

Each motion segment uses quintic smoothstep position interpolation:

```text
s(u) = 10u^3 - 15u^4 + 6u^5,  0 <= u <= 1
q(u) = q_start + s(u) * (q_goal - q_start)
```

Segment duration is selected from the configured per-joint velocity limits and
acceleration limit. The trajectory has zero velocity and acceleration at each
semantic endpoint. There are no intermediate settle-and-zero events. The final
measured joint tolerance remains 0.025 rad; if simulation tracking lags after
the planned duration, the controller holds the endpoint until measured joints
arrive rather than advancing early.

## Safety and Failure Behavior

- Mobile-base velocity remains zero for the complete sequence.
- The target snapshot remains immutable after planning starts.
- Hard joint limits and the configured PICK excursion envelope remain active.
- A plan with no common fixed orientation above the 0.50 fraction, excessive
  descent XY bow, non-monotonic descent Z, or excessive orientation deviation
  enters `PICK:PLAN_FAILED` and holds the open gripper.
- `RESET` clears the fixed plan, trajectory timing state, and all endpoints.
- The controller never falls back to reactive target tracking during this test.

## Parameters

The dedicated top-down launch enables:

- `top_down_use_fixed_reachable_orientation: true`
- `top_down_minimum_orientation_fraction: 0.50`
- `top_down_orientation_search_steps: 20`
- `top_down_descend_max_xy_deviation: 0.015`
- `top_down_descend_max_orientation_deviation: 0.035`
- semantic segment velocity limits based on the existing alignment and path
  arrays, with HOME using the alignment array instead of `home_max_joint_velocity`.

The generic mission configuration keeps this behavior disabled by default.

## Verification

Automated tests must prove:

- the live target selects a common fixed rotation reachable at both endpoints;
- descent FK samples are monotonic in Z and stay within the XY/orientation
  bounds;
- descent and lift use the same endpoint orientation;
- quintic interpolation has exact endpoints and zero endpoint velocity;
- intermediate waypoint completion no longer resets velocity because there are
  no intermediate execution stops;
- HOME uses the same semantic segment executor;
- failed common-orientation search holds `PICK:PLAN_FAILED`;
- all existing package tests and `colcon build --packages-select
  ee_switch_debug --symlink-install` pass.

One simulator run then confirms the state sequence, fixed base, absence of wrist
rotation during descent/lift, successful grasp, and smooth HOME. Visual success
requires review of a new recording; logs alone are insufficient to certify
natural motion.
