# Precomputed Top-Down Sequence Design

## Goal

Make one `PICK` command execute a stable top-down grasp without the arm
continuously correcting a moving Cartesian error while the wrist is rotating.

## Root Cause

The current controller treats joints 2 and 3 as a two-link arm.  In the actual
OMY-F3M chain, joints 4 through 6 and the 24.4 cm tool offset also change the
end-effector position.  The controller therefore moves joints 2 and 3 toward a
position, rotates the wrist, observes the resulting position change, and moves
joints 2 and 3 again.  Near the straight-elbow configuration the two-link
Jacobian becomes singular, so this feedback loop can stall or twist the arm.

## Selected Architecture

Use the complete six-joint kinematic chain as a planner and retain grouped
joint control as the executor.

1. On the first fresh `PICK` target, copy the target position and orientation.
2. Solve the complete six-joint pose at 10 cm above the grasp target once.
3. Generate a vertical Cartesian descent as fixed IK waypoints from the
   pre-grasp pose to the grasp pose.
4. Execute fixed targets in the order `joint1`, `joint2+joint3`, then
   `joint4+joint5+joint6`.
5. Execute the precomputed descent waypoints, close the gripper, replay them in
   reverse to lift, and return home.

The controller must never recompute the plan because the marker or mask changes
during the active sequence.  An invalid or unreachable IK plan stops the arm;
it does not fall back to the old reactive wrist-compensation behavior.

## Kinematic Model

The model is taken from the robot USD used by the running Isaac simulation.
Joint axes are `Z, Y, Y, Y, Z, Y`.  Parent-to-joint translations in metres are:

```text
joint1 (0, 0, 0.1715)
joint2 (0, -0.1215, 0)
joint3 (0, 0, 0.2470)
joint4 (0, 0.1215, 0.2195)
joint5 (0, -0.1130, 0)
joint6 (0, 0, 0.1155)
tool   (0, -0.244030948, 0)
```

Forward kinematics from this model was checked against the live
`link0 -> end_effector_link` TF at the home pose and matches its position and
rotation.

## IK and Safety

- Use damped least-squares numerical IK with a finite-difference geometric
  Jacobian.
- Solve within configured hard joint limits and the configured per-pick
  excursion envelope.
- Reject solutions outside position or orientation tolerance.
- Build the descent at bounded Cartesian spacing and seed every waypoint with
  the previous solution to preserve continuity.
- Stage completion is based on measured joint error, not target TF error.
- Apply stage velocity and acceleration limits while moving to fixed targets.

## GPD Extension

The planner accepts an arbitrary grasp position and rotation.  A future GPD
result therefore uses the same pipeline: translate the grasp pose backward by
the configured approach distance, solve the fixed pre-grasp and approach
waypoints, then execute them with the same grouped controller.

## Verification

- Forward kinematics reproduces a known live home-pose TF.
- IK reconstructs poses produced by forward kinematics.
- Pre-grasp is 10 cm above the grasp and descent waypoints keep orientation
  fixed while decreasing only world Z.
- The sequence state machine changes only the intended joint group in each
  alignment stage and never replans after latching.
- Existing package tests and a package build pass before simulator testing.
