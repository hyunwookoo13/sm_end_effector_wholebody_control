# Near Pick Direct Approach Design

## Goal

After a completed pick-and-place task, start a nearby next pick with the
existing precision controller instead of backing away and driving forward
again.

## Design

Add a pick-specific direct-approach distance to the existing navigation-skip
decision. A pick target within `1.20 m` of the base skips Nav2, which also
clears the pending consecutive-task retreat without commanding motion. A pick
outside that range keeps the current rear-LiDAR-safe `0.40 m` retreat before
Nav2.

The new threshold is independent from the existing place threshold so each can
be tuned without changing the other. Arm control, perception, Nav2 behavior,
and the retreat implementation remain unchanged.

## Verification

- A pick at `1.10 m` skips navigation and retreat.
- A pick beyond `1.20 m` still retreats before submitting a Nav2 goal.
- Place direct approach keeps its current behavior.
- Package tests and the affected ROS 2 packages build successfully.
