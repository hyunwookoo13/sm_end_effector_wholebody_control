# PICK Perception Bundle Snapshot Design

## Goal

Freeze the complete perception data used by a PICK task instead of freezing
only the final controller TF. After `PICK`, the first fresh ROI point cloud and
the first grasp result produced from that frozen ROI become the task snapshot.
The ROI/mask visualization, grasp marker, target TF and arm controller must all
use that same immutable snapshot until the task ends or is reset.

This design supersedes the TF-only scope in
`2026-07-16-first-post-pick-grasp-snapshot-design.md`.

## Root Cause

The current TF bridge freezes `grasp_position_target`, but the rest of the
pipeline remains connected to live topics:

- `sm_grasping_inference` consumes the live
  `/sm_florence_2_vlm/roi_pointcloud`;
- `sm_gripper_marker` consumes the live `/sm_grasping/grasp_best`;
- RViz displays the live ROI point cloud and gripper markers.

When the arm occludes the object, YOLOE produces a different mask and ROI,
`sm_grasping_ros2` generates another heuristic grasp, and the visible marker
moves or disappears even though the separate controller TF may already be
frozen.

## Considered Approaches

1. Freeze only the RViz marker. This hides the symptom but leaves grasp
   inference connected to changing perception data.
2. Add PICK-specific state directly to YOLOE and `sm_grasping_ros2`. This can
   work, but it couples two reusable perception packages to one mission state
   machine.
3. Insert a task-scoped snapshot relay between the reusable perception
   components. This is selected because the orchestration layer owns mission
   state while YOLOE and `sm_grasping_ros2` remain independently reusable.

## Topic Architecture

The test launch remaps live producer outputs and exposes the existing canonical
topics through a new `pick_perception_snapshot` relay:

```text
YOLOE
  /sm_florence_2_vlm/roi_pointcloud_live
                  |
                  v
pick_perception_snapshot
  /sm_florence_2_vlm/roi_pointcloud
                  |
                  v
sm_grasping_inference
  /sm_grasping/grasp_best_live
  /sm_grasping/grasp_openings_live
                  |
                  v
pick_perception_snapshot
  /sm_grasping/grasp_best
  /sm_grasping/grasp_openings
          |                    |
          v                    v
grasp_target_tf_bridge   sm_gripper_marker
          |
          v
arm controller
```

RViz continues using the canonical topic names, so no manual display
reconfiguration is required.

## State Model

- `LIVE`: raw ROI, grasp and opening messages pass through to canonical topics.
- `WAITING_ROI`: entered on `PICK`; all prior task snapshots are invalidated.
  The first fresh raw ROI point cloud is captured.
- `WAITING_GRASP`: the captured ROI is repeatedly published to
  `sm_grasping_ros2`. Raw grasp results are ignored for a short settle window
  so an inference already running before `PICK` cannot own the new task.
- `LOCKED`: the first valid post-settle grasp and its most recent opening are
  captured. The relay repeatedly publishes the same ROI, grasp and opening.
  All later live updates are ignored.

`RESET`, `PICK:HOLD`, and `PICK:DONE` release the bundle and return to `LIVE`.
A repeated `PICK` always invalidates the previous bundle and starts a new
`WAITING_ROI` boundary.

## Safety Behavior

- No pre-PICK ROI or grasp may become the task snapshot.
- The controller receives no acceptable fresh target until a locked grasp is
  available.
- Empty ROI clouds do not consume the snapshot.
- Invalid grasp messages with an empty frame are ignored.
- Loss, reappearance or deformation of the live mask after `LOCKED` cannot
  change the canonical ROI, grasp pose, opening, marker or target TF.
- The raw `_live` topics remain available for diagnostics.

## Tests

- `PICK` invalidates prior ROI, grasp and opening snapshots.
- The first non-empty raw ROI after `PICK` is locked.
- Raw grasp messages before ROI capture or during the settle window are
  ignored.
- The first valid post-settle grasp and opening are locked.
- Later ROI, grasp and opening messages cannot modify the bundle.
- The timer republishes byte-identical ROI data and numerically identical pose
  and opening values while locked.
- `RESET`, `PICK:HOLD`, `PICK:DONE`, and repeated `PICK` transition correctly.
- The launch remaps YOLOE and `sm_grasping_ros2` through the relay and routes
  the marker and TF bridge to canonical locked topics.
- Existing `ee_switch_debug`, `sm_grasping_ros2`, and YOLOE tests continue to
  pass.
