# First-Post-PICK Grasp Snapshot Design

## Goal

Treat each `PICK` command as a new perception boundary. The arm must use
exactly the first valid `/sm_grasping/grasp_best` message received after that
command and must ignore every later grasp update until the pick sequence ends
or is reset.

This prevents the approaching gripper from occluding the object, changing the
ROI point or orientation, and moving the active grasp target while the arm is
already following it.

## Considered Approaches

1. Freeze the most recently cached grasp when `PICK` arrives. This responds
   immediately, but it can reuse a stale sample from a previous task or a
   pre-command camera state.
2. Freeze when the controller enters `ARM_TRACK`. This is the existing
   behavior, but the state transition occurs after the command and does not
   define which post-command perception sample owns the task.
3. Discard the previous task's cached target and freeze the first valid grasp
   received after `PICK`. This is selected because it gives every task one
   explicit, fresh and immutable perception snapshot.

## State Model

The TF bridge will use three task-level states:

- `LIVE`: perception may update the preview target before a pick task.
- `WAITING_FIRST_PICK_GRASP`: entered immediately on `PICK`. The previous
  transform is invalidated and is not broadcast as a valid target for the new
  task.
- `PICK_SNAPSHOT_LOCKED`: entered by the first valid post-command
  `grasp_best`. Both position and orientation are stored, continuously
  broadcast with fresh TF timestamps, and never overwritten by later
  detections.

`RESET`, `PICK:HOLD`, or `PICK:DONE` releases the snapshot and returns the
bridge to `LIVE`.

## Data Flow

1. The bridge receives `PICK`.
2. It invalidates `latest_transform`, clears any prior latch, records that it
   is waiting for a new sample, and stops publishing the previous task's TF.
3. The arm controller records the command time and rejects TF data older than
   that time using its existing `target_accept_after_ns` check.
4. The first valid `grasp_best` received after the command is transformed into
   `chassis_link` and stored as the task snapshot.
5. The bridge freezes the entire transform: translation and quaternion.
6. Every subsequent `grasp_best` is ignored even if the object disappears,
   reappears, is partially masked by the hand, or produces a different ROI.
7. The arm proceeds only after the locked transform is broadcast with a
   post-command timestamp.

## Failure and Safety Behavior

- If no valid post-command grasp arrives, the bridge remains in
  `WAITING_FIRST_PICK_GRASP`; it must not fall back to the old cached target.
- While waiting, the arm receives no acceptable fresh target and follows its
  existing safe no-target behavior.
- An empty frame, failed TF conversion, or invalid grasp message does not
  consume the one allowed snapshot.
- `ARM_TRACK` state messages cannot replace, release, or recapture an active
  PICK snapshot.
- A repeated `PICK` command always starts a new boundary and discards the
  previous snapshot.

## Diagnostics

Logs must distinguish:

- previous target invalidated on `PICK`;
- waiting for the first post-command grasp;
- snapshot locked, including position and quaternion;
- later grasp update ignored because the snapshot is locked;
- snapshot released and the reason for release.

## Tests

- `PICK` invalidates a pre-existing cached transform.
- The first valid grasp after `PICK` becomes the snapshot.
- The second and later grasp messages cannot change either position or
  orientation.
- Invalid first messages do not consume the snapshot.
- No transform is broadcast while waiting for the first valid grasp.
- `ARM_TRACK` cannot overwrite or release the task snapshot.
- `RESET`, `PICK:HOLD`, and `PICK:DONE` release the snapshot.
- A second `PICK` requires a new first post-command grasp.
- Existing launch, bridge, controller freshness, wrist and pick/place tests
  continue to pass.
