# Semantic Mobile Manipulation Orchestration

Modular ROS 2 system for natural-language object selection, Semantic DB lookup,
Nav2 travel, RGB-D grasp refinement, and whole-body Pick & Place in Isaac Sim.
The current design keeps the previously validated fast Pick & Place pipeline
intact and adds replaceable orchestration modules in front of it.

## Current MVP

```mermaid
flowchart LR
    A[Natural-language command] --> B[Task parser]
    B --> C[Static Semantic DB]
    C --> D[Nav2 approach pose]
    D --> E[Fresh RGB-D perception]
    E --> F[Existing Whole-body Pick & Place]
```

The MVP currently supports the following end-to-end flow:

1. Parse a Korean or English Pick & Place command.
2. Resolve the requested objects and their stored `map`-frame approach poses.
3. Drive to the Pick workspace with Nav2.
4. Use current RGB-D perception and the existing whole-body controller to pick.
5. Drive to the Place workspace when it is external to the Pick workspace.
6. Refresh perception and complete the existing Place behavior.

The Semantic DB is intentionally read-only during the current Isaac Sim MVP.
Base commands remain arbitrated by `sm_base_control_manager`, so Nav2 and the
precision manipulation controller do not publish to the robot simultaneously.

## Packages

- `sm_bringup`: full-system launch composition and operator entry points.
- `sm_base_control_manager`: base-command arbitration and final `/cmd_vel`
  publication.
- `sm_ee_wholebody_control`: EE-targeted arm/base switching, TF bridges, and
  whole-body controllers.
- `sm_florence_2_vlm_ros2`: Florence-2 RGB-D object detection and ROI point
  cloud generation.
- `sm_grasping_ros2`: grasp candidate generation, best-pose selection, and
  RViz gripper markers.
- `sm_natural_language_task`: natural-language task parsing and interactive
  task input.
- `sm_navigation_nav2`: Nav2 launch and configuration ownership.
- `sm_semantic_map_interfaces`: service contract for semantic object lookup.
- `sm_semantic_map`: static object database, lookup server, map markers, and
  position collection/calibration tools.
- `sm_semantic_mvp`: end-to-end Semantic DB, Nav2, and existing Pick & Place
  mission orchestration.
- `sm_task_orchestrator`: Pick/Place phases, navigation handoff, safe retreat,
  and task-state coordination.

Research requirements, architecture, experiment, and evidence templates are
maintained in [`ros2_ws/research-os`](ros2_ws/research-os/README.md).

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-up-to sm_bringup
source install/setup.bash
```

Model files and Python virtual environments are not tracked.

## Semantic DB + Existing Pick & Place

Start Isaac Sim and press **Play**, then launch the integrated MVP:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity></Tracing></Domain></CycloneDDS>'
ros2 launch sm_semantic_mvp semantic_db_existing_pick_place.launch.py
```

Send a task from another sourced terminal:

```bash
ros2 topic pub --once /natural_language_task std_msgs/msg/String \
  "{data: '빨간 캔을 분홍 박스에 넣어줘'}"
```

Monitor the orchestration state with:

```bash
ros2 topic echo /semantic_mvp/status
ros2 topic echo /semantic_navigation/status
ros2 topic echo /pick_place_task_state
```

See the [Semantic MVP runbook](ros2_ws/src/sm_semantic_mvp/README.md) and
[Semantic map documentation](ros2_ws/src/sm_semantic_map/README.md) for the
detailed sequence and DB inspection commands.

## Existing Pick And Place

Build the full-system launch and all of its package dependencies after
changing launch/config/Python files:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-up-to sm_bringup
source install/setup.bash
```

Run the Florence grasp + pick/place task manager:

```bash
ros2 launch sm_bringup florence_pick_place_control.launch.py \
  publish_fixed_camera_tf:=false \
  pick_object:=can \
  place_object:=box \
  enable_base_motion:=true \
  target_parent_frame:=odom
```

Use `place_object:=dish` when placing into the dish. Keep
`target_parent_frame:=odom` when `enable_base_motion:=true`; this freezes the
place target in the world/odom frame while the base moves.

Useful tuning arguments:

```bash
grasp_offset_z:=0.03          # pre-grasp height above the grasp target
grasp_descend_depth:=0.07     # downward pick motion before closing gripper
place_offset_z:=0.10          # pre-place height above the place target
place_descend_depth:=0.01     # downward place motion before releasing
place_offset_x:=0.0           # place target x correction
place_offset_y:=0.0           # place target y correction
return_home_after_place:=true # skip vertical retreat and go to safety pose
```

To start a task manually instead of autostarting from launch:

```bash
ros2 topic pub --once /pick_place_task std_msgs/msg/String "{data: 'can,box'}"
```
