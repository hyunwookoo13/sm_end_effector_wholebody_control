# sm_end_effector_wholebody_control

ROS 2 packages for natural-language object selection, perception, grasp pose
estimation, Nav2 travel, and EE-targeted mobile-manipulator control in Isaac Sim.

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
- `sm_task_orchestrator`: Pick/Place phases, navigation handoff, safe retreat,
  and task-state coordination.

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-up-to sm_bringup
source install/setup.bash
```

Model files and Python virtual environments are not tracked.

## Pick And Place

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
