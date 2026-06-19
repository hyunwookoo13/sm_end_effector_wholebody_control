# sm_end_effector_wholebody_control

ROS 2 packages for Florence-2 object detection, grasp pose estimation, and
mobile-manipulator whole-body position control in Isaac Sim.

## Packages

- `ee_switch_debug`: mobile-base/arm switching, grasp target TF bridging, and
  arm position control.
- `sm_florence_2_vlm_ros2`: Florence-2 RGB-D object detection and ROI point
  cloud generation.
- `sm_grasping_ros2`: grasp candidate generation, best-pose selection, and
  RViz gripper markers.

## Build

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Model files and Python virtual environments are not tracked.
