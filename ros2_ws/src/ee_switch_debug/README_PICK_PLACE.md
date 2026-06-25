# Pick and Place Launch Notes

## Near Pick and Place

One workspace/table pick-and-place. This is the stable structure checkpointed on
2026-06-25.

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug florence_pick_place_control.launch.py \
  publish_fixed_camera_tf:=false \
  pick_object:=can \
  place_object:=box \
  enable_base_motion:=true \
  target_parent_frame:=odom
```

Source checkpoint archive:

```text
/home/kiro/Desktop/hw_ws/backup/near_pick_place_checkpoint_20260625_1443.tar.gz
```

## Long Range Pick and Place

Pick with the first RSD455 camera and grasping pipeline, then detect the far box
with the second RSD455 camera and move the mobile base while holding the object.

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py \
  publish_fixed_camera_tf:=false \
  pick_object:=can \
  place_object:=box \
  target_parent_frame:=odom
```

The long-range launch uses these camera topics for place detection:

```text
/rsd455/rgb2
/rsd455/depth2
/rsd455/camera_info2
```

The default second camera optical frame is:

```text
rsd455_color_optical_frame2
```

If the second camera TF frame name differs, override it:

```bash
place_camera_frame:=<actual_second_camera_optical_frame>
```

Place tuning:

```text
place_descend_depth:=0.02..0.04
place_open_duration:=0.8
```
