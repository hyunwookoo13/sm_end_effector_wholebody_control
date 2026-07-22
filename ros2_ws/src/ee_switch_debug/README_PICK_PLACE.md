# Pick and Place Launch Notes

## Fixed Top-Down Grasp Test

This launch uses YOLOE plus `sm_grasping_ros2`, freezes the first fresh grasp
after `PICK`, validates one complete six-axis endpoint plan, and executes:

```text
BLENDED_APPROACH(Joint 1 + Joint 2/3 + Joint 4/5/6 group paths)
    -> PREGRASP_VERIFY -> FIXED-ORIENTATION DESCEND -> GRASP
    -> FIXED-ORIENTATION LIFT -> CONTINUOUS HOME -> HOLD
```

The mobile base remains fixed. The planner searches for the closest-to-top-down
orientation that is reachable both 10 cm above the object and at the grasp
point. For the recorded current-distance geometry this is approximately 24
degrees from exact top-down. DESCEND and LIFT keep that one orientation instead
of rotating the wrist near the object.

The whole-arm IK solution proves that the final pregrasp and grasp poses are
reachable and that DESCEND is safe. Joint 1 direction, Joint 2/3 rho-Z, and
Joint 4/5/6 orientation paths are still generated with separate ownership, but
their trajectories play simultaneously on one shared clock. This is temporal
blending, not runtime six-axis IK or Jacobian control. Joint 2/3 compensation
starts early enough to keep pace with wrist rotation. 101 FK samples reject any
composed path that violates limits or drops below the configured safe height.

The approach and descent are both computed once
from the first post-`PICK` snapshot, so later mask or marker motion cannot steer
the arm. A 2.5 cm inward radial correction compensates the grasp generator's
outward offset. BLENDED_APPROACH, DESCEND, LIFT, and HOME use bounded
time-scaled trajectories with no intermediate waypoint stops. The approach
groups, LIFT, and HOME use compact cubic S-curves, while DESCEND keeps the
minimum-jerk quintic profile. The dedicated launch velocity limits,
1.6 rad/s^2 acceleration limit, and 0.6 s minimum duration are unchanged. The
dedicated test launch keeps the blended approach endpoint and DESCEND strict at
0.025 rad so PREGRASP_VERIFY cannot inherit a coarse wrist pose. LIFT
and HOME accept up to 0.050 rad measured-joint error to avoid multi-second
actuator-settling pauses. The complete blended approach reaches zero endpoint
velocity before DESCEND; a new recording is required to verify
the resulting duration and visual naturalness for each target pose.

After GRASP, LIFT remains on the validated vertical path until the measured EE
is 6 cm above the planned grasp pose. It then hands its bounded joint velocity
directly to the acceleration-limited HOME controller, so the arm curves toward
HOME without stopping at the handoff. This velocity preservation applies only
to LIFT-to-HOME; APPROACH still stops and verifies alignment before DESCEND.

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug yoloe_top_down_grasp_test.launch.py
```

In another terminal, select the detected class and issue one pick:

```bash
ros2 topic pub --once /sm_florence_2_vlm/target_objects \
  std_msgs/msg/String "{data: 'can'}"
ros2 topic pub --once /arm_task_command std_msgs/msg/String "{data: 'PICK'}"
```

Watch `/arm_task_state` for `PICK:BLENDED_APPROACH`, `PICK:PREGRASP_VERIFY`,
`PICK:DESCEND`, `PICK:GRASP`,
`PICK:LIFT`, `PICK:HOME`, and `PICK:HOLD`. `PICK:PLAN_FAILED` means no common
fixed orientation met the configured workspace, joint-envelope, or descent-path
limits. The arm deliberately holds instead of falling back to reactive position
correction.

The grasp marker and OMY EE use different local axes. The planner converts the
marker's `+X` approach and `+Y` closing axes to the OMY EE's `-Y` approach and
`+X` closing axes before solving IK.

## Nav2 Obstacle-Aware Long-Range Mode

The long-range launch can use both Isaac Sim lidars to build rolling costmaps in
the `odom` frame. Nav2 drives to a standoff pose first, then the existing
whole-body controller performs the final object-relative alignment.

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py \
  autostart:=false \
  enable_nav2:=true
```

The natural-language console is unchanged:

```bash
ros2 run ee_switch_debug natural_language_task_console
```

Useful RViz displays are `/global_costmap/costmap`,
`/local_costmap/costmap`, and `/plan`. The default pick and place base
standoffs are both `0.70 m`; adjust them with `pick_standoff_m` and
`place_standoff_m`. Set `enable_nav2:=false` to retain the previous direct
whole-body approach.

Nav2 publishes `/cmd_vel_navigation`, the manipulation controller publishes
`/cmd_vel_manipulation`, and `navigation_cmd_mux` is the only node that
publishes `/cmd_vel`.

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

Run YOLOE detection on both RSD455 cameras, generate grasp candidates on both ROI
streams, and let the task manager choose whichever camera currently sees the
requested pick/place target.

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

The long-range launch uses these camera topic groups:

```text
/rsd455/rgb
/rsd455/depth
/rsd455/camera_info

/rsd455/rgb2
/rsd455/depth2
/rsd455/camera_info2
```

The default camera optical frames are:

```text
rsd455_color_optical_frame
rsd455_color_optical_frame2
```

If the camera TF frame name differs, override it:

```bash
pick_camera_frame:=<actual_first_camera_optical_frame>
place_camera_frame:=<actual_second_camera_optical_frame>
```

Place tuning:

```text
place_descend_depth:=0.02..0.04
place_open_duration:=0.8
```

Grasp wrist orientation tuning:

```text
grasp_orientation_wrist_mode:=link6
grasp_link6_orientation_offset:=0.0
```

`link6` mode keeps the current wrist shape for joints 4 and 5, but rotates joint 6
from the `grasp_best` orientation instead of using the old fixed -90 degree value.
Use `off` to restore the fixed wrist behavior, or `full` to let the grasp pose drive
joints 4, 5, and 6 together after RViz validation.

## Natural Language Task Input

Install Ollama and pull the local parser model:

```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull gemma3:1b
```

Run the long-range launch in wait-for-command mode:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py autostart:=false
```

Option 1: send a natural-language command directly:

```bash
ros2 topic pub --once /natural_language_task std_msgs/msg/String "{data: '캔을 집어서 박스에 넣어줘'}"
```

Option 2: run the interactive input console in another terminal:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ee_switch_debug natural_language_task_console
```

Then type a command and press Enter:

```text
캔을 집어서 박스에 넣어줘
```

Color-qualified targets are supported through YOLOE plus mask color validation:

```text
파란 캔을 집어서 박스에 넣어줘
빨간 캔을 집어서 박스에 넣어줘
파란 캔을 노란 박스에 넣어줘
초록 컵을 분홍 박스에 넣어줘
blue can | red can | yellow box | pink box | green cup
```

Color-qualified targets are resolved as semantic class plus ROI color. For example,
`파란 캔을 노란 박스에 넣어줘` becomes `pick=blue can` and `place=yellow box`;
YOLOE proposes text-prompted masks, but the final bbox/ROI is kept only when the
semantic class, mask geometry, and requested RGB color match.

The long-range launch runs grasping on both camera ROI streams. Pick is no longer
fixed to the first camera only: `/sm_grasping/grasp_best` and
`/sm_grasping_place/grasp_best` are both valid pick candidates, and place detection
can come from either `/sm_florence_2_vlm/detections` or
`/sm_florence_2_vlm_place/detections`.

During a task, the manager publishes both targets to YOLOE immediately:

```json
{"target_objects": ["blue can", "yellow box"], "roi_target_objects": ["blue can"]}
```

This lets both cameras pre-cache the place target while the arm is still picking,
but keeps the grasp ROI point cloud limited to the pick object. As soon as the arm
reports `PICK:HOLD`, the cached place target can be used without waiting for a new
place-only detection cycle.

To avoid reusing a grasp from the previous command, the task manager ignores
grasp results for a short window after each new task starts. The default is:

```text
fresh_grasp_delay_sec:=0.35
```

The grasping node also consumes each ROI point cloud once, so an old pick ROI is
not repeatedly re-published as a fresh `grasp_best` after the next command.

The parser publishes validated JSON to `/pick_place_task`, for example:

```json
{"pick": "can", "place": "box"}
```
