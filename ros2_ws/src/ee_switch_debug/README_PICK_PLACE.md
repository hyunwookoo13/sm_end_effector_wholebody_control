# Pick and Place Launch Notes

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
ros2 run sm_natural_language_task natural_language_task_console
```

Useful RViz displays are `/global_costmap/costmap`,
`/local_costmap/costmap`, and `/plan`. The default pick and place base
standoffs are both `0.70 m`; adjust them with `pick_standoff_m` and
`place_standoff_m`. Set `enable_nav2:=false` to retain the previous direct
whole-body approach.

Nav2 publishes `/cmd_vel_navigation`, the manipulation controller publishes
`/cmd_vel_manipulation`, and `navigation_cmd_mux` is the only node that
publishes `/cmd_vel`.

After a completed place, a consecutive remote pick reuses the rear-LiDAR-safe
retreat before Nav2 is allowed to rotate or drive. The default retreat is
`0.40 m` backward at `0.25 m/s`; a close pick that already satisfies the
existing Nav2 skip condition proceeds directly without retreating.

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
ollama pull gemma3:4b
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
ros2 run sm_natural_language_task natural_language_task_console
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

The default natural-language path is open vocabulary. Local Ollama runs
`gemma3:4b` once per command and converts Korean or English input into concise
English visual queries. The queries are not checked against a fixed object-name
list, so a new object name does not require a parser alias. For example:

```text
오렌지를 분홍색 박스에 넣어줘
```

becomes:

```json
{"pick": "orange", "place": "pink box"}
```

The parser first separates the original pick and place phrases, then translates
each phrase in an isolated model request. A destination attribute is not copied
to an unqualified pick object: `사과를 노란색 박스에 넣어줘` becomes
`{"pick": "apple", "place": "yellow box"}`. The three warm model requests took
about 1.55-1.60 seconds in live tests, before any navigation or manipulation
starts.

Language output does not start motion by itself. The task manager keeps the base
stopped in `FIND_PICK` until YOLOE and grasping publish fresh current-task data.
An Ollama timeout, ambiguous command, missing detection, or missing grasp does
not fall back to a different object.

The old alias parser is available only as an explicit compatibility mode:

```bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py \
  use_local_llm:=false \
  use_rule_task_parser:=true
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
