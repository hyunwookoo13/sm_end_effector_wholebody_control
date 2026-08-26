# Semantic DB + Existing Pick & Place MVP

This package adds only the orchestration layer in front of the existing fast
pick-and-place pipeline.

```text
/natural_language_task
  -> static Semantic DB lookup
  -> DB Nav2 approach to the pick workspace
  -> existing perception + Whole-body Pick
  -> safe TRANSPORT pose
  -> DB Nav2 approach to the place workspace
  -> fresh post-arrival place perception + existing Whole-body Place
```

The semantic DB remains read-only during execution. Pick and place may belong to
different zones; the mission orchestrator requests each stored approach pose in
sequence. The existing task manager is paused only after `PICK:HOLD`, while the
arm retracts to `TRANSPORT`, and resumes its original Place behavior after the
second Nav2 approach. A complete Nav2 `SUCCEEDED` result remains the fallback if
an early distance handoff is not reached.

The default `nav2_static_map_mvp.yaml` retains the fixed `map` frame and static
costmap while restoring the previous long-range dynamics: `1.5 m/s` maximum
linear velocity, `2.0 m/s²` acceleration, and `0.45 rad/s` maximum rotation.

## Run

Start Isaac Sim and press **Play**, then run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity></Tracing></Domain></CycloneDDS>'
ros2 launch sm_semantic_mvp semantic_db_existing_pick_place.launch.py
```

In another terminal, submit a task:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><Tracing><Verbosity>severe</Verbosity></Tracing></Domain></CycloneDDS>'
ros2 topic pub --once /natural_language_task std_msgs/msg/String \
  "{data: '빨간 캔을 분홍 박스에 넣어줘'}"
```

Expected sequence:

1. `red_can` resolves to `first_table` and `pink_box` to `second_table`.
2. The mission orchestrator sends the stored `red_can` approach pose to Nav2.
3. At the first handoff, it dispatches the unchanged existing task contract:
   `{"pick": "red can", "place": "pink box"}`.
4. On `PICK:HOLD`, the task manager retracts the arm and waits in
   `WAIT_PLACE_NAVIGATION`.
5. The mission orchestrator sends the stored `pink_box` approach pose to Nav2.
6. At the second handoff, the manager discards transit-time cached Place data,
   accepts a fresh camera-2 detection, and continues `PLACE -> DONE`.

Useful status topics:

```bash
ros2 topic echo /semantic_mvp/status
ros2 topic echo /semantic_navigation/status
ros2 topic echo /pick_place_task_state
ros2 topic echo /pick_place_task
```
