# sm_semantic_map

This package owns semantic object-map data and calibration utilities. The
warehouse seed in `config/warehouse_objects.yaml` contains camera-observed
object poses in the fixed `map` frame plus prevalidated Nav2 approach poses.

Collect repeated positions from one perception topic with:

```bash
ros2 run sm_semantic_map collect_object_positions \
  --topic /sm_florence_2_vlm/detections \
  --objects "red can" orange "yellow box" \
  --samples 12
```

The collector reports median world/map positions, standard deviation, sample
count, and median detector confidence. It never publishes motion commands.

Initialize and query the SQLite database with:

```bash
ros2 run sm_semantic_map semantic_map_db \
  --db semantic_map.sqlite3 init \
  --seed install/sm_semantic_map/share/sm_semantic_map/config/warehouse_objects.yaml

ros2 run sm_semantic_map semantic_map_db \
  --db semantic_map.sqlite3 find "파란 캔"
```

## Natural language lookup validation

The lookup-only bringup starts three replaceable modules and no navigation or
velocity publisher:

```text
/natural_language_task
  -> sm_natural_language_task
  -> /semantic_lookup/task
  -> sm_task_orchestrator semantic_task_resolver
  -> /semantic_map/find_object (pick, then place)
  -> /semantic_lookup/resolved_task
```

Run the deterministic MVP parser and database resolver with:

```bash
ros2 launch sm_bringup semantic_task_lookup.launch.py
```

Send a Korean command and inspect the resolved map poses:

```bash
ros2 topic pub --once /natural_language_task std_msgs/msg/String \
  "{data: '파란 캔을 노란 상자에 넣어줘'}"

ros2 topic echo /semantic_lookup/resolved_task
```

For open-vocabulary parsing, replace the rule adapter at launch time:

```bash
ros2 launch sm_bringup semantic_task_lookup.launch.py \
  use_ollama:=true use_rule_fallback:=false
```

## Integrated MVP

The complete lookup, Nav2 approach, and unchanged Pick & Place pipeline is owned
by `sm_semantic_mvp`:

```bash
ros2 launch sm_semantic_mvp semantic_db_existing_pick_place.launch.py
```

The Semantic DB stays read-only during this Isaac Sim MVP.
