# `sm_mapping_2d` Mapping MVP Design

## Objective

Add a removable ROS 2 package named `sm_mapping_2d` that wraps SLAM Toolbox for
one purpose: build and save a 2D occupancy map while the Isaac Sim mobile
manipulator is driven manually.

This first milestone proves the mapping sensor and TF chain without changing or
launching the working Nav2, whole-body control, perception, grasping, or task
orchestration packages.

## Package Boundary

`sm_mapping_2d` owns:

- the SLAM Toolbox online asynchronous mapping launch;
- mapping parameters for the Isaac Sim robot;
- an RViz configuration for LaserScan, TF, robot pose, and OccupancyGrid;
- documented commands for preflight checks, teleoperation, and map saving;
- package-contract tests for launch defaults and installed assets.

`sm_mapping_2d` does not own:

- Isaac Sim or the ROS bridge;
- robot velocity generation or teleoperation;
- Nav2 planning, control, or costmaps;
- localization from a saved map;
- front/rear LaserScan fusion;
- semantic objects, databases, ontologies, perception, or manipulation.

No existing package depends on `sm_mapping_2d` in this milestone. Removing the
package directory therefore removes the feature without changing the working
system.

## Existing Interfaces

The current Nav2 configuration establishes the robot-side conventions used by
the mapping package:

- odometry frame: `odom`;
- robot base frame: `chassis_link`;
- front LaserScan: `/laser_scan_1`;
- rear LaserScan: `/laser_scan_2`;
- simulation clock: `/clock`.

The ROS bridge was not publishing the scan topics during design-time inspection,
so runtime preflight must confirm each LaserScan header frame and its TF path to
`chassis_link` before mapping begins.

## Data Flow

```text
Isaac Sim /clock
Isaac Sim /laser_scan_1
TF: odom -> chassis_link -> lidar frame
                  |
                  v
       sm_mapping_2d mapping launch
                  |
                  v
       SLAM Toolbox async mapper
          |                 |
          v                 v
       /map            TF: map -> odom
          |
          v
occupancy map save + pose graph serialization
```

The first test uses `/laser_scan_1` only. This keeps the dependency and CPU cost
minimal, and manual rotation allows the robot to observe the rest of the room.
The scan topic remains a launch argument so a future scan-merger can supply a
combined topic without modifying this package.

## Package Shape

The package follows the workspace's existing `ament_python` launch/config-only
pattern.

```text
sm_mapping_2d/
├── config/
│   ├── slam_toolbox_mapping.yaml
│   └── mapping.rviz
├── launch/
│   └── mapping.launch.py
├── resource/
│   └── sm_mapping_2d
├── sm_mapping_2d/
│   └── __init__.py
├── test/
│   └── test_mapping_package_contract.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```

Generated map artifacts are operational data and are not written inside the
installed package. The README uses a user-selected output directory, with
`ros2_ws/maps/warehouse` as the example.

## Launch Contract

The package exposes:

```bash
ros2 launch sm_mapping_2d mapping.launch.py
```

Launch arguments:

| Argument | Default | Meaning |
|---|---|---|
| `use_sim_time` | `true` | Use Isaac Sim `/clock` |
| `scan_topic` | `/laser_scan_1` | LaserScan consumed by SLAM Toolbox |
| `map_frame` | `map` | Persistent global mapping frame |
| `odom_frame` | `odom` | Existing odometry frame |
| `base_frame` | `chassis_link` | Existing navigation base frame |
| `params_file` | package mapping YAML | Replaceable SLAM Toolbox configuration |
| `rviz` | `true` | Start the package RViz view |

The launch starts SLAM Toolbox and optional RViz only. It does not publish
`/cmd_vel` and does not launch Nav2 or any current Pick-and-Place nodes.

## Mapping Configuration

The initial configuration uses SLAM Toolbox's online asynchronous mode with:

- 0.05 m occupancy-grid resolution;
- simulation time enabled;
- `map`, `odom`, and `chassis_link` frame conventions;
- conservative map publication/update intervals rather than maximum-rate
  updates;
- scan matching and loop closure enabled;
- minimum translation and rotation thresholds so stationary scans do not cause
  unnecessary processing.

Parameters remain in a package-local YAML file. Performance tuning is allowed
only after the first map reveals a concrete problem.

## Operator Workflow

1. Open `project/tutorial/13_eew_slam_mapping.usd`, press Play, and confirm the
   ROS bridge is active.
2. Confirm `/clock`, `/odom`, `/laser_scan_1`, and the required TF chain.
3. Start `sm_mapping_2d` and confirm `/map` appears in RViz.
4. In a separate terminal, run `teleop_twist_keyboard` against `/cmd_vel`.
5. Drive a slow loop around the warehouse, revisiting the starting region to
   exercise loop closure.
6. Save an OccupancyGrid image/YAML with `nav2_map_server`'s `map_saver_cli`.
7. Serialize the SLAM Toolbox pose graph with `/slam_toolbox/serialize_map`.
8. Stop teleoperation, mapping, and Isaac Sim in that order.

Teleoperation remains a separate operator command so mapping can never move the
robot by itself and no velocity publisher is hidden inside the module.

## Failure Handling

- Missing `/clock`: no mapping run is accepted until simulation time advances.
- Missing scan: inspect the Isaac ROS bridge and the selected `scan_topic`.
- Missing TF: verify `odom -> chassis_link -> LaserScan frame`; do not invent a
  static transform unless it matches the USD sensor mounting.
- Distorted map while turning: reduce teleoperation angular speed before tuning
  SLAM parameters.
- Forks or pallet gaps absent from the map: first confirm the raw scan in RViz;
  this is a simulation geometry/sensor-height issue, not an SLAM parameter issue.
- CPU pressure: keep only Isaac Sim, ROS bridge, SLAM Toolbox, teleoperation, and
  one RViz process running for this milestone.

## Verification and Acceptance

Static checks:

- `colcon build --packages-select sm_mapping_2d` succeeds;
- package tests verify installed launch/config/RViz assets;
- launch-contract tests verify the frame and scan defaults;
- launch text contains no `/cmd_vel` publisher and no dependency on current
  control packages.

Runtime acceptance:

- `/map` updates while the robot is teleoperated;
- warehouse walls, pallets, barrels, and the forklift appear in approximately
  correct relative positions;
- revisiting the starting area does not produce a duplicated wall layout;
- both an occupancy map (`.pgm`/`.yaml`) and serialized pose graph are written;
- stopping or deleting `sm_mapping_2d` leaves the existing Pick-and-Place launch
  and packages unchanged.

Localization, Nav2 consumption of the saved map, dual-LiDAR fusion, and semantic
annotations are explicit later milestones and are not acceptance requirements.
