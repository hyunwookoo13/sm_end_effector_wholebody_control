# `sm_mapping_2d` Mapping MVP Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a removable ROS 2 package that creates and saves a warehouse 2D map from one Isaac Sim LaserScan while the operator drives the robot manually.

**Architecture:** `sm_mapping_2d` is an independent `ament_python` launch/config package around SLAM Toolbox's asynchronous mapper. It consumes `/clock`, `/laser_scan_1`, and the existing `odom -> chassis_link -> lidar` TF chain, publishes `/map` and `map -> odom`, and never starts navigation, manipulation, perception, or a velocity publisher.

**Tech Stack:** ROS 2 Humble, Python launch, SLAM Toolbox 2.6, RViz2, Nav2 map saver, pytest, PyYAML

## Global Constraints

- Create only `ros2_ws/src/sm_mapping_2d`; do not modify any existing ROS package.
- Use `ament_python` to match the current workspace package pattern.
- Default frames are `map`, `odom`, and `chassis_link`.
- Default scan input is `/laser_scan_1`; dual-LiDAR fusion is outside this milestone.
- Use Isaac Sim time by default with `use_sim_time:=true`.
- The package must not publish `/cmd_vel`, start teleoperation, or launch Nav2.
- Mapping runs in online asynchronous mode at 0.05 m resolution with conservative update thresholds.
- Generated `.pgm`, `.yaml`, and pose-graph files live under `ros2_ws/maps/`, not inside the package.
- Existing uncommitted files belong to the user; stage and commit only files under `ros2_ws/src/sm_mapping_2d`.

---

## File Structure

Create these files and no others:

```text
ros2_ws/src/sm_mapping_2d/
├── config/
│   ├── mapping.rviz
│   └── slam_toolbox_mapping.yaml
├── launch/
│   └── mapping.launch.py
├── resource/
│   └── sm_mapping_2d
├── sm_mapping_2d/
│   └── __init__.py
├── test/
│   └── test_mapping_package_contract.py
├── README.md
├── package.xml
├── setup.cfg
└── setup.py
```

Responsibilities:

- `package.xml`, `setup.py`, `setup.cfg`, `resource/`, `__init__.py`: package discovery, dependencies, and asset installation.
- `launch/mapping.launch.py`: the only executable entry point; starts SLAM Toolbox and optional RViz.
- `config/slam_toolbox_mapping.yaml`: CPU-conscious mapping and scan-matching parameters.
- `config/mapping.rviz`: fixed `map` view containing Map, LaserScan, and TF displays.
- `README.md`: build, preflight, teleoperation, map saving, shutdown, and removal instructions.
- `test/test_mapping_package_contract.py`: static contract checks that prevent accidental ownership creep.

---

### Task 1: Package Contract and Metadata

**Files:**
- Create: `ros2_ws/src/sm_mapping_2d/test/test_mapping_package_contract.py`
- Create: `ros2_ws/src/sm_mapping_2d/package.xml`
- Create: `ros2_ws/src/sm_mapping_2d/setup.py`
- Create: `ros2_ws/src/sm_mapping_2d/setup.cfg`
- Create: `ros2_ws/src/sm_mapping_2d/resource/sm_mapping_2d`
- Create: `ros2_ws/src/sm_mapping_2d/sm_mapping_2d/__init__.py`

**Interfaces:**
- Consumes: ROS 2 Humble package discovery and the installed `slam_toolbox`, `rviz2`, `nav2_map_server`, and `teleop_twist_keyboard` packages.
- Produces: an installable package named `sm_mapping_2d` whose launch/config/README assets can be installed by later tasks.

- [ ] **Step 1: Write the failing metadata contract test**

Create `test/test_mapping_package_contract.py`:

```python
from pathlib import Path
from xml.etree import ElementTree

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_manifest_declares_mapping_runtime_dependencies():
    package = ElementTree.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert package.findtext("name") == "sm_mapping_2d"
    dependencies = {node.text for node in package.findall("exec_depend")}
    assert {
        "launch",
        "launch_ros",
        "nav2_map_server",
        "rviz2",
        "slam_toolbox",
        "teleop_twist_keyboard",
    } <= dependencies


def test_setup_installs_launch_config_and_readme_assets():
    setup_text = (PACKAGE_ROOT / "setup.py").read_text()
    assert 'glob("launch/*.launch.py")' in setup_text
    assert 'glob("config/*.yaml")' in setup_text
    assert 'glob("config/*.rviz")' in setup_text
    assert '"README.md"' in setup_text
```

The `yaml` import is used by Task 2 tests and intentionally establishes the test dependency now.

- [ ] **Step 2: Run the test and verify the metadata is absent**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
python3 -m pytest -q src/sm_mapping_2d/test/test_mapping_package_contract.py
```

Expected: FAIL with `FileNotFoundError` for `package.xml`.

- [ ] **Step 3: Add the minimal package metadata**

Create `package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sm_mapping_2d</name>
  <version>0.1.0</version>
  <description>Removable SLAM Toolbox mapping integration for the mobile manipulator.</description>
  <maintainer email="gunmin0525@gmail.com">kiro</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>teleop_twist_keyboard</exec_depend>

  <test_depend>python3-pytest</test_depend>
  <test_depend>python3-yaml</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Create `setup.py`:

```python
from glob import glob

from setuptools import setup


package_name = "sm_mapping_2d"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Removable SLAM Toolbox mapping integration for the mobile manipulator.",
    license="MIT",
    tests_require=["pytest"],
)
```

Create `setup.cfg`:

```ini
[develop]
script_dir=$base/lib/sm_mapping_2d
[install]
install_scripts=$base/lib/sm_mapping_2d
```

Create an empty `resource/sm_mapping_2d` and an empty `sm_mapping_2d/__init__.py`.

- [ ] **Step 4: Add temporary empty installed assets so the metadata test can execute**

Create an empty `README.md`, then create the `launch/` and `config/` directories. Do not add launch/config implementations until their failing tests exist in Task 2.

- [ ] **Step 5: Run the metadata contract**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
python3 -m pytest -q src/sm_mapping_2d/test/test_mapping_package_contract.py
```

Expected: `2 passed`.

- [ ] **Step 6: Commit the scaffold only**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/src/sm_mapping_2d
git commit -m "feat: scaffold removable 2d mapping module"
```

Expected: the commit contains only `ros2_ws/src/sm_mapping_2d/**`.

---

### Task 2: SLAM Toolbox Launch and CPU-Conscious Mapping Configuration

**Files:**
- Modify: `ros2_ws/src/sm_mapping_2d/test/test_mapping_package_contract.py`
- Create: `ros2_ws/src/sm_mapping_2d/launch/mapping.launch.py`
- Create: `ros2_ws/src/sm_mapping_2d/config/slam_toolbox_mapping.yaml`

**Interfaces:**
- Consumes: `/clock`, `/laser_scan_1`, `odom -> chassis_link`, and the LaserScan sensor TF.
- Produces: `/map`, `/map_metadata`, `map -> odom`, and SLAM Toolbox map-saving services.

- [ ] **Step 1: Add failing launch and YAML contract tests**

Append to `test/test_mapping_package_contract.py`:

```python
def test_launch_defaults_to_current_robot_frames_and_front_scan():
    launch_text = (PACKAGE_ROOT / "launch" / "mapping.launch.py").read_text()
    for token in (
        'default_value="/laser_scan_1"',
        'default_value="map"',
        'default_value="odom"',
        'default_value="chassis_link"',
        'default_value="true"',
        'executable="async_slam_toolbox_node"',
        'executable="rviz2"',
    ):
        assert token in launch_text
    assert "/cmd_vel" not in launch_text
    assert "nav2_controller" not in launch_text
    assert "sm_task_orchestrator" not in launch_text
    assert "sm_ee_wholebody_control" not in launch_text


def test_mapping_config_uses_cpu_conscious_online_mapping_defaults():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "slam_toolbox_mapping.yaml").read_text()
    )
    params = config["slam_toolbox"]["ros__parameters"]
    assert params["mode"] == "mapping"
    assert params["resolution"] == 0.05
    assert params["map_update_interval"] == 2.0
    assert params["minimum_time_interval"] == 0.2
    assert params["minimum_travel_distance"] == 0.1
    assert params["minimum_travel_heading"] == 0.1
    assert params["enable_interactive_mode"] is False
    assert params["do_loop_closing"] is True
```

- [ ] **Step 2: Run the new contracts and verify they fail**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
python3 -m pytest -q src/sm_mapping_2d/test/test_mapping_package_contract.py
```

Expected: FAIL because `mapping.launch.py` and `slam_toolbox_mapping.yaml` do not exist.

- [ ] **Step 3: Implement `mapping.launch.py`**

Create:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = LaunchConfiguration("scan_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    params_file = LaunchConfiguration("params_file")
    rviz = LaunchConfiguration("rviz")

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "scan_topic": scan_topic,
                "map_frame": map_frame,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="sm_mapping_rviz",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("sm_mapping_2d"), "config", "mapping.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("scan_topic", default_value="/laser_scan_1"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="chassis_link"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_mapping_2d"),
                        "config",
                        "slam_toolbox_mapping.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument("rviz", default_value="true"),
            slam_toolbox,
            rviz_node,
        ]
    )
```

- [ ] **Step 4: Implement `slam_toolbox_mapping.yaml`**

Create:

```yaml
slam_toolbox:
  ros__parameters:
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    odom_frame: odom
    map_frame: map
    base_frame: chassis_link
    scan_topic: /laser_scan_1
    use_map_saver: true
    mode: mapping

    debug_logging: false
    throttle_scans: 1
    transform_publish_period: 0.05
    map_update_interval: 2.0
    resolution: 0.05
    min_laser_range: 0.45
    max_laser_range: 10.0
    minimum_time_interval: 0.2
    transform_timeout: 0.2
    tf_buffer_duration: 30.0
    stack_size_to_use: 40000000
    enable_interactive_mode: false

    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.1
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    min_pass_through: 2
    occupancy_threshold: 0.1
```

- [ ] **Step 5: Run the package contract tests**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
python3 -m pytest -q src/sm_mapping_2d/test/test_mapping_package_contract.py
```

Expected: `4 passed`.

- [ ] **Step 6: Validate launch syntax without starting Isaac Sim**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sm_mapping_2d
source install/setup.bash
ros2 launch sm_mapping_2d mapping.launch.py --show-args
```

Expected: all seven launch arguments appear with the defaults specified above.

- [ ] **Step 7: Commit mapping behavior**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/src/sm_mapping_2d
git commit -m "feat: add slam toolbox mapping launch"
```

Expected: the commit modifies only the new package.

---

### Task 3: RViz View and Reproducible Operator Workflow

**Files:**
- Modify: `ros2_ws/src/sm_mapping_2d/test/test_mapping_package_contract.py`
- Create: `ros2_ws/src/sm_mapping_2d/config/mapping.rviz`
- Modify: `ros2_ws/src/sm_mapping_2d/README.md`

**Interfaces:**
- Consumes: `/map`, `/laser_scan_1`, `/tf`, `/tf_static`, `/clock`, and operator keyboard input.
- Produces: a visible map validation view and exact save artifacts under `/home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse/`.

- [ ] **Step 1: Add failing RViz and workflow contract tests**

Append to `test/test_mapping_package_contract.py`:

```python
def test_rviz_and_readme_expose_the_mapping_workflow():
    rviz_text = (PACKAGE_ROOT / "config" / "mapping.rviz").read_text()
    assert "rviz_default_plugins/Map" in rviz_text
    assert "rviz_default_plugins/LaserScan" in rviz_text
    assert "rviz_default_plugins/TF" in rviz_text
    assert "Fixed Frame: map" in rviz_text
    assert "Value: /map" in rviz_text
    assert "Value: /laser_scan_1" in rviz_text

    readme = (PACKAGE_ROOT / "README.md").read_text()
    assert "teleop_twist_keyboard" in readme
    assert "map_saver_cli" in readme
    assert "/slam_toolbox/serialize_map" in readme
    assert "13_eew_slam_mapping.usd" in readme
```

- [ ] **Step 2: Run the new contract and verify it fails**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
python3 -m pytest -q src/sm_mapping_2d/test/test_mapping_package_contract.py
```

Expected: FAIL because the RViz configuration is absent and the README is empty.

- [ ] **Step 3: Add the minimal RViz configuration**

Create `config/mapping.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map
      Topic:
        Value: /map
      Update Topic:
        Value: /map_updates
    - Alpha: 1
      Class: rviz_default_plugins/LaserScan
      Enabled: true
      Name: Front LaserScan
      Size (Pixels): 3
      Style: Points
      Topic:
        Value: /laser_scan_1
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 20
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Top Down
      Scale: 30
Window Geometry:
  Height: 900
  Width: 1400
```

- [ ] **Step 4: Write the exact operator README**

Write `README.md` with these sections and commands:

````markdown
# sm_mapping_2d

`sm_mapping_2d` runs a removable SLAM Toolbox mapping profile for the Isaac Sim
mobile manipulator. It does not launch Nav2, perception, manipulation, or a
velocity publisher.

## Build

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sm_mapping_2d
source install/setup.bash
```

## Simulator and ROS preflight

Open `/home/kiro/Desktop/hw_ws/project/tutorial/13_eew_slam_mapping.usd` and
press Play. Confirm the simulation clock, odometry transform, and front scan:

```bash
ros2 topic echo /clock --once
ros2 topic echo /laser_scan_1 --once --field header
ros2 run tf2_ros tf2_echo odom chassis_link
```

The LaserScan header's `frame_id` must also have a TF path to `chassis_link`.
Do not add a guessed static transform if this path is missing; fix the Isaac Sim
sensor/TF publisher instead.

## Start mapping

```bash
ros2 launch sm_mapping_2d mapping.launch.py
```

To map without RViz or use a different LaserScan:

```bash
ros2 launch sm_mapping_2d mapping.launch.py rviz:=false
ros2 launch sm_mapping_2d mapping.launch.py scan_topic:=/laser_scan_2
```

## Teleoperate

Run this in a separate sourced terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Drive slowly around the warehouse and return near the starting region once so
SLAM Toolbox can close the loop. Mapping itself never moves the robot.

## Save the map

```bash
mkdir -p /home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse
ros2 run nav2_map_server map_saver_cli \
  -f /home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse/warehouse
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse/warehouse'}"
```

Successful saving produces `warehouse.pgm`, `warehouse.yaml`,
`warehouse.posegraph`, and `warehouse.data`.

## Shutdown and removal

Stop teleoperation, mapping, and Isaac Sim in that order. To remove the feature,
delete only `ros2_ws/src/sm_mapping_2d` and rebuild the workspace; no current
bringup package references this module.
````

- [ ] **Step 5: Run all package contracts**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
python3 -m pytest -q src/sm_mapping_2d/test/test_mapping_package_contract.py
```

Expected: `5 passed`.

- [ ] **Step 6: Rebuild and verify installed assets**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sm_mapping_2d
source install/setup.bash
ros2 pkg prefix sm_mapping_2d
test -f install/sm_mapping_2d/share/sm_mapping_2d/config/mapping.rviz
test -f install/sm_mapping_2d/share/sm_mapping_2d/config/slam_toolbox_mapping.yaml
test -f install/sm_mapping_2d/share/sm_mapping_2d/README.md
```

Expected: package prefix is `/home/kiro/Desktop/hw_ws/ros2_ws/install/sm_mapping_2d`, and all three `test -f` commands exit 0.

- [ ] **Step 7: Commit the operator workflow**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/src/sm_mapping_2d
git commit -m "docs: add 2d mapping operator workflow"
```

Expected: the commit modifies only the new package.

---

### Task 4: Runtime Mapping Gate

**Files:**
- Verify only: no source files change unless a runtime observation disproves a documented default.

**Interfaces:**
- Consumes: the running `13_eew_slam_mapping.usd` ROS bridge and manual teleoperation.
- Produces: `/map` plus saved warehouse OccupancyGrid and SLAM Toolbox pose graph files.

- [ ] **Step 1: Confirm live inputs after Isaac Sim Play**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /clock --once
ros2 topic echo /laser_scan_1 --once --field header
ros2 run tf2_ros tf2_echo odom chassis_link
```

Expected: simulation time advances, the scan has a non-empty frame ID, and `odom -> chassis_link` updates without TF lookup errors.

- [ ] **Step 2: Start a headless mapping smoke test**

Run in terminal A:

```bash
ros2 launch sm_mapping_2d mapping.launch.py rviz:=false
```

Run in terminal B:

```bash
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map odom
```

Expected: `/map` reports `resolution: 0.05`, nonzero width and height, and `map -> odom` is available.

- [ ] **Step 3: Perform the manual mapping loop**

Run in terminal C:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
```

Drive at low speed around both sides of the central obstacles, observe the wall,
pallet, barrel, and forklift regions, and return near the start. Do not run the
natural-language Pick-and-Place bringup during this gate.

- [ ] **Step 4: Save both map representations**

Run:

```bash
mkdir -p /home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse
ros2 run nav2_map_server map_saver_cli \
  -f /home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse/warehouse
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse/warehouse'}"
ls -lh /home/kiro/Desktop/hw_ws/ros2_ws/maps/warehouse/warehouse.{pgm,yaml,posegraph,data}
```

Expected: four non-empty files are listed.

- [ ] **Step 5: Run the final regression gate**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select sm_mapping_2d --event-handlers console_direct+
colcon test-result --verbose
git -C /home/kiro/Desktop/hw_ws status --short
```

Expected: `sm_mapping_2d` tests pass; generated maps are untracked operational data; pre-existing user changes remain unchanged; no existing package file was added to the implementation commits.
