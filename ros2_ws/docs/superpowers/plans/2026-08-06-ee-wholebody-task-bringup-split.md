# EE Whole-Body, Task Orchestrator, and Bringup Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace `ee_switch_debug` with `sm_task_orchestrator`, `sm_ee_wholebody_control`, and `sm_bringup` without changing the verified natural-language Pick-and-Place behavior.

**Architecture:** Move the existing Pick/Place state machine and Nav2 goal geometry into the orchestrator, move EE-targeted arm/base controllers and their resources into the control package, and move cross-package launch composition into a logic-free bringup package. Preserve every runtime node name, topic, action, frame, parameter default, QoS choice, and controller value; only ownership/import/resource paths change.

**Tech Stack:** ROS 2 Humble, `ament_python`, `ament_cmake`, Python 3, `rclpy`, Nav2, ROS 2 launch, pytest, colcon, Isaac Sim ROS bridge.

## Global Constraints

- Treat commit `f8a7122` as the verified runtime rollback baseline and commit `1da97b0` as the approved design baseline.
- Preserve the untracked `ros2_ws/dds_setting/` directory and never stage, delete, or rewrite it.
- Do not change velocities, gains, offsets, timeouts, model prompts, aliases, standoff distances, task phases, or handoff behavior.
- Preserve runtime node names, executable names, topics, actions, message types, frame names, QoS, and launch argument names/defaults.
- Keep `navigation_cmd_mux` as the sole final `/cmd_vel` publisher.
- Use `/usr/bin/python3` for pytest after sourcing `/opt/ros/humble/setup.bash` and the current workspace overlay.
- Use `colcon build --executor sequential` for every build to minimize CPU load.
- Move implementation files without internal edits first; make import/package-owner updates separately and minimally.
- Commit each extraction and cleanup gate separately so it can be reverted independently.
- Top-down grasp integration, control tuning, new collision logic, hot swapping, and DDS cleanup are out of scope.

---

### Task 1: Freeze the Phase 2 Baseline and Ownership Contract

**Files:**
- Create: `ros2_ws/docs/architecture/runtime-contract-phase-2.md`

**Interfaces:**
- Consumes: Current `ee_switch_debug` metadata, installed executables, full launch arguments, and Phase 1 runtime contract.
- Produces: A committed baseline record used by all later tasks.

- [ ] **Step 1: Record current package and launch ownership**

Create the Phase 2 runtime contract with the current package list, all legacy console scripts, six launch filenames, the full launch command, and protected defaults copied from Phase 1. Include:

```text
pick_place_task_manager -> ee_switch_debug (baseline only)
arm_yaw_rho_z_position_controller -> ee_switch_debug (baseline only)
florence_long_range_pick_place_control.launch.py -> ee_switch_debug (baseline only)
navigation_cmd_mux -> sm_base_control_manager
/cmd_vel final publisher -> navigation_cmd_mux only
```

- [ ] **Step 2: Run and record baseline tests**

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest \
  src/ee_switch_debug/test src/sm_natural_language_task/test \
  src/sm_navigation_nav2/test src/sm_base_control_manager/test \
  src/sm_florence_2_vlm_ros2/test src/sm_grasping_ros2/test -q
```

Expected: `90 passed`; record exact count and elapsed time.

- [ ] **Step 3: Commit baseline evidence**

```bash
git add ros2_ws/docs/architecture/runtime-contract-phase-2.md
git commit -m "docs: freeze phase two runtime baseline"
```

### Task 2: Extract `sm_task_orchestrator`

**Files:**
- Create: `ros2_ws/src/sm_task_orchestrator/{package.xml,setup.py,setup.cfg}`
- Create: `ros2_ws/src/sm_task_orchestrator/resource/sm_task_orchestrator`
- Create: `ros2_ws/src/sm_task_orchestrator/sm_task_orchestrator/__init__.py`
- Create: `ros2_ws/src/sm_task_orchestrator/test/test_package_contract.py`
- Move: `ee_switch_debug/pick_place_task_manager.py` and `navigation_geometry.py` into the new Python module directory.
- Move: `test_consecutive_task_retreat.py`, `test_navigation_geometry.py`, `test_pick_place_precache.py`, and `test_semantic_box_aliases.py` into the new test directory.
- Modify: moved tests only for package imports.
- Modify: `ros2_ws/src/ee_switch_debug/{setup.py,package.xml}`

**Interfaces:**
- Consumes: `/pick_place_task`, perception/grasp topics, `/arm_task_state`, Nav2 `NavigateToPose`, rear laser scan.
- Produces: unchanged `pick_place_task_manager` executable/node and unchanged task, base-mode, blend, and retreat topics.

- [ ] **Step 1: Write the ownership test and verify RED**

```python
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

def test_manifest_names_task_orchestrator():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert root.findtext("name") == "sm_task_orchestrator"

def test_task_modules_are_owned_here():
    module_root = PACKAGE_ROOT / "sm_task_orchestrator"
    assert (module_root / "pick_place_task_manager.py").is_file()
    assert (module_root / "navigation_geometry.py").is_file()
```

```bash
/usr/bin/python3 -m pytest src/sm_task_orchestrator/test/test_package_contract.py -q
```

Expected: FAIL because package metadata and moved modules do not exist yet.

- [ ] **Step 2: Create package metadata**

Use `ament_python` and direct dependencies:

```xml
<exec_depend>action_msgs</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>tf2_ros</exec_depend>
```

Define exactly:

```python
"pick_place_task_manager = sm_task_orchestrator.pick_place_task_manager:main"
```

- [ ] **Step 3: Move code/tests without logic changes**

Use `git mv`; update imports only to:

```python
from sm_task_orchestrator.pick_place_task_manager import ...
from sm_task_orchestrator.navigation_geometry import ...
```

Keep `from .navigation_geometry import (...)` inside the manager.

- [ ] **Step 4: Remove old entry point and stale dependencies**

Delete only the manager console-script line from the legacy setup. Remove `nav2_msgs` and `action_msgs` from the legacy manifest only after `rg` proves no remaining legacy module imports them.

- [ ] **Step 5: Verify GREEN**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest src/sm_task_orchestrator/test -q
```

- [ ] **Step 6: Build and verify executable owner**

```bash
colcon build --executor sequential --packages-select sm_task_orchestrator
source install/setup.bash
ros2 pkg executables sm_task_orchestrator
```

Expected: `sm_task_orchestrator pick_place_task_manager`.

- [ ] **Step 7: Commit**

```bash
git add ros2_ws/src/sm_task_orchestrator ros2_ws/src/ee_switch_debug
git commit -m "refactor: extract task orchestrator package"
```

### Task 3: Extract `sm_ee_wholebody_control`

**Files:**
- Create: `ros2_ws/src/sm_ee_wholebody_control/{package.xml,setup.py,setup.cfg}`
- Create: `ros2_ws/src/sm_ee_wholebody_control/resource/sm_ee_wholebody_control`
- Create: `ros2_ws/src/sm_ee_wholebody_control/sm_ee_wholebody_control/__init__.py`
- Move: all remaining legacy Python modules except the old `__init__.py` into the new module directory; create a fresh empty `sm_ee_wholebody_control/__init__.py`.
- Move: legacy `config/*.yaml` into the new config directory.
- Move: `arm_position_target_in.launch.py`, `grasp_wholebody.launch.py`, and `wholebody_monitor.launch.py` into the new launch directory.
- Move: `test_wrist_orientation.py` into the new test directory.
- Create: `ros2_ws/src/sm_ee_wholebody_control/test/test_package_contract.py`
- Modify: `ros2_ws/src/sm_grasping_ros2/launch/rsd455_florence_grasping.launch.py`

**Interfaces:**
- Consumes: EE/TF targets, `/arm_task_command`, joint state, grasp target, and existing base mode/blend inputs.
- Produces: unchanged controller/node names, joint/gripper commands, `/arm_task_state`, and `/cmd_vel_manipulation`.

- [ ] **Step 1: Write ownership test and verify RED**

```python
from pathlib import Path
import ast

ROOT = Path(__file__).resolve().parents[1]

def test_active_controller_and_config_are_owned_here():
    assert (ROOT / "sm_ee_wholebody_control" / "arm_yaw_rho_z_position_controller.py").is_file()
    assert (ROOT / "config" / "arm_position_target_in.yaml").is_file()

def test_setup_preserves_active_entry_point():
    tree = ast.parse((ROOT / "setup.py").read_text())
    assert "arm_yaw_rho_z_position_controller" in ast.unparse(tree)
```

Run the focused file and expect failure before the move.

- [ ] **Step 2: Create metadata and preserve retained entry points**

Copy all remaining legacy console-script names, changing only the module owner, for example:

```python
"arm_yaw_rho_z_position_controller = sm_ee_wholebody_control.arm_yaw_rho_z_position_controller:main"
```

Install both YAML files and three controller-focused launches.

- [ ] **Step 3: Move code/resources without tuning**

Use `git mv`. Replace only `ee_switch_debug.` Python imports and `package="ee_switch_debug"`/`FindPackageShare("ee_switch_debug")` launch owners. Update the grasping launch controller owner to `sm_ee_wholebody_control`.

- [ ] **Step 4: Run focused tests**

```bash
/usr/bin/python3 -m pytest src/sm_ee_wholebody_control/test -q
```

Expected: ownership and wrist-orientation tests pass with unchanged numeric expectations.

- [ ] **Step 5: Build sequentially**

```bash
colcon build --executor sequential \
  --packages-select sm_ee_wholebody_control sm_grasping_ros2
source install/setup.bash
ros2 pkg executables sm_ee_wholebody_control
```

Expected: all retained legacy executables except `pick_place_task_manager` are owned by this package.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/sm_ee_wholebody_control ros2_ws/src/ee_switch_debug \
  ros2_ws/src/sm_grasping_ros2/launch/rsd455_florence_grasping.launch.py
git commit -m "refactor: extract ee wholebody control package"
```

### Task 4: Create Logic-Free `sm_bringup`

**Files:**
- Create: `ros2_ws/src/sm_bringup/{package.xml,CMakeLists.txt}`
- Create: `ros2_ws/src/sm_bringup/launch/natural_language_pick_place.launch.py`
- Create: `ros2_ws/src/sm_bringup/launch/florence_long_range_pick_place_control.launch.py`
- Move: `florence_grasp_position_control.launch.py` and `florence_pick_place_control.launch.py` into `sm_bringup/launch/`.
- Move/rename: legacy full long-range launch → `sm_bringup/launch/natural_language_pick_place.launch.py`.
- Move: legacy `README_PICK_PLACE.md` → `sm_bringup/README_PICK_PLACE.md`.
- Create: `ros2_ws/src/sm_bringup/test/test_bringup_contract.py`

**Interfaces:**
- Consumes: Executables/resources from perception, grasping, language, Nav2, base manager, orchestrator, and whole-body control.
- Produces: canonical and compatibility full-system launches; no runtime node.

- [ ] **Step 1: Write bringup contract and verify RED**

```python
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def test_manifest_depends_on_runtime_packages():
    xml = ET.parse(ROOT / "package.xml").getroot()
    deps = {node.text for node in xml.findall("exec_depend")}
    assert {"sm_base_control_manager", "sm_ee_wholebody_control",
            "sm_florence_2_vlm_ros2", "sm_grasping_ros2",
            "sm_natural_language_task", "sm_navigation_nav2",
            "sm_task_orchestrator"} <= deps

def test_canonical_launch_has_new_owners_only():
    text = (ROOT / "launch" / "natural_language_pick_place.launch.py").read_text()
    assert 'package="sm_task_orchestrator"' in text
    assert 'package="sm_ee_wholebody_control"' in text
    assert "ee_switch_debug" not in text
```

Expected: FAIL before package creation.

- [ ] **Step 2: Create an `ament_cmake` launch-only package**

Declare `ament_cmake`, `launch`, `launch_ros`, and the seven runtime packages. Use:

```cmake
cmake_minimum_required(VERSION 3.8)
project(sm_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README_PICK_PLACE.md DESTINATION share/${PROJECT_NAME})
ament_package()
```

- [ ] **Step 3: Move integration launches and replace owners only**

In the canonical launch change only:

```python
package="sm_task_orchestrator"
package="sm_ee_wholebody_control"
FindPackageShare("sm_ee_wholebody_control")
```

All argument names/defaults and runtime values remain unchanged.

- [ ] **Step 4: Add thin compatibility include**

The old filename under `sm_bringup` includes the canonical launch:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    source = PythonLaunchDescriptionSource(PathJoinSubstitution([
        FindPackageShare("sm_bringup"), "launch",
        "natural_language_pick_place.launch.py",
    ]))
    return LaunchDescription([IncludeLaunchDescription(source)])
```

- [ ] **Step 5: Test, build, and render arguments**

```bash
/usr/bin/python3 -m pytest src/sm_bringup/test -q
colcon build --executor sequential --packages-up-to sm_bringup
source install/setup.bash
ros2 launch sm_bringup natural_language_pick_place.launch.py --show-args
```

Expected: tests/build pass and protected defaults match the Phase 2 contract.

- [ ] **Step 6: Commit**

```bash
git add ros2_ws/src/sm_bringup ros2_ws/src/ee_switch_debug
git commit -m "refactor: add modular system bringup package"
```

### Task 5: Remove `ee_switch_debug` Safely

**Files:**
- Move: legacy `docs/` and remaining top-level historical Markdown into `ros2_ws/docs/archive/ee_switch_debug/`.
- Create: `ros2_ws/docs/archive/ee_switch_debug/README.md`
- Delete after scans: legacy packaging/resource files and empty/cache directories.
- Modify: `README.md` and current non-archived documentation.
- Modify: `ros2_ws/docs/architecture/runtime-contract-phase-2.md`

**Interfaces:**
- Consumes: New owners from Tasks 2-4.
- Produces: No legacy package or live reference.

- [ ] **Step 1: Add archive warning**

```markdown
# Archived `ee_switch_debug` material

These files describe historical experiments and pre-split paths. Do not run
their commands as current instructions. Current owners are
`sm_task_orchestrator`, `sm_ee_wholebody_control`, and `sm_bringup`; use the
root README for supported commands.
```

- [ ] **Step 2: Move historical docs and scan**

```bash
rg -n "ee_switch_debug" ros2_ws/src README.md ros2_ws/docs \
  --glob '!ros2_ws/docs/archive/ee_switch_debug/**'
```

Expected before fixes: only supported docs/metadata references; no live Python import or launch owner remains.

- [ ] **Step 3: Update supported commands**

Use only:

```bash
colcon build --executor sequential --packages-up-to sm_bringup
ros2 launch sm_bringup natural_language_pick_place.launch.py
```

Document all eight final packages.

- [ ] **Step 4: Prove legacy code is unreferenced before deletion**

```bash
rg -n "from ee_switch_debug|import ee_switch_debug|package=[\"']ee_switch_debug|FindPackageShare\([\"']ee_switch_debug" ros2_ws/src
rg -n "ee_switch_debug" ros2_ws/src/*/{package.xml,setup.py,setup.cfg,CMakeLists.txt} 2>/dev/null
```

Expected: no matches. Then remove only the verified legacy packaging and empty directories; do not touch `dds_setting/`.

- [ ] **Step 5: Run final-owner tests**

```bash
/usr/bin/python3 -m pytest \
  src/sm_bringup/test src/sm_task_orchestrator/test \
  src/sm_ee_wholebody_control/test src/sm_natural_language_task/test \
  src/sm_navigation_nav2/test src/sm_base_control_manager/test \
  src/sm_florence_2_vlm_ros2/test src/sm_grasping_ros2/test -q
```

Expected: original 90 behavioral tests plus new contract tests pass.

- [ ] **Step 6: Commit**

```bash
git add README.md ros2_ws/docs ros2_ws/src
git commit -m "refactor: retire legacy ee switch debug package"
```

### Task 6: Final Automated Verification

**Files:**
- Modify: `ros2_ws/docs/architecture/runtime-contract-phase-2.md`

**Interfaces:**
- Consumes: Final eight-package workspace.
- Produces: Reproducible build/test/ownership evidence.

- [ ] **Step 1: Run low-CPU dependency build**

```bash
source /opt/ros/humble/setup.bash
phase2_prefix=$(mktemp -d /tmp/sm-phase2-XXXXXX)
printf '%s\n' "$phase2_prefix" > /tmp/sm-phase2-prefix-path
colcon --log-base "$phase2_prefix/log" build --executor sequential \
  --build-base "$phase2_prefix/build" \
  --install-base "$phase2_prefix/install" \
  --packages-up-to sm_bringup
```

Expected: all eight packages build in a fresh isolated prefix, preventing stale `ee_switch_debug` files in the normal `install/` directory from affecting verification.

- [ ] **Step 2: Run the complete source suite from the isolated overlay**

```bash
phase2_prefix=$(< /tmp/sm-phase2-prefix-path)
source /opt/ros/humble/setup.bash
source "$phase2_prefix/install/setup.bash"
/usr/bin/python3 -m pytest \
  src/sm_bringup/test src/sm_task_orchestrator/test \
  src/sm_ee_wholebody_control/test src/sm_natural_language_task/test \
  src/sm_navigation_nav2/test src/sm_base_control_manager/test \
  src/sm_florence_2_vlm_ros2/test src/sm_grasping_ros2/test -q
```

Record exact pass count, elapsed time, and exit code.

- [ ] **Step 3: Verify installed owners and launch**

```bash
phase2_prefix=$(< /tmp/sm-phase2-prefix-path)
source "$phase2_prefix/install/setup.bash"
ros2 pkg executables sm_task_orchestrator
ros2 pkg executables sm_ee_wholebody_control
ros2 launch sm_bringup natural_language_pick_place.launch.py --show-args
ros2 pkg prefix ee_switch_debug
```

Expected: new owners resolve, launch defaults are unchanged, and the legacy package lookup fails in a fresh overlay.

- [ ] **Step 4: Run ownership/diff checks**

```bash
rg -n "ee_switch_debug" ros2_ws/src README.md \
  --glob '!**/docs/archive/ee_switch_debug/**'
git diff --check
git status --short
```

Expected: no live references/errors and only preserved `ros2_ws/dds_setting/` is untracked.

- [ ] **Step 5: Commit evidence**

```bash
git add ros2_ws/docs/architecture/runtime-contract-phase-2.md
git commit -m "docs: verify modular control package split"
```

### Task 7: Isaac Sim Runtime Regression

**Files:**
- Modify: `ros2_ws/docs/architecture/runtime-contract-phase-2.md`

**Interfaces:**
- Consumes: Active baseline Isaac Sim stage and final `sm_bringup` launch.
- Produces: End-to-end packaging-regression evidence.

- [ ] **Step 1: Confirm bridge rates**

```bash
source /opt/ros/humble/setup.bash
phase2_prefix=$(< /tmp/sm-phase2-prefix-path)
source "$phase2_prefix/install/setup.bash"
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /odom
```

Expected: approximately 33 Hz; do not start the robot stack if absent.

- [ ] **Step 2: Start final bringup**

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
phase2_prefix=$(< /tmp/sm-phase2-prefix-path)
source "$phase2_prefix/install/setup.bash"
ros2 launch sm_bringup natural_language_pick_place.launch.py \
  autostart:=false enable_nav2:=true
```

- [ ] **Step 3: Verify graph and sole velocity owner**

```bash
ros2 node list | sort
ros2 topic info /cmd_vel -v
```

Expected: all capability nodes present; exactly one `/cmd_vel` publisher named `navigation_cmd_mux`.

- [ ] **Step 4: Run consecutive commands**

```bash
ros2 topic pub --once /natural_language_task std_msgs/msg/String \
  "{data: '빨간색 캔을 노란색 박스에 넣어줘'}"
ros2 topic pub --once /natural_language_task std_msgs/msg/String \
  "{data: '오랜지를 노란색 박스에 넣어줘'}"
```

Wait for red-can `DONE` before orange. Expected: identical parser JSON, both `DONE`, and `SAFE_RETREAT`/`RETREAT` before the second remote task.

- [ ] **Step 5: Record and commit runtime evidence**

```bash
git add ros2_ws/docs/architecture/runtime-contract-phase-2.md
git commit -m "docs: close phase two runtime gate"
```

- [ ] **Step 6: Final status**

```bash
phase2_prefix=$(< /tmp/sm-phase2-prefix-path)
case "$phase2_prefix" in
  /tmp/sm-phase2-*) find "$phase2_prefix" -depth -delete ;;
  *) echo "Refusing unexpected temp path: $phase2_prefix"; exit 1 ;;
esac
find /tmp/sm-phase2-prefix-path -maxdepth 0 -delete
git diff --check
git status -sb
```

Expected: clean branch except preserved untracked `ros2_ws/dds_setting/`.
