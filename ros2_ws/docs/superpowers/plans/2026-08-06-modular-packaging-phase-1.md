# Modular Packaging Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the current runtime behavior while extracting natural-language processing, Nav2 integration, and base-command arbitration from `ee_switch_debug` into independently buildable ROS 2 packages.

**Architecture:** This phase performs file and ownership moves only. Existing node names, topic names, action names, parameter defaults, controller values, and the top-level `florence_long_range_pick_place_control.launch.py` entry point remain compatible; the monolithic launch delegates Nav2 startup to `sm_navigation_nav2` and starts the moved nodes from their new packages.

**Tech Stack:** ROS 2 Humble, Python 3, `ament_python`, `rclpy`, ROS 2 launch, Nav2, `pytest`, `colcon`.

## Global Constraints

- Do not change runtime control behavior, controller gains, object aliases, prompts, standoff distances, timeouts, topic names, node names, or QoS settings.
- Keep `sm_grasping_ros2` and `sm_florence_2_vlm_ros2` names unchanged.
- Preserve `ros2_ws/dds_setting/`; do not add, delete, move, or edit it.
- Use sequential or low-parallelism builds to avoid unnecessary CPU load.
- Keep the original top-level launch command working throughout Phase 1.
- Delete legacy source files only in the same commit that installs and verifies their replacement.
- Do not begin whole-body controller or task-orchestrator extraction in this plan.
- Each task must leave the workspace buildable and independently revertible.

---

## Planned File Ownership

### `sm_natural_language_task`

- `sm_natural_language_task/natural_language_task_parser.py`: Gemma/Ollama parsing, validation, aliases, and task publication.
- `sm_natural_language_task/natural_language_task_console.py`: interactive command console.
- `test/test_open_vocabulary_natural_language_parser.py`: parser behavior.
- `test/test_natural_language_color_aliases.py`: alias compatibility.

### `sm_navigation_nav2`

- `launch/nav2_navigation.launch.py`: controller, planner, behavior, BT navigator, and lifecycle nodes.
- `config/nav2_rolling_odom.yaml`: unchanged Nav2 parameters.
- `test/test_nav2_launch_contract.py`: required node/remapping/config contract.

### `sm_base_control_manager`

- `sm_base_control_manager/navigation_cmd_mux.py`: navigation/manipulation/retreat arbitration and final `/cmd_vel` publication.
- `test/test_navigation_cmd_mux.py`: selection, blending, and slew limiting.

### `ee_switch_debug` retained in Phase 1

- `pick_place_task_manager.py`, navigation goal geometry, whole-body controllers, TF bridges, controller configs, and the top-level full-system launch.
- The top-level launch changes only package ownership and Nav2 launch inclusion.

---

### Task 1: Capture and Verify the Working Baseline

**Files:**
- Create: `ros2_ws/docs/architecture/runtime-contract-phase-1.md`
- Reference: `ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py`
- Reference: `ros2_ws/src/ee_switch_debug/setup.py`

**Interfaces:**
- Consumes: Current `ee_switch_debug` launch graph and installed ROS 2 package metadata.
- Produces: A concrete compatibility checklist used by Tasks 2-5.

- [ ] **Step 1: Run the existing unit-test baseline**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
python3 -m pytest src/ee_switch_debug/test src/sm_florence_2_vlm_ros2/test src/sm_grasping_ros2/test -q
```

Expected: all currently collected tests pass. Record the exact passed-test count in the runtime-contract document.

- [ ] **Step 2: Run a low-CPU baseline build**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-select ee_switch_debug sm_florence_2_vlm_ros2 sm_grasping_ros2
```

Expected: all three packages finish successfully.

- [ ] **Step 3: Capture the launch argument contract**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py --show-args
```

Copy the complete argument names and defaults into `runtime-contract-phase-1.md`. At minimum, explicitly record these unchanged defaults:

```text
enable_nav2=false
pick_standoff_m=0.70
place_standoff_m=0.70
hybrid_outer_distance_m=1.40
hybrid_inner_distance_m=0.85
local_llm_model=gemma3:4b
use_local_llm=true
use_rule_task_parser=false
```

- [ ] **Step 4: Write the runtime node/topic contract**

Create `runtime-contract-phase-1.md` with this required table:

```markdown
| Capability | Node name | Executable owner | Output/interface |
|---|---|---|---|
| Language parser | `natural_language_task_parser` | `ee_switch_debug` | `/pick_place_task`, `/natural_language_task_status` |
| Navigation | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | Nav2 via `ee_switch_debug` launch | `/cmd_vel_navigation`, `navigate_to_pose` |
| Base arbitration | `navigation_cmd_mux` | `ee_switch_debug` | `/cmd_vel` |
| Task orchestration | `pick_place_task_manager` | `ee_switch_debug` | `/base_control_mode`, `/base_control_blend` |
| Manipulation | `arm_yaw_rho_z_position_controller` | `ee_switch_debug` | `/joint_position_command`, `/cmd_vel_manipulation` |
```

Add the baseline test count, build result, launch command, and the explicit rule that Phase 1 may change only the `Executable owner` column.

- [ ] **Step 5: Commit the baseline contract**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/docs/architecture/runtime-contract-phase-1.md
git commit -m "docs: record modularization runtime baseline"
```

---

### Task 2: Extract `sm_natural_language_task`

**Files:**
- Create: `ros2_ws/src/sm_natural_language_task/package.xml`
- Create: `ros2_ws/src/sm_natural_language_task/setup.py`
- Create: `ros2_ws/src/sm_natural_language_task/setup.cfg`
- Create: `ros2_ws/src/sm_natural_language_task/resource/sm_natural_language_task`
- Create: `ros2_ws/src/sm_natural_language_task/sm_natural_language_task/__init__.py`
- Move: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py` → `ros2_ws/src/sm_natural_language_task/sm_natural_language_task/natural_language_task_parser.py`
- Move: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_console.py` → `ros2_ws/src/sm_natural_language_task/sm_natural_language_task/natural_language_task_console.py`
- Move: `ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py` → `ros2_ws/src/sm_natural_language_task/test/test_open_vocabulary_natural_language_parser.py`
- Move: `ros2_ws/src/ee_switch_debug/test/test_natural_language_color_aliases.py` → `ros2_ws/src/sm_natural_language_task/test/test_natural_language_color_aliases.py`
- Modify: `ros2_ws/src/ee_switch_debug/setup.py`
- Modify: `ros2_ws/src/ee_switch_debug/package.xml`
- Modify: `ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py`

**Interfaces:**
- Consumes: `/natural_language_task` as `std_msgs/msg/String`; Ollama HTTP endpoint from `ollama_url`.
- Produces: `/pick_place_task` and `/natural_language_task_status` as `std_msgs/msg/String` with payloads identical to the baseline.
- Produces executables: `natural_language_task_parser`, `natural_language_task_console`.

- [ ] **Step 1: Create a failing package-import test**

Create `ros2_ws/src/sm_natural_language_task/test/test_package_imports.py`:

```python
from sm_natural_language_task.natural_language_task_parser import DEFAULT_OLLAMA_MODEL
from sm_natural_language_task.natural_language_task_console import NaturalLanguageTaskConsole


def test_language_package_exports_current_nodes():
    assert DEFAULT_OLLAMA_MODEL == "gemma3:4b"
    assert NaturalLanguageTaskConsole.__name__ == "NaturalLanguageTaskConsole"
```

- [ ] **Step 2: Run the test and verify the new package is absent**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
python3 -m pytest src/sm_natural_language_task/test/test_package_imports.py -q
```

Expected: collection fails with `ModuleNotFoundError: No module named 'sm_natural_language_task'`.

- [ ] **Step 3: Create the package metadata**

Create `package.xml` with exact runtime dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>sm_natural_language_task</name>
  <version>0.1.0</version>
  <description>Natural-language to structured pick-and-place task adapter.</description>
  <maintainer email="gunmin0525@gmail.com">kiro</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <test_depend>python3-pytest</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

Create `setup.py` with these console scripts:

```python
from setuptools import find_packages, setup

package_name = "sm_natural_language_task"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Natural-language to structured pick-and-place task adapter.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "natural_language_task_parser = sm_natural_language_task.natural_language_task_parser:main",
            "natural_language_task_console = sm_natural_language_task.natural_language_task_console:main",
        ],
    },
)
```

Create `setup.cfg`:

```ini
[develop]
script_dir=$base/lib/sm_natural_language_task
[install]
install_scripts=$base/lib/sm_natural_language_task
```

Create an empty resource marker and `__init__.py`.

- [ ] **Step 4: Move the implementation and behavior tests**

Move the two node modules and two behavior-test files listed above. Change only test imports:

```python
from sm_natural_language_task.natural_language_task_parser import (
    DEFAULT_OLLAMA_MODEL,
    NaturalLanguageTaskParser,
)
```

Do not change parser constants, prompts, aliases, validation, timeout behavior, topic defaults, threading, or JSON payload shape.

- [ ] **Step 5: Run all language tests**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
PYTHONPATH=src/sm_natural_language_task python3 -m pytest src/sm_natural_language_task/test -q
```

Expected: the import test and all moved parser tests pass with the same behavior as Task 1.

- [ ] **Step 6: Switch launch ownership and remove duplicate entry points**

In `florence_long_range_pick_place_control.launch.py`, change only:

```python
package="sm_natural_language_task",
executable="natural_language_task_parser",
name="natural_language_task_parser",
```

In `ee_switch_debug/setup.py`, remove the old parser and console console-script lines. Verify no source import still names the old module:

Add this runtime dependency to `ee_switch_debug/package.xml` because its retained full-system launch starts the new package:

```xml
<exec_depend>sm_natural_language_task</exec_depend>
```

```bash
rg -n "ee_switch_debug\.natural_language_task|natural_language_task_(parser|console)" \
  /home/kiro/Desktop/hw_ws/ros2_ws/src
```

Expected: launch references the new package; tests import the new package; no old Python-module import remains.

- [ ] **Step 7: Build and verify executable ownership**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-select sm_natural_language_task ee_switch_debug
source install/setup.bash
ros2 pkg executables sm_natural_language_task
```

Expected executable output contains exactly the two language executables. Run the moved tests again against the installed overlay.

- [ ] **Step 8: Commit the language extraction**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/src/sm_natural_language_task ros2_ws/src/ee_switch_debug
git commit -m "refactor: extract natural language task package"
```

---

### Task 3: Extract the Nav2 Adapter Package

**Files:**
- Create: `ros2_ws/src/sm_navigation_nav2/package.xml`
- Create: `ros2_ws/src/sm_navigation_nav2/setup.py`
- Create: `ros2_ws/src/sm_navigation_nav2/setup.cfg`
- Create: `ros2_ws/src/sm_navigation_nav2/resource/sm_navigation_nav2`
- Create: `ros2_ws/src/sm_navigation_nav2/sm_navigation_nav2/__init__.py`
- Create: `ros2_ws/src/sm_navigation_nav2/launch/nav2_navigation.launch.py`
- Move: `ros2_ws/src/ee_switch_debug/config/nav2_rolling_odom.yaml` → `ros2_ws/src/sm_navigation_nav2/config/nav2_rolling_odom.yaml`
- Create: `ros2_ws/src/sm_navigation_nav2/test/test_nav2_launch_contract.py`
- Modify: `ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/package.xml`
- Test: `ros2_ws/src/ee_switch_debug/test/test_navigation_geometry.py`

**Interfaces:**
- Consumes: `nav2_msgs/action/NavigateToPose`, odometry, TF, maps/scans already configured in `nav2_rolling_odom.yaml`.
- Produces: `/cmd_vel_navigation` and unchanged Nav2 action/status interfaces.
- Does not own: `/cmd_vel`, standoff calculation, Pick/Place state, or precision manipulation.

- [ ] **Step 1: Write a failing Nav2 package contract test**

Create `test_nav2_launch_contract.py`:

```python
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_nav2_launch_owns_required_servers_and_remap():
    launch_text = (PACKAGE_ROOT / "launch" / "nav2_navigation.launch.py").read_text()
    for executable in (
        'executable="controller_server"',
        'executable="planner_server"',
        'executable="behavior_server"',
        'executable="bt_navigator"',
        'executable="lifecycle_manager"',
    ):
        assert executable in launch_text
    assert '("cmd_vel", "/cmd_vel_navigation")' in launch_text


def test_nav2_config_is_installed_from_this_package():
    assert (PACKAGE_ROOT / "config" / "nav2_rolling_odom.yaml").is_file()
```

- [ ] **Step 2: Run the test and verify the launch/config are absent**

Run:

```bash
python3 -m pytest src/sm_navigation_nav2/test/test_nav2_launch_contract.py -q
```

Expected: FAIL because the new launch file and config do not yet exist.

- [ ] **Step 3: Create Nav2 package metadata**

Create an `ament_python` package with version `0.1.0`. Its `package.xml` must contain:

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
<exec_depend>nav2_bt_navigator</exec_depend>
<exec_depend>nav2_behaviors</exec_depend>
<exec_depend>nav2_controller</exec_depend>
<exec_depend>nav2_lifecycle_manager</exec_depend>
<exec_depend>nav2_navfn_planner</exec_depend>
<exec_depend>nav2_planner</exec_depend>
```

The `setup.py` data files must install `launch/*.launch.py` and `config/*.yaml`:

```python
data_files=[
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    (f"share/{package_name}/config", glob("config/*.yaml")),
]
```

This package defines no console scripts in Phase 1.

- [ ] **Step 4: Move the Nav2 configuration without edits**

Move `nav2_rolling_odom.yaml` byte-for-byte and verify:

```bash
git diff --no-index \
  <(git show HEAD:ros2_ws/src/ee_switch_debug/config/nav2_rolling_odom.yaml) \
  ros2_ws/src/sm_navigation_nav2/config/nav2_rolling_odom.yaml
```

Expected: no differences.

- [ ] **Step 5: Create the extracted Nav2 launch**

Create `nav2_navigation.launch.py` with the same five node definitions currently in the full-system launch. The launch must declare only `nav2_params_file`, defaulting to:

```python
PathJoinSubstitution(
    [FindPackageShare("sm_navigation_nav2"), "config", "nav2_rolling_odom.yaml"]
)
```

Preserve these exact node names and remappings:

```python
Node(package="nav2_controller", executable="controller_server", name="controller_server",
     output="screen", parameters=[nav2_params_file],
     remappings=[("cmd_vel", "/cmd_vel_navigation")])
Node(package="nav2_planner", executable="planner_server", name="planner_server",
     output="screen", parameters=[nav2_params_file])
Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
     output="screen", parameters=[nav2_params_file],
     remappings=[("cmd_vel", "/cmd_vel_navigation")])
Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
     output="screen", parameters=[nav2_params_file])
```

The lifecycle manager remains named `lifecycle_manager_navigation`, uses simulation time, autostarts, and manages the four server names in the same order as the baseline.

- [ ] **Step 6: Replace direct Nav2 nodes with a conditional include**

In the full-system launch, import:

```python
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
```

Replace the five direct Nav2 `Node` actions with:

```python
nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare("sm_navigation_nav2"), "launch", "nav2_navigation.launch.py"]
        )
    ),
    condition=nav2_condition,
    launch_arguments={
        "nav2_params_file": LaunchConfiguration("nav2_params_file"),
    }.items(),
)
```

Keep the top-level `enable_nav2` argument and change only the default path of `nav2_params_file` to `sm_navigation_nav2`.

- [ ] **Step 7: Move Nav2 runtime dependencies out of `ee_switch_debug`**

Remove the server-package dependencies listed in Step 3 from `ee_switch_debug/package.xml`. Retain `nav2_msgs` because `pick_place_task_manager` still owns the action client in Phase 1.

Add the retained launch dependency on the adapter:

```xml
<exec_depend>sm_navigation_nav2</exec_depend>
```

- [ ] **Step 8: Run package and compatibility tests**

Run:

```bash
python3 -m pytest src/sm_navigation_nav2/test/test_nav2_launch_contract.py \
  src/ee_switch_debug/test/test_navigation_geometry.py -q
colcon build --executor sequential --packages-select sm_navigation_nav2 ee_switch_debug
source install/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py --show-args
```

Expected: tests pass, build succeeds, and the top-level launch arguments/default values match Task 1.

- [ ] **Step 9: Commit the Nav2 extraction**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/src/sm_navigation_nav2 ros2_ws/src/ee_switch_debug
git commit -m "refactor: extract Nav2 integration package"
```

---

### Task 4: Extract `sm_base_control_manager`

**Files:**
- Create: `ros2_ws/src/sm_base_control_manager/package.xml`
- Create: `ros2_ws/src/sm_base_control_manager/setup.py`
- Create: `ros2_ws/src/sm_base_control_manager/setup.cfg`
- Create: `ros2_ws/src/sm_base_control_manager/resource/sm_base_control_manager`
- Create: `ros2_ws/src/sm_base_control_manager/sm_base_control_manager/__init__.py`
- Move: `ros2_ws/src/ee_switch_debug/ee_switch_debug/navigation_cmd_mux.py` → `ros2_ws/src/sm_base_control_manager/sm_base_control_manager/navigation_cmd_mux.py`
- Move: `ros2_ws/src/ee_switch_debug/test/test_navigation_cmd_mux.py` → `ros2_ws/src/sm_base_control_manager/test/test_navigation_cmd_mux.py`
- Modify: `ros2_ws/src/ee_switch_debug/setup.py`
- Modify: `ros2_ws/src/ee_switch_debug/package.xml`
- Modify: `ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py`

**Interfaces:**
- Consumes: `/cmd_vel_navigation`, `/cmd_vel_manipulation`, `/cmd_vel_retreat`, `/base_control_mode`, `/base_control_blend`.
- Produces: the only final `/cmd_vel` command.
- Preserves pure functions: `selected_source(mode: str) -> str | None`, `smooth_blend_value(navigation: float, manipulation: float, weight: float) -> float`, and `slew_value(current: float, target: float, max_rate: float, dt: float) -> float`.

- [ ] **Step 1: Add a failing import test for the new owner**

Create the new package/test directories, move only `test_navigation_cmd_mux.py` to its planned new path, and change its import before moving the implementation. Use:

```python
from sm_base_control_manager.navigation_cmd_mux import (
    selected_source,
    slew_value,
    smooth_blend_value,
)
```

Run:

```bash
PYTHONPATH=src/sm_base_control_manager python3 -m pytest \
  src/sm_base_control_manager/test/test_navigation_cmd_mux.py -q
```

Expected: collection fails because the new module does not exist yet.

- [ ] **Step 2: Create package metadata**

Create `package.xml` with these runtime dependencies only:

```xml
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

Create `setup.py` with this single executable:

```python
entry_points={
    "console_scripts": [
        "navigation_cmd_mux = sm_base_control_manager.navigation_cmd_mux:main",
    ],
}
```

Use version `0.1.0`, install the resource marker and `package.xml`, and create standard `setup.cfg` paths for `sm_base_control_manager`.

- [ ] **Step 3: Move the mux implementation without logic changes**

Move `navigation_cmd_mux.py` and update only the test import. Do not alter parameters, defaults, topic names, timer rate, timeout behavior, blend equations, acceleration limiting, or stop behavior.

- [ ] **Step 4: Run the moved mux tests**

Run:

```bash
PYTHONPATH=src/sm_base_control_manager python3 -m pytest \
  src/sm_base_control_manager/test/test_navigation_cmd_mux.py -q
```

Expected: all original mux tests pass.

- [ ] **Step 5: Switch launch and entry-point ownership**

In the full-system launch, change only the mux package:

```python
package="sm_base_control_manager",
executable="navigation_cmd_mux",
name="navigation_cmd_mux",
```

Remove the old `navigation_cmd_mux` entry point from `ee_switch_debug/setup.py`. Verify references:

Add this retained launch dependency to `ee_switch_debug/package.xml`:

```xml
<exec_depend>sm_base_control_manager</exec_depend>
```

```bash
rg -n "ee_switch_debug\.navigation_cmd_mux|executable=\"navigation_cmd_mux\"" \
  /home/kiro/Desktop/hw_ws/ros2_ws/src
```

Expected: no old Python import remains and the launch names the new owner.

- [ ] **Step 6: Build and verify the installed executable**

Run:

```bash
colcon build --executor sequential --packages-select sm_base_control_manager ee_switch_debug
source install/setup.bash
ros2 pkg executables sm_base_control_manager
```

Expected: `sm_base_control_manager navigation_cmd_mux` is installed.

- [ ] **Step 7: Commit the base-control extraction**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/src/sm_base_control_manager ros2_ws/src/ee_switch_debug
git commit -m "refactor: extract base control manager package"
```

---

### Task 5: Phase 1 Integration Verification and Cleanup

**Files:**
- Modify: `ros2_ws/docs/architecture/runtime-contract-phase-1.md`
- Modify: `ros2_ws/src/ee_switch_debug/package.xml`
- Modify: `ros2_ws/src/ee_switch_debug/setup.py`
- Inspect: all files under `ros2_ws/src`

**Interfaces:**
- Consumes: packages produced by Tasks 2-4.
- Produces: a buildable five-package runtime graph with unchanged public behavior and recorded verification evidence.

- [ ] **Step 1: Run stale-reference checks**

Run:

```bash
cd /home/kiro/Desktop/hw_ws
rg -n "ee_switch_debug\.(natural_language_task_parser|natural_language_task_console|navigation_cmd_mux)" ros2_ws/src
rg -n "FindPackageShare\(\"ee_switch_debug\"\).*nav2_rolling_odom|ee_switch_debug/config/nav2_rolling_odom" ros2_ws/src
```

Expected: both commands return no matches.

- [ ] **Step 2: Verify package dependency ownership**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
colcon list
rosdep check --from-paths src --ignore-src
```

Expected: `colcon list` includes the three new packages. `rosdep check` reports no missing declared system dependency; if the local machine lacks a declared runtime package, record the exact package name rather than changing dependency declarations to hide it.

- [ ] **Step 3: Run the complete test suite**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
python3 -m pytest \
  src/ee_switch_debug/test \
  src/sm_natural_language_task/test \
  src/sm_navigation_nav2/test \
  src/sm_base_control_manager/test \
  src/sm_florence_2_vlm_ros2/test \
  src/sm_grasping_ros2/test -q
```

Expected: no baseline test is lost and all new contract tests pass.

- [ ] **Step 4: Perform a clean low-CPU workspace build**

Do not delete the user's normal `build/`, `install/`, or `log/` directories. Use isolated output directories:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base log_modular_phase1 build --executor sequential \
  --build-base build_modular_phase1 \
  --install-base install_modular_phase1
```

Expected: every workspace package builds successfully.

- [ ] **Step 5: Verify the installed launch contract**

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/kiro/Desktop/hw_ws/ros2_ws/install_modular_phase1/setup.bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py --show-args
ros2 pkg executables sm_natural_language_task
ros2 pkg executables sm_base_control_manager
```

Expected: launch arguments equal Task 1 and both packages expose the expected executables.

- [ ] **Step 6: Record ownership changes and verification results**

Update the runtime-contract table so only the owner cells change:

```markdown
| Language parser | `natural_language_task_parser` | `sm_natural_language_task` | `/pick_place_task`, `/natural_language_task_status` |
| Navigation | `controller_server`, `planner_server`, `behavior_server`, `bt_navigator` | `sm_navigation_nav2` | `/cmd_vel_navigation`, `navigate_to_pose` |
| Base arbitration | `navigation_cmd_mux` | `sm_base_control_manager` | `/cmd_vel` |
```

Record test count, build command/result, and any environment-only warnings.

- [ ] **Step 7: Run the Isaac Sim end-to-end regression**

With the existing Isaac Sim stage already running, launch the same full-system command used for the baseline, with Nav2 enabled and without changing control values. Verify:

```text
1. Korean natural-language command publishes the same pick/place JSON.
2. YOLOE selects the requested object.
3. Nav2 publishes through /cmd_vel_navigation.
4. navigation_cmd_mux is the only final /cmd_vel publisher.
5. Red-can to yellow-box completes at baseline behavior.
6. Orange to yellow-box completes at baseline behavior.
7. A consecutive remote task performs the existing safe-retreat behavior.
```

Capture the relevant ROS log lines in `runtime-contract-phase-1.md`. Apple behavior is recorded but is not required to improve in this packaging phase.

- [ ] **Step 8: Commit Phase 1 verification evidence**

```bash
cd /home/kiro/Desktop/hw_ws
git add ros2_ws/docs/architecture/runtime-contract-phase-1.md \
  ros2_ws/src/ee_switch_debug/package.xml \
  ros2_ws/src/ee_switch_debug/setup.py
git commit -m "docs: verify modular packaging phase one"
```

- [ ] **Step 9: Confirm the tree preserves user-owned files**

Run:

```bash
git status --short
git log -6 --oneline --decorate
```

Expected: `ros2_ws/dds_setting/` remains present and unmodified. Do not add it to any commit.

---

## Follow-up Plans

After Phase 1 is verified in Isaac Sim, create separate implementation plans for:

1. `sm_wholebody_control` extraction and controller/debug-tool classification.
2. `sm_pick_place_orchestrator` extraction and stable task/navigation interfaces.
3. `sm_system_bringup` launch profiles and compatibility launch deprecation.
4. Legacy `ee_switch_debug` removal, unused-node deletion, documentation consolidation, and final workspace cleanup.

Each follow-up plan must preserve the same verification gates and use deletion-only commits after replacement behavior is proven.
