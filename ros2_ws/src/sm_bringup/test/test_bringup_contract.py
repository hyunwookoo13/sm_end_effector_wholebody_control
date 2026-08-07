from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_manifest_depends_on_runtime_packages():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    dependencies = {node.text for node in root.findall("exec_depend")}
    assert {
        "sm_base_control_manager",
        "sm_ee_wholebody_control",
        "sm_florence_2_vlm_ros2",
        "sm_grasping_ros2",
        "sm_natural_language_task",
        "sm_navigation_nav2",
        "sm_task_orchestrator",
    } <= dependencies


def test_canonical_launch_has_new_owners_only():
    launch_text = (
        PACKAGE_ROOT / "launch" / "natural_language_pick_place.launch.py"
    ).read_text()
    assert 'package="sm_task_orchestrator"' in launch_text
    assert 'package="sm_ee_wholebody_control"' in launch_text
    assert "ee_switch_debug" not in launch_text
