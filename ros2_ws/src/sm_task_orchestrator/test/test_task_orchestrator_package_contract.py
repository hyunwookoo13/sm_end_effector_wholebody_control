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
