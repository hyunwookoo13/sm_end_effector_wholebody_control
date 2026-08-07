from sm_task_orchestrator.pick_place_task_manager import class_aliases_for as task_aliases_for
from sm_florence_2_vlm.florence_2_vlm_node import (
    Florence2Detector,
    class_aliases_for as florence_aliases_for,
)


def test_colored_box_targets_include_tray_aliases():
    assert "tray" in task_aliases_for("yellow box")
    assert "tray" in florence_aliases_for("yellow box")


def test_florence_prompt_expands_colored_box_targets():
    prompt = Florence2Detector.build_prompt(["yellow box"])

    assert "yellow box" in prompt
    assert "yellow tray" in prompt
    assert "tray" in prompt
