from pathlib import Path


def test_dual_rsd455_launch_starts_both_yoloe_nodes():
    launch_file = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "sm_yoloe_vlm_dual_rsd455.launch.py"
    )
    source = launch_file.read_text()

    assert "sm_yoloe_vlm_rsd455.yaml" in source
    assert "sm_yoloe_vlm_rsd455_place.yaml" in source
    assert source.count('executable="sm_yoloe_vlm_node"') == 2
    assert 'name="sm_yoloe_vlm"' in source
    assert 'name="sm_yoloe_vlm_place"' in source
