import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_active_controller_and_config_are_owned_here():
    module_root = PACKAGE_ROOT / "sm_ee_wholebody_control"
    assert (module_root / "arm_yaw_rho_z_position_controller.py").is_file()
    assert (PACKAGE_ROOT / "config" / "arm_position_target_in.yaml").is_file()


def test_setup_preserves_active_controller_entry_point():
    tree = ast.parse((PACKAGE_ROOT / "setup.py").read_text())
    assert "arm_yaw_rho_z_position_controller" in ast.unparse(tree)
