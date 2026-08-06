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
