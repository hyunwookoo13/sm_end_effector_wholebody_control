from glob import glob

from setuptools import setup

package_name = "ee_switch_debug"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README_PICK_PLACE.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="Debug node for reading target frames relative to the mobile base.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_yaw_velocity_controller = ee_switch_debug.arm_yaw_velocity_controller:main",
            "arm_yaw_rho_z_position_controller = ee_switch_debug.arm_yaw_rho_z_position_controller:main",
            "arm_yaw_rho_z_velocity_controller = ee_switch_debug.arm_yaw_rho_z_velocity_controller:main",
            "arm_joint_effect_probe = ee_switch_debug.arm_joint_effect_probe:main",
            "arm_rho_z_velocity_controller = ee_switch_debug.arm_rho_z_velocity_controller:main",
            "arm_z_velocity_controller = ee_switch_debug.arm_z_velocity_controller:main",
            "florence_target_tf_bridge = ee_switch_debug.florence_target_tf_bridge:main",
            "fixed_camera_tf_publisher = ee_switch_debug.fixed_camera_tf_publisher:main",
            "grasp_target_tf_bridge = ee_switch_debug.grasp_target_tf_bridge:main",
            "l100_pointer_target = ee_switch_debug.l100_pointer_target:main",
            "live_yaw_rho_z_monitor = ee_switch_debug.live_yaw_rho_z_monitor:main",
            "mobile_target_controller = ee_switch_debug.mobile_target_controller:main",
            "natural_language_task_console = ee_switch_debug.natural_language_task_console:main",
            "natural_language_task_parser = ee_switch_debug.natural_language_task_parser:main",
            "navigation_cmd_mux = ee_switch_debug.navigation_cmd_mux:main",
            "pick_perception_snapshot = ee_switch_debug.pick_perception_snapshot:main",
            "pick_place_task_manager = ee_switch_debug.pick_place_task_manager:main",
            "target_arm_ik_controller = ee_switch_debug.target_arm_ik_controller:main",
            "target_base_controller = ee_switch_debug.target_base_controller:main",
            "target_frame_debug = ee_switch_debug.target_frame_debug:main",
            "wholebody_yaw_rho_z_controller = ee_switch_debug.wholebody_yaw_rho_z_controller:main",
            "target_wholebody_controller = ee_switch_debug.target_wholebody_controller:main",
            "wholebody_debug_logger = ee_switch_debug.wholebody_debug_logger:main",
            "plot_wholebody_debug = ee_switch_debug.plot_wholebody_debug:main",
            "plot_yaw_rho_z_debug = ee_switch_debug.plot_yaw_rho_z_debug:main",
        ],
    },
)
