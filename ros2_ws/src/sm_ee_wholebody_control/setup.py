from glob import glob

from setuptools import setup


package_name = "sm_ee_wholebody_control"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiro",
    maintainer_email="gunmin0525@gmail.com",
    description="End-effector-targeted arm and mobile-base whole-body control.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_yaw_velocity_controller = sm_ee_wholebody_control.arm_yaw_velocity_controller:main",
            "arm_yaw_rho_z_position_controller = sm_ee_wholebody_control.arm_yaw_rho_z_position_controller:main",
            "arm_yaw_rho_z_velocity_controller = sm_ee_wholebody_control.arm_yaw_rho_z_velocity_controller:main",
            "arm_joint_effect_probe = sm_ee_wholebody_control.arm_joint_effect_probe:main",
            "arm_rho_z_velocity_controller = sm_ee_wholebody_control.arm_rho_z_velocity_controller:main",
            "arm_z_velocity_controller = sm_ee_wholebody_control.arm_z_velocity_controller:main",
            "florence_target_tf_bridge = sm_ee_wholebody_control.florence_target_tf_bridge:main",
            "fixed_camera_tf_publisher = sm_ee_wholebody_control.fixed_camera_tf_publisher:main",
            "grasp_target_tf_bridge = sm_ee_wholebody_control.grasp_target_tf_bridge:main",
            "l100_pointer_target = sm_ee_wholebody_control.l100_pointer_target:main",
            "live_yaw_rho_z_monitor = sm_ee_wholebody_control.live_yaw_rho_z_monitor:main",
            "mobile_target_controller = sm_ee_wholebody_control.mobile_target_controller:main",
            "target_arm_ik_controller = sm_ee_wholebody_control.target_arm_ik_controller:main",
            "target_base_controller = sm_ee_wholebody_control.target_base_controller:main",
            "target_frame_debug = sm_ee_wholebody_control.target_frame_debug:main",
            "wholebody_yaw_rho_z_controller = sm_ee_wholebody_control.wholebody_yaw_rho_z_controller:main",
            "target_wholebody_controller = sm_ee_wholebody_control.target_wholebody_controller:main",
            "wholebody_debug_logger = sm_ee_wholebody_control.wholebody_debug_logger:main",
            "plot_wholebody_debug = sm_ee_wholebody_control.plot_wholebody_debug:main",
            "plot_yaw_rho_z_debug = sm_ee_wholebody_control.plot_yaw_rho_z_debug:main",
        ],
    },
)
