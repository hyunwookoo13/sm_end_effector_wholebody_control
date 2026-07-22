from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fixed_camera_tf = Node(
        package="ee_switch_debug",
        executable="fixed_camera_tf_publisher",
        name="rsd455_fixed_camera_tf_publisher",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("publish_fixed_camera_tf")),
        parameters=[
            {
                "use_sim_time": True,
                "parent_frame": LaunchConfiguration("camera_parent_frame"),
                "child_frame": LaunchConfiguration("camera_frame"),
                "isaac_x": -0.04447,
                "isaac_y": 3.46295,
                "isaac_z": 1.25048,
                "mapping": "isaac_y_to_ros_x",
                "qx": 0.7071068,
                "qy": 0.7071068,
                "qz": 0.0,
                "qw": 0.0,
                "rate_hz": 30.0,
            }
        ],
    )

    yoloe_node = Node(
        package="sm_florence_2_vlm_ros2",
        executable="sm_yoloe_vlm_node",
        name="sm_yoloe_vlm",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("yoloe_config_file"),
            {
                "use_sim_time": True,
                "camera_frame": LaunchConfiguration("camera_frame"),
                "roi_pointcloud_topic": "/sm_florence_2_vlm/roi_pointcloud_live",
            },
        ],
    )

    perception_snapshot = Node(
        package="ee_switch_debug",
        executable="pick_perception_snapshot",
        name="pick_perception_snapshot",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "live_roi_topic": (
                    "/sm_florence_2_vlm/roi_pointcloud_live"
                ),
                "output_roi_topic": "/sm_florence_2_vlm/roi_pointcloud",
                "live_grasp_topic": "/sm_grasping/grasp_best_live",
                "output_grasp_topic": "/sm_grasping/grasp_best",
                "live_opening_topic": "/sm_grasping/grasp_openings_live",
                "output_opening_topic": "/sm_grasping/grasp_openings",
                "task_command_topic": "/arm_task_command",
                "task_state_topic": "/arm_task_state",
                "grasp_settle_sec": 0.4,
                "publish_rate_hz": 15.0,
            }
        ],
    )

    grasping_node = Node(
        package="sm_grasping_ros2",
        executable="grasping_inference_node",
        name="sm_grasping_inference",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("grasping_config_file"),
            {
                "use_sim_time": True,
                "target_frame": LaunchConfiguration("grasp_parent_frame"),
                "input_roi_pointcloud_topic": "/sm_florence_2_vlm/roi_pointcloud",
                "output_grasp_best_topic": "/sm_grasping/grasp_best_live",
                "output_grasp_openings_topic": "/sm_grasping/grasp_openings_live",
            },
        ],
    )

    marker_node = Node(
        package="sm_grasping_ros2",
        executable="gripper_marker_node",
        name="sm_gripper_marker",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("grasping_config_file"),
            {
                "use_sim_time": True,
                "input_grasp_best_topic": "/sm_grasping/grasp_best",
                "input_grasp_openings_topic": "/sm_grasping/grasp_openings",
            },
        ],
    )

    grasp_tf_bridge = Node(
        package="ee_switch_debug",
        executable="grasp_target_tf_bridge",
        name="grasp_target_tf_bridge",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "use_sim_time": True,
                "input_topic": "/sm_grasping/grasp_best",
                "parent_frame": LaunchConfiguration("grasp_parent_frame"),
                "target_frame": "grasp_position_target",
                "rate_hz": 30.0,
                "timeout_sec": 0.8,
                "latch_target": False,
                "freeze_on_arm_track": True,
                "state_topic": "/arm_safety_state",
                "freeze_on_pick_command": True,
                "task_command_topic": "/arm_task_command",
                "task_state_topic": "/arm_task_state",
                "offset_x": 0.0,
                "offset_y": 0.0,
                "offset_z": 0.0,
            }
        ],
    )

    controller = Node(
        package="ee_switch_debug",
        executable="arm_yaw_rho_z_position_controller",
        name="arm_yaw_rho_z_position_controller",
        output="screen",
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("controller_config_file"),
            {
                "use_sim_time": True,
                "target_frame": "grasp_position_target",
                "enable_base_motion": False,
                "enable_staged_top_down_approach": True,
                "enable_precomputed_top_down_sequence": True,
                "top_down_use_fixed_reachable_orientation": True,
                "top_down_use_group_sequential_approach": True,
                "top_down_blend_orientation_during_descent": False,
                "top_down_minimum_orientation_fraction": 0.50,
                "top_down_orientation_search_steps": 20,
                "top_down_descend_max_xy_deviation": 0.015,
                "top_down_descend_max_orientation_deviation": 0.035,
                "top_down_segment_min_duration": 0.6,
                "top_down_settle_max_command_offset": 0.10,
                "top_down_transit_joint_tolerance": 0.050,
                "top_down_stage_acceleration": 1.6,
                "top_down_lift_home_clearance": 0.06,
                "top_down_alignment_velocity_limits": [0.70, 1.05, 1.20, 0.95, 0.95, 0.95],
                "top_down_path_velocity_limits": [0.42, 0.56, 0.68, 0.56, 0.42, 0.56],
                "top_down_radial_inward_offset": 0.025,
                "top_down_waypoint_spacing": 0.01,
                "top_down_approach_joint_step": 0.12,
                "top_down_minimum_approach_clearance": 0.08,
                "top_down_pregrasp_clearance": ParameterValue(
                    LaunchConfiguration("grasp_pregrasp_clearance_z"), value_type=float
                ),
                "grasp_safe_clearance_z": ParameterValue(
                    LaunchConfiguration("grasp_safe_clearance_z"), value_type=float
                ),
                "grasp_pregrasp_clearance_z": ParameterValue(
                    LaunchConfiguration("grasp_pregrasp_clearance_z"), value_type=float
                ),
                "grasp_final_offset_z": ParameterValue(
                    LaunchConfiguration("grasp_final_offset_z"), value_type=float
                ),
                # One PICK command runs align -> descend -> grasp -> lift -> home.
                "require_descend_confirmation": False,
                "gripper_open_position": 0.0,
                "gripper_close_position": 0.8,
                "grasp_lift_height": 0.12,
                "return_home_after_pick": True,
                # The full top-down wrist solution needs more travel than the
                # conservative generic Pick envelope used by the full mission.
                "pick_max_joint_excursion": [0.65, 2.50, 2.30, 3.00, 3.00, 3.00],
                "wrist_recovery_enter_error_m": 0.08,
                "wrist_recovery_exit_error_m": 0.035,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_frame",
                default_value="rsd455_color_optical_frame",
                description="Pick RGB-D camera optical frame.",
            ),
            DeclareLaunchArgument(
                "camera_parent_frame",
                default_value="world",
                description="Parent frame used only by the optional fixed camera TF.",
            ),
            DeclareLaunchArgument(
                "grasp_parent_frame",
                default_value="chassis_link",
                description="Frame in which the frozen grasp target is controlled.",
            ),
            DeclareLaunchArgument(
                "publish_fixed_camera_tf",
                default_value="false",
                description="Publish the configured fixed world-to-camera TF when Isaac does not publish it.",
            ),
            DeclareLaunchArgument(
                "yoloe_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_florence_2_vlm_ros2"),
                        "config",
                        "sm_yoloe_vlm_rsd455.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "grasping_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sm_grasping_ros2"),
                        "config",
                        "sm_grasping_isaac.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "controller_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("ee_switch_debug"),
                        "config",
                        "arm_position_target_in.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "grasp_safe_clearance_z",
                default_value="0.18",
                description="Clearance used while position and wrist orientation are aligned.",
            ),
            DeclareLaunchArgument(
                "grasp_pregrasp_clearance_z",
                default_value="0.10",
                description="Fixed pre-grasp height used by the six-axis top-down planner.",
            ),
            DeclareLaunchArgument(
                "grasp_final_offset_z",
                default_value="0.0",
                description="Final offset from the grasp marker after vertical descent.",
            ),
            fixed_camera_tf,
            yoloe_node,
            perception_snapshot,
            grasping_node,
            marker_node,
            grasp_tf_bridge,
            controller,
        ]
    )
