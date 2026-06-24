from math import acos, atan2, cos, hypot, pi, sin, tanh

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


def quaternion_to_matrix(q) -> list[list[float]]:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    norm = (x*x + y*y + z*z + w*w)**0.5
    if norm > 1e-9:
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
    else:
        x, y, z, w = 0.0, 0.0, 0.0, 1.0
    return [
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - w*z), 2.0*(x*z + w*y)],
        [2.0*(x*y + w*z), 1.0 - 2.0*(x*x + z*z), 2.0*(y*z - w*x)],
        [2.0*(x*z - w*y), 2.0*(y*z + w*x), 1.0 - 2.0*(x*x + y*y)]
    ]


def transpose_matrix(M: list[list[float]]) -> list[list[float]]:
    return [
        [M[0][0], M[1][0], M[2][0]],
        [M[0][1], M[1][1], M[2][1]],
        [M[0][2], M[1][2], M[2][2]]
    ]


def multiply_matrices(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    C = [[0.0]*3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            C[i][j] = sum(A[i][k] * B[k][j] for k in range(3))
    return C


def decompose_yzy(R: list[list[float]]) -> tuple[float, float, float]:
    r11, r12, r13 = R[0][0], R[0][1], R[0][2]
    r21, r22, r23 = R[1][0], R[1][1], R[1][2]
    r31, r32, r33 = R[2][0], R[2][1], R[2][2]
    
    cos_beta = clamp(r22, -1.0, 1.0)
    beta = acos(cos_beta)
    sin_beta = sin(beta)
    
    if abs(sin_beta) > 1e-6:
        alpha = atan2(r32, -r12)
        gamma = atan2(r23, r21)
    else:
        alpha = 0.0
        if cos_beta > 0.0:
            gamma = atan2(r13, r11)
        else:
            gamma = atan2(-r13, -r11)
            
    return alpha, beta, gamma


class ArmYawRhoZPositionController(Node):
    def __init__(self) -> None:
        super().__init__("arm_yaw_rho_z_position_controller")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("base_frame", "chassis_control_frame")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_position_topic", "/joint_position_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("state_topic", "/arm_safety_state")
        self.declare_parameter("task_command_topic", "/arm_task_command")
        self.declare_parameter("task_state_topic", "/arm_task_state")
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("yaw_error_source", "joint")
        self.declare_parameter("joint1_sign", 1.0)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)
        self.declare_parameter("joint2_rho_per_rad", 0.2715)
        self.declare_parameter("joint2_z_per_rad", -0.1972)
        self.declare_parameter("joint3_rho_per_rad", 0.1521)
        self.declare_parameter("joint3_z_per_rad", -0.2631)
        self.declare_parameter("k_yaw", 1.5)
        self.declare_parameter("k_rho", 1.0)
        self.declare_parameter("k_z", 1.5)
        self.declare_parameter("max_yaw_velocity", 0.5)
        self.declare_parameter("max_rho_velocity", 0.04)
        self.declare_parameter("max_z_velocity", 0.04)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("max_joint2_velocity", 0.4)
        self.declare_parameter("max_joint3_velocity", 0.4)
        self.declare_parameter("max_joint_acceleration", 0.8)
        self.declare_parameter("approach_slowdown_distance", 0.15)
        self.declare_parameter("approach_min_scale", 0.15)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("rho_tolerance", 0.04)
        self.declare_parameter("z_tolerance", 0.04)
        self.declare_parameter("singularity_epsilon", 0.01)
        self.declare_parameter(
            "home_positions",
            [0.0, -1.5708, 2.0944, -0.5236, 1.5708, 0.0],
        )
        self.declare_parameter("home_kp", 2.0)
        self.declare_parameter("home_max_joint_velocity", 1.0)
        self.declare_parameter("home_position_tolerance", 0.03)
        self.declare_parameter("safety_enter_rho_min", 0.20)
        self.declare_parameter("safety_enter_rho_max", 0.55)
        self.declare_parameter("safety_exit_rho_min", 0.15)
        self.declare_parameter("safety_exit_rho_max", 0.65)
        self.declare_parameter("safety_enter_z_min", 0.35)
        self.declare_parameter("safety_enter_z_max", 0.78)
        self.declare_parameter("safety_exit_z_min", 0.30)
        self.declare_parameter("safety_exit_z_max", 0.85)
        self.declare_parameter("safety_enter_yaw_max", 1.0472)
        self.declare_parameter("safety_exit_yaw_max", 1.3090)
        self.declare_parameter("base_stop_rho", 0.50)
        self.declare_parameter("switching_rho", 0.55)
        self.declare_parameter("switching_alpha", 10.0)
        self.declare_parameter("arm_switching_rho", 0.75)
        self.declare_parameter("arm_switching_alpha", 10.0)
        self.declare_parameter("arm_blend_exponent", 1.0)
        self.declare_parameter("base_yaw_tolerance", 0.08)
        self.declare_parameter("k_base_yaw", 2.0)
        self.declare_parameter("k_base_linear", 0.8)
        self.declare_parameter("max_base_yaw_rate", 0.6)
        self.declare_parameter("max_base_linear", 0.20)
        self.declare_parameter("enable_base_motion", False)
        self.declare_parameter(
            "joint_lower_limits",
            [-3.1416, -3.1416, -3.1416, -3.1416, -3.1416, -3.1416],
        )
        self.declare_parameter(
            "joint_upper_limits",
            [3.1416, 3.1416, 3.1416, 3.1416, 3.1416, 3.1416],
        )
        self.declare_parameter("k_wrist", 2.0)
        self.declare_parameter("max_wrist_velocity", 1.0)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)

        # Grasp sequence parameters
        self.declare_parameter("grasp_descend_speed", 0.05)
        self.declare_parameter("grasp_lift_speed", 0.08)
        self.declare_parameter("grasp_lift_height", 0.12)
        self.declare_parameter("grasp_descend_depth", 0.08)
        self.declare_parameter("grasp_close_duration", 1.0)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", 0.8)
        self.declare_parameter("grasp_offset_z", 0.08)
        self.declare_parameter("place_offset_z", 0.10)
        self.declare_parameter("place_descend_depth", 0.08)
        self.declare_parameter("place_open_duration", 0.5)
        self.declare_parameter("return_home_after_place", True)
        self.declare_parameter("gripper_joint_names", ["rh_l1", "rh_r1_joint"])

        self.arm_base_frame = str(self.get_parameter("arm_base_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.link1 = float(self.get_parameter("link1").value)
        self.link2 = float(self.get_parameter("link2").value)
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        joint_position_topic = str(self.get_parameter("joint_position_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        task_command_topic = str(self.get_parameter("task_command_topic").value)
        task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.yaw_error_source = str(self.get_parameter("yaw_error_source").value)
        self.joint1_sign = float(self.get_parameter("joint1_sign").value)
        self.joint2_sign = float(self.get_parameter("joint2_sign").value)
        self.joint3_sign = float(self.get_parameter("joint3_sign").value)
        self.joint2_rho_per_rad = float(self.get_parameter("joint2_rho_per_rad").value)
        self.joint2_z_per_rad = float(self.get_parameter("joint2_z_per_rad").value)
        self.joint3_rho_per_rad = float(self.get_parameter("joint3_rho_per_rad").value)
        self.joint3_z_per_rad = float(self.get_parameter("joint3_z_per_rad").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_z = float(self.get_parameter("k_z").value)
        self.max_yaw_velocity = float(self.get_parameter("max_yaw_velocity").value)
        self.max_rho_velocity = float(self.get_parameter("max_rho_velocity").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.max_joint2_velocity = float(
            self.get_parameter("max_joint2_velocity").value
        )
        self.max_joint3_velocity = float(
            self.get_parameter("max_joint3_velocity").value
        )
        self.max_joint_acceleration = float(self.get_parameter("max_joint_acceleration").value)
        self.approach_slowdown_distance = max(
            float(self.get_parameter("approach_slowdown_distance").value),
            0.001,
        )
        self.approach_min_scale = clamp(
            float(self.get_parameter("approach_min_scale").value),
            0.0,
            1.0,
        )
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.rho_tolerance = float(self.get_parameter("rho_tolerance").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.singularity_epsilon = float(self.get_parameter("singularity_epsilon").value)
        self.home_positions = list(self.get_parameter("home_positions").value)
        self.home_kp = float(self.get_parameter("home_kp").value)
        self.home_max_joint_velocity = float(
            self.get_parameter("home_max_joint_velocity").value
        )
        self.home_position_tolerance = float(
            self.get_parameter("home_position_tolerance").value
        )
        self.safety_enter_rho_min = float(
            self.get_parameter("safety_enter_rho_min").value
        )
        self.safety_enter_rho_max = float(
            self.get_parameter("safety_enter_rho_max").value
        )
        self.safety_exit_rho_min = float(
            self.get_parameter("safety_exit_rho_min").value
        )
        self.safety_exit_rho_max = float(
            self.get_parameter("safety_exit_rho_max").value
        )
        self.safety_enter_z_min = float(
            self.get_parameter("safety_enter_z_min").value
        )
        self.safety_enter_z_max = float(
            self.get_parameter("safety_enter_z_max").value
        )
        self.safety_exit_z_min = float(
            self.get_parameter("safety_exit_z_min").value
        )
        self.safety_exit_z_max = float(
            self.get_parameter("safety_exit_z_max").value
        )
        self.safety_enter_yaw_max = float(
            self.get_parameter("safety_enter_yaw_max").value
        )
        self.safety_exit_yaw_max = float(
            self.get_parameter("safety_exit_yaw_max").value
        )
        self.base_stop_rho = float(self.get_parameter("base_stop_rho").value)
        self.switching_rho = float(self.get_parameter("switching_rho").value)
        self.switching_alpha = float(
            self.get_parameter("switching_alpha").value
        )
        self.arm_switching_rho = float(
            self.get_parameter("arm_switching_rho").value
        )
        self.arm_switching_alpha = float(
            self.get_parameter("arm_switching_alpha").value
        )
        self.arm_blend_exponent = max(
            float(self.get_parameter("arm_blend_exponent").value),
            0.01,
        )
        self.base_yaw_tolerance = float(
            self.get_parameter("base_yaw_tolerance").value
        )
        self.k_base_yaw = float(self.get_parameter("k_base_yaw").value)
        self.k_base_linear = float(self.get_parameter("k_base_linear").value)
        self.max_base_yaw_rate = float(
            self.get_parameter("max_base_yaw_rate").value
        )
        self.max_base_linear = float(self.get_parameter("max_base_linear").value)
        self.enable_base_motion = bool(
            self.get_parameter("enable_base_motion").value
        )
        self.joint_lower_limits = list(self.get_parameter("joint_lower_limits").value)
        self.joint_upper_limits = list(self.get_parameter("joint_upper_limits").value)
        self.k_wrist = float(self.get_parameter("k_wrist").value)
        self.max_wrist_velocity = float(self.get_parameter("max_wrist_velocity").value)

        # Grasp sequence parameters
        self.grasp_descend_speed = float(self.get_parameter("grasp_descend_speed").value)
        self.grasp_lift_speed = float(self.get_parameter("grasp_lift_speed").value)
        self.grasp_lift_height = float(self.get_parameter("grasp_lift_height").value)
        self.grasp_descend_depth = max(
            0.0,
            float(self.get_parameter("grasp_descend_depth").value),
        )
        self.grasp_close_duration = float(self.get_parameter("grasp_close_duration").value)
        self.gripper_open_position = float(self.get_parameter("gripper_open_position").value)
        self.gripper_close_position = float(self.get_parameter("gripper_close_position").value)
        self.grasp_offset_z = float(self.get_parameter("grasp_offset_z").value)
        self.place_offset_z = float(self.get_parameter("place_offset_z").value)
        self.place_descend_depth = max(
            0.0,
            float(self.get_parameter("place_descend_depth").value),
        )
        self.place_open_duration = max(
            0.0,
            float(self.get_parameter("place_open_duration").value),
        )
        self.return_home_after_place = bool(
            self.get_parameter("return_home_after_place").value
        )
        self.gripper_joint_names = list(self.get_parameter("gripper_joint_names").value)

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}
        self.position_command: list[float] | None = None
        self.previous_velocity = [0.0] * 6
        self.control_state: str | None = None
        self.target_received = False
        self.last_update_time = None
        self.last_debug = "waiting for joint states"

        # Grasp sequence state
        self.task_mode = "PICK"
        self.force_safety_pose = False
        self.grasp_phase = "APPROACH"  # APPROACH, DESCEND, GRASP, LIFT, HOLD
        self.grasp_z_offset = self.grasp_offset_z  # current dynamic z offset
        self.grasp_phase_start_time = None
        self.gripper_position = self.gripper_open_position
        self.lift_z_accumulated = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(String, task_command_topic, self.on_task_command, 10)
        self.position_pub = self.create_publisher(JointState, joint_position_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.task_state_pub = self.create_publisher(String, task_state_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(self.rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)

        self.get_logger().info(
            f"Arm position controller: {self.arm_base_frame} -> {self.target_frame}, "
            f"base frame={self.base_frame}, publishing {joint_position_topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))
        if self.position_command is None and all(name in self.current_joints for name in self.joint_names):
            self.position_command = [self.current_joints[name] for name in self.joint_names]
            self.last_update_time = self.get_clock().now()
            self.get_logger().info("Position command initialized from current joint states")

    def on_task_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "PICK":
            self.start_pick_sequence()
        elif command == "PLACE":
            self.start_place_sequence()
        elif command == "RESET":
            self.start_pick_sequence()
            self.set_control_state("RETURN_HOME")
        else:
            self.get_logger().warn(f"Unknown arm task command: {msg.data!r}")

    def start_pick_sequence(self) -> None:
        self.task_mode = "PICK"
        self.force_safety_pose = False
        self.grasp_phase = "APPROACH"
        self.grasp_z_offset = self.grasp_offset_z
        self.grasp_phase_start_time = None
        self.gripper_position = self.gripper_open_position
        self.lift_z_accumulated = 0.0
        self.previous_velocity = [0.0] * 6
        self.publish_task_state()
        self.get_logger().warn("Arm task command: PICK")

    def start_place_sequence(self) -> None:
        self.task_mode = "PLACE"
        self.force_safety_pose = False
        self.grasp_phase = "APPROACH"
        self.grasp_z_offset = self.place_offset_z
        self.grasp_phase_start_time = None
        self.gripper_position = self.gripper_close_position
        self.lift_z_accumulated = 0.0
        self.previous_velocity = [0.0] * 6
        self.publish_task_state()
        self.get_logger().warn("Arm task command: PLACE")

    def publish_task_state(self) -> None:
        self.task_state_pub.publish(String(data=f"{self.task_mode}:{self.grasp_phase}"))

    def on_timer(self) -> None:
        if self.position_command is None or self.last_update_time is None:
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.2:
            self.previous_velocity = [0.0] * 6
            self.cmd_pub.publish(Twist())
            self.publish_position_command()
            return

        try:
            target_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            ee_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            base_target_transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            link3_transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                "link3",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.set_control_state("RETURN_HOME")
            if self.target_received:
                desired_velocity = self.compute_home_velocities()
            else:
                desired_velocity = [0.0] * 6
            limited_velocity = self.limit_acceleration(desired_velocity, dt)
            self.integrate_position_command(limited_velocity, dt)
            self.cmd_pub.publish(Twist())
            self.publish_position_command()
            self.last_debug = f"RETURN_HOME; base stopped; TF lookup failed: {exc}"
            return

        self.target_received = True
        target = target_transform.transform.translation
        ee = ee_transform.transform.translation

        R_target = quaternion_to_matrix(target_transform.transform.rotation)
        R_ee = quaternion_to_matrix(ee_transform.transform.rotation)

        offset_local = [0.008493, 0.017565, 0.0]
        # Current wrist center position in link0 frame
        wc_x = ee.x - (R_ee[0][0]*offset_local[0] + R_ee[0][1]*offset_local[1] + R_ee[0][2]*offset_local[2])
        wc_y = ee.y - (R_ee[1][0]*offset_local[0] + R_ee[1][1]*offset_local[1] + R_ee[1][2]*offset_local[2])
        wc_z = ee.z - (R_ee[2][0]*offset_local[0] + R_ee[2][1]*offset_local[1] + R_ee[2][2]*offset_local[2])

        # Target wrist center position in link0 frame
        wc_target_x = target.x - (R_target[0][0]*offset_local[0] + R_target[0][1]*offset_local[1] + R_target[0][2]*offset_local[2])
        wc_target_y = target.y - (R_target[1][0]*offset_local[0] + R_target[1][1]*offset_local[1] + R_target[1][2]*offset_local[2])
        wc_target_z = target.z - (R_target[2][0]*offset_local[0] + R_target[2][1]*offset_local[1] + R_target[2][2]*offset_local[2])

        # Dynamic pre-grasp/descent/lift offset relative to the detected grasp target.
        wc_target_z = wc_target_z + self.grasp_z_offset

        base_target = base_target_transform.transform.translation
        target_yaw = atan2(wc_target_y, wc_target_x)
        target_rho = hypot(wc_target_x, wc_target_y)
        ee_yaw = atan2(wc_y, wc_x)
        
        joint_yaw_error = normalize_angle(
            target_yaw - self.joint1_sign * self.current_joints["joint1"]
        )
        ee_yaw_error = normalize_angle(target_yaw - ee_yaw)
        current_rho = hypot(wc_x, wc_y)
        if self.yaw_error_source == "ee" and current_rho >= 0.25:
            yaw_error = ee_yaw_error
        else:
            yaw_error = joint_yaw_error
        rho_error = target_rho - hypot(wc_x, wc_y)
        z_error = wc_target_z - wc_z

        self.update_control_state(target_rho, wc_target_z, target_yaw)

        # Check alignment of joints 1, 2, 3
        pos_aligned = (
            abs(yaw_error) <= self.yaw_tolerance
            and abs(rho_error) <= self.rho_tolerance
            and abs(z_error) <= self.z_tolerance
        )

        # Simple wrist control: keep joints 4,5 at home, always rotate joint 6 by -90 deg
        q4_des = self.home_positions[3]
        q5_des = self.home_positions[4]
        q6_des = -1.5708  # -90 degrees to orient gripper for grasping

        # ── Pick/place sequence state machine ──
        self._update_task_phase(pos_aligned, dt)
        self.publish_task_state()

        q4_curr = self.current_joints["joint4"]
        q5_curr = self.current_joints["joint5"]
        q6_curr = self.current_joints["joint6"]
        
        e4 = normalize_angle(q4_des - q4_curr)
        e5 = normalize_angle(q5_des - q5_curr)
        e6 = normalize_angle(q6_des - q6_curr)
        
        joint4_velocity = clamp(self.k_wrist * e4, -self.max_wrist_velocity, self.max_wrist_velocity)
        joint5_velocity = clamp(self.k_wrist * e5, -self.max_wrist_velocity, self.max_wrist_velocity)
        joint6_velocity = clamp(self.k_wrist * e6, -self.max_wrist_velocity, self.max_wrist_velocity)
        wrist_velocities = [joint4_velocity, joint5_velocity, joint6_velocity]

        if self.control_state == "RETURN_HOME":
            home_velocity = self.compute_home_velocities()
            if self.force_safety_pose:
                limited_velocity = self.limit_acceleration(home_velocity, dt)
                self.integrate_position_command(limited_velocity, dt)
                self.cmd_pub.publish(Twist())
                self.publish_position_command()
                home_error = max(
                    abs(self.home_positions[index] - self.current_joints[self.joint_names[index]])
                    for index in range(6)
                )
                self.last_debug = (
                    f"MISSION_DONE_RETURN_HOME home_error={home_error:.3f}, "
                    f"qdot=[{limited_velocity[0]:.3f}, {limited_velocity[1]:.3f}, "
                    f"{limited_velocity[2]:.3f}], base stopped"
                )
                return

            # Force wrist back to home during RETURN_HOME
            q4_des = self.home_positions[3]
            q5_des = self.home_positions[4]
            q6_des = self.home_positions[5]
            e4 = normalize_angle(q4_des - q4_curr)
            e5 = normalize_angle(q5_des - q5_curr)
            e6 = normalize_angle(q6_des - q6_curr)
            home_wrist_vels = [
                clamp(self.k_wrist * e4, -self.max_wrist_velocity, self.max_wrist_velocity),
                clamp(self.k_wrist * e5, -self.max_wrist_velocity, self.max_wrist_velocity),
                clamp(self.k_wrist * e6, -self.max_wrist_velocity, self.max_wrist_velocity)
            ]
            tracking_velocity = self.compute_joint_velocities(
                yaw_error,
                rho_error,
                z_error,
                self.current_joints["joint2"],
                self.current_joints["joint3"],
            )
            tracking_velocity.extend(home_wrist_vels)
            mu = self.compute_switching(
                target_rho,
                self.switching_rho,
                self.switching_alpha,
            )
            arm_mu = self.compute_switching(
                target_rho,
                self.arm_switching_rho,
                self.arm_switching_alpha,
            )
            arm_blend = (
                arm_mu ** self.arm_blend_exponent
                if self.is_transition_target_safe(wc_target_z, target_yaw)
                else 0.0
            )
            desired_velocity = [
                (1.0 - arm_blend) * home + arm_blend * tracking
                for home, tracking in zip(home_velocity, tracking_velocity)
            ]
            limited_velocity = self.limit_acceleration(desired_velocity, dt)
            self.integrate_position_command(limited_velocity, dt)
            base_scale = 1.0 - mu
            twist = self.compute_base_twist(
                base_target.x,
                base_target.y,
                target_rho,
                base_scale,
            )
            self.cmd_pub.publish(twist)
            self.publish_position_command()
            home_error = max(
                abs(self.home_positions[index] - self.current_joints[self.joint_names[index]])
                for index in range(6)
            )
            self.last_debug = (
                f"RETURN_HOME target=[rho {target_rho:.3f}, yaw {target_yaw:.3f}, "
                f"z {wc_target_z:.3f}], home_error={home_error:.3f}, "
                f"mu={mu:.3f}, arm_mu={arm_mu:.3f}, "
                f"arm_blend={arm_blend:.3f}, "
                f"base_scale={base_scale:.3f}, "
                f"base=[vx {twist.linear.x:.3f}, wz {twist.angular.z:.3f}]"
            )
            return

        self.cmd_pub.publish(Twist())

        desired_velocity = self.compute_joint_velocities(
            yaw_error,
            rho_error,
            z_error,
            self.current_joints["joint2"],
            self.current_joints["joint3"],
        )
        desired_velocity.extend(wrist_velocities)
        limited_velocity = self.limit_acceleration(desired_velocity, dt)
        self.integrate_position_command(limited_velocity, dt)

        self.publish_position_command()
        self.last_debug = (
            f"ARM_TRACK target=[rho {target_rho:.3f}, yaw {target_yaw:.3f}, "
            f"z {wc_target_z:.3f}], error=[yaw {yaw_error:.3f}, "
            f"rho {rho_error:.3f}, z {z_error:.3f}], "
            f"qdot=[{limited_velocity[0]:.3f}, {limited_velocity[1]:.3f}, "
            f"{limited_velocity[2]:.3f}], qcmd=[{self.position_command[0]:.3f}, "
            f"{self.position_command[1]:.3f}, {self.position_command[2]:.3f}], "
            f"task={self.task_mode}, phase={self.grasp_phase}, "
            f"z_off={self.grasp_z_offset:.3f}, "
            f"grip={self.gripper_position:.2f}"
        )

    def update_control_state(
        self,
        target_rho: float,
        target_z: float,
        target_yaw: float,
    ) -> None:
        if self.force_safety_pose:
            self.set_control_state("RETURN_HOME")
            return

        inside_enter = (
            self.safety_enter_rho_min <= target_rho <= self.safety_enter_rho_max
            and self.safety_enter_z_min <= target_z <= self.safety_enter_z_max
            and abs(target_yaw) <= self.safety_enter_yaw_max
        )
        inside_exit = (
            self.safety_exit_rho_min <= target_rho <= self.safety_exit_rho_max
            and self.safety_exit_z_min <= target_z <= self.safety_exit_z_max
            and abs(target_yaw) <= self.safety_exit_yaw_max
        )

        if self.control_state is None:
            self.set_control_state("ARM_TRACK" if inside_enter else "RETURN_HOME")
        elif self.control_state == "ARM_TRACK" and not inside_exit:
            self.set_control_state("RETURN_HOME")
        elif self.control_state == "RETURN_HOME" and inside_enter:
            self.set_control_state("ARM_TRACK")

    def set_control_state(self, state: str) -> None:
        if self.control_state == state:
            return
        previous = self.control_state or "UNINITIALIZED"
        self.control_state = state
        self.previous_velocity = [0.0] * 6
        self.state_pub.publish(String(data=state))
        self.get_logger().warn(f"Control state changed: {previous} -> {state}")
        if state == "RETURN_HOME" and not self.is_holding_object() and not self.force_safety_pose:
            self._reset_pick_phase()

    def is_holding_object(self) -> bool:
        if self.task_mode == "PLACE":
            return self.grasp_phase not in ("HOLD",)
        return self.grasp_phase in ("GRASP", "LIFT", "HOLD")

    def compute_home_velocities(self) -> list[float]:
        velocities = []
        for index, joint_name in enumerate(self.joint_names):
            error = self.home_positions[index] - self.current_joints[joint_name]
            if abs(error) <= self.home_position_tolerance:
                velocities.append(0.0)
            else:
                velocities.append(
                    clamp(
                        self.home_kp * error,
                        -self.home_max_joint_velocity,
                        self.home_max_joint_velocity,
                    )
                )
        return velocities

    def compute_base_twist(
        self,
        target_x: float,
        target_y: float,
        arm_target_rho: float,
        base_scale: float,
    ) -> Twist:
        twist = Twist()
        if not self.enable_base_motion or base_scale <= 0.0:
            return twist

        yaw_error = atan2(target_y, target_x)
        twist.angular.z = base_scale * clamp(
            self.k_base_yaw * yaw_error,
            -self.max_base_yaw_rate,
            self.max_base_yaw_rate,
        )
        if arm_target_rho > self.base_stop_rho:
            heading_scale = max(0.0, cos(yaw_error))
            nominal_linear = clamp(
                self.k_base_linear * (arm_target_rho - self.base_stop_rho),
                0.0,
                self.max_base_linear,
            )
            twist.linear.x = base_scale * heading_scale * nominal_linear
        return twist

    def compute_switching(
        self,
        arm_target_rho: float,
        switching_rho: float,
        switching_alpha: float,
    ) -> float:
        mu = 0.5 * (
            1.0
            - tanh(
                switching_alpha
                * (arm_target_rho - switching_rho)
            )
        )
        return clamp(mu, 0.0, 1.0)

    def is_transition_target_safe(
        self,
        target_z: float,
        target_yaw: float,
    ) -> bool:
        return (
            self.safety_exit_z_min <= target_z <= self.safety_exit_z_max
            and abs(target_yaw) <= self.safety_exit_yaw_max
        )

    def integrate_position_command(
        self,
        velocity: list[float],
        dt: float,
    ) -> None:
        if self.position_command is None:
            return
        for index in range(6):
            next_position = self.position_command[index] + velocity[index] * dt
            self.position_command[index] = clamp(
                next_position,
                self.joint_lower_limits[index],
                self.joint_upper_limits[index],
            )

    def compute_joint_velocities(
        self,
        yaw_error: float,
        rho_error: float,
        z_error: float,
        q2: float,
        q3: float,
    ) -> list[float]:
        q1_dot = 0.0
        if abs(yaw_error) > self.yaw_tolerance:
            q1_dot = self.joint1_sign * clamp(
                self.k_yaw * yaw_error,
                -self.max_yaw_velocity,
                self.max_yaw_velocity,
            )

        if abs(rho_error) <= self.rho_tolerance and abs(z_error) <= self.z_tolerance:
            return [q1_dot, 0.0, 0.0]

        approach_error = hypot(rho_error, z_error)
        approach_scale = clamp(
            approach_error / self.approach_slowdown_distance,
            self.approach_min_scale,
            1.0,
        )
        v_rho = approach_scale * clamp(
            self.k_rho * rho_error,
            -self.max_rho_velocity,
            self.max_rho_velocity,
        )
        v_z = approach_scale * clamp(
            self.k_z * z_error,
            -self.max_z_velocity,
            self.max_z_velocity,
        )

        # Dynamic analytical Jacobian
        q2_phys = q2 * self.joint2_sign
        q3_phys = q3 * self.joint3_sign

        # Correct row assignment: Row 1 = d_rho (cos), Row 2 = d_z (-sin)
        j11 = self.link1 * cos(q2_phys) + self.link2 * cos(q2_phys + q3_phys)
        j12 = self.link2 * cos(q2_phys + q3_phys)
        j21 = -self.link1 * sin(q2_phys) - self.link2 * sin(q2_phys + q3_phys)
        j22 = -self.link2 * sin(q2_phys + q3_phys)

        determinant = j11 * j22 - j12 * j21

        if abs(determinant) < self.singularity_epsilon:
            return [q1_dot, 0.0, 0.0]

        q2_dot_phys = (j22 * v_rho - j12 * v_z) / determinant
        q3_dot_phys = (-j21 * v_rho + j11 * v_z) / determinant
        
        q2_dot = q2_dot_phys * self.joint2_sign
        q3_dot = q3_dot_phys * self.joint3_sign

        # Preserve the joint2/joint3 velocity ratio so the end effector keeps
        # the requested rho-z direction when either joint reaches its limit.
        velocity_scale = min(
            1.0,
            self.max_joint2_velocity / max(abs(q2_dot), 1e-9),
            self.max_joint3_velocity / max(abs(q3_dot), 1e-9),
        )
        return [
            q1_dot,
            q2_dot * velocity_scale,
            q3_dot * velocity_scale,
        ]

    def limit_acceleration(self, desired_velocity: list[float], dt: float) -> list[float]:
        max_delta = self.max_joint_acceleration * dt
        limited = []
        for previous, desired in zip(self.previous_velocity, desired_velocity):
            limited.append(previous + clamp(desired - previous, -max_delta, max_delta))
        self.previous_velocity = limited
        return limited

    def publish_position_command(self) -> None:
        if self.position_command is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names) + self.gripper_joint_names
        msg.position = list(self.position_command) + [
            self.gripper_position
        ] * len(self.gripper_joint_names)
        self.position_pub.publish(msg)

    # ── Pick/place sequence logic ─────────────────────────────────
    def _update_task_phase(self, pos_aligned: bool, dt: float) -> None:
        if self.task_mode == "PLACE":
            self._update_place_phase(pos_aligned, dt)
        else:
            self._update_pick_phase(pos_aligned, dt)

    def _update_pick_phase(self, pos_aligned: bool, dt: float) -> None:
        """Drive the pick APPROACH → DESCEND → GRASP → LIFT → HOLD sequence."""
        if self.grasp_phase == "APPROACH":
            self.gripper_position = self.gripper_open_position
            if pos_aligned:
                self.grasp_phase = "DESCEND"
                self.grasp_z_offset = self.grasp_offset_z
                self.get_logger().warn("Grasp phase: APPROACH → DESCEND")

        elif self.grasp_phase == "DESCEND":
            self.gripper_position = self.gripper_open_position
            # Gradually reduce the z offset to lower the arm
            descend_target_offset = self.grasp_offset_z - self.grasp_descend_depth
            if self.grasp_z_offset > descend_target_offset:
                self.grasp_z_offset = max(
                    descend_target_offset,
                    self.grasp_z_offset - self.grasp_descend_speed * dt,
                )
                return

            if pos_aligned:
                self.grasp_phase = "GRASP"
                self.grasp_phase_start_time = self.get_clock().now()
                self.get_logger().warn("Grasp phase: DESCEND → GRASP")

        elif self.grasp_phase == "GRASP":
            self.gripper_position = self.gripper_close_position
            elapsed = (
                self.get_clock().now() - self.grasp_phase_start_time
            ).nanoseconds * 1e-9
            if elapsed >= self.grasp_close_duration:
                self.grasp_phase = "LIFT"
                self.lift_z_accumulated = 0.0
                self.get_logger().warn("Grasp phase: GRASP → LIFT")

        elif self.grasp_phase == "LIFT":
            self.gripper_position = self.gripper_close_position
            if self.lift_z_accumulated < self.grasp_lift_height:
                lift_step = min(
                    self.grasp_lift_speed * dt,
                    self.grasp_lift_height - self.lift_z_accumulated,
                )
                self.grasp_z_offset += lift_step
                self.lift_z_accumulated += lift_step
                return

            if pos_aligned:
                self.grasp_phase = "HOLD"
                self.get_logger().warn("Grasp phase: LIFT → HOLD")

        elif self.grasp_phase == "HOLD":
            self.gripper_position = self.gripper_close_position

    def _update_place_phase(self, pos_aligned: bool, dt: float) -> None:
        """Drive the place APPROACH → DESCEND → RELEASE → RETREAT → HOLD sequence."""
        if self.grasp_phase == "APPROACH":
            self.gripper_position = self.gripper_close_position
            if pos_aligned:
                self.grasp_phase = "DESCEND"
                self.grasp_z_offset = self.place_offset_z
                self.get_logger().warn("Place phase: APPROACH → DESCEND")

        elif self.grasp_phase == "DESCEND":
            self.gripper_position = self.gripper_close_position
            release_offset = self.place_offset_z - self.place_descend_depth
            if self.grasp_z_offset > release_offset:
                self.grasp_z_offset = max(
                    release_offset,
                    self.grasp_z_offset - self.grasp_descend_speed * dt,
                )
                return

            if pos_aligned:
                self.grasp_phase = "RELEASE"
                self.grasp_phase_start_time = self.get_clock().now()
                self.get_logger().warn("Place phase: DESCEND → RELEASE")

        elif self.grasp_phase == "RELEASE":
            self.gripper_position = self.gripper_open_position
            elapsed = (
                self.get_clock().now() - self.grasp_phase_start_time
            ).nanoseconds * 1e-9
            if elapsed >= self.place_open_duration:
                if self.return_home_after_place:
                    self.grasp_phase = "HOLD"
                    self.force_safety_pose = True
                    self.set_control_state("RETURN_HOME")
                    self.get_logger().warn(
                        "Place phase: RELEASE → HOLD; returning to safety pose"
                    )
                    return

                self.grasp_phase = "RETREAT"
                self.get_logger().warn("Place phase: RELEASE → RETREAT")

        elif self.grasp_phase == "RETREAT":
            self.gripper_position = self.gripper_open_position
            if self.grasp_z_offset < self.place_offset_z:
                self.grasp_z_offset = min(
                    self.place_offset_z,
                    self.grasp_z_offset + self.grasp_lift_speed * dt,
                )
                return

            if pos_aligned:
                self.grasp_phase = "HOLD"
                self.get_logger().warn("Place phase: RETREAT → HOLD")

        elif self.grasp_phase == "HOLD":
            self.gripper_position = self.gripper_open_position

    def _reset_pick_phase(self) -> None:
        """Reset pick sequence before an object is held."""
        if self.grasp_phase != "APPROACH":
            self.get_logger().warn(
                f"Grasp phase reset: {self.grasp_phase} → APPROACH"
            )
        self.task_mode = "PICK"
        self.grasp_phase = "APPROACH"
        self.grasp_z_offset = self.grasp_offset_z
        self.gripper_position = self.gripper_open_position
        self.lift_z_accumulated = 0.0

    def on_log_timer(self) -> None:
        if self.control_state is not None:
            self.state_pub.publish(String(data=self.control_state))
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmYawRhoZPositionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
