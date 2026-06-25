from math import acos, atan2, cos, hypot, pi, sin, sqrt, tanh

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
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


class WholeBodyYawRhoZController(Node):
    def __init__(self) -> None:
        super().__init__("wholebody_yaw_rho_z_controller")

        self.declare_parameter("base_frame", "chassis_link")
        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_velocity_topic", "/joint_velocity_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/wholebody_yaw_rho_z_debug")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("yaw_joint_name", "joint1")
        self.declare_parameter("yaw_error_source", "joint")
        self.declare_parameter("joint1_sign", 1.0)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)
        self.declare_parameter("joint2_rho_per_rad", 0.2715)
        self.declare_parameter("joint2_z_per_rad", -0.1972)
        self.declare_parameter("joint3_rho_per_rad", 0.1521)
        self.declare_parameter("joint3_z_per_rad", -0.2631)

        self.declare_parameter("switching_rho", 0.55)
        self.declare_parameter("switching_alpha", 8.0)
        self.declare_parameter("switching_mode", "fixed")
        self.declare_parameter("workspace_radius", 0.80)
        self.declare_parameter("switching_ratio", 1.0)
        self.declare_parameter("min_switching_rho", 0.10)
        self.declare_parameter("max_switching_rho", 1.00)
        self.declare_parameter("base_stop_rho", 0.42)
        self.declare_parameter("base_yaw_tolerance", 0.08)

        self.declare_parameter("k_arm_yaw", 1.5)
        self.declare_parameter("k_rho", 1.0)
        self.declare_parameter("k_z", 1.5)
        self.declare_parameter("k_base_yaw", 2.5)
        self.declare_parameter("k_base_linear", 0.8)
        self.declare_parameter("k_wrist", 2.0)
        self.declare_parameter("max_wrist_velocity", 1.0)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)

        self.declare_parameter("max_arm_yaw_velocity", 0.5)
        self.declare_parameter("max_rho_velocity", 0.04)
        self.declare_parameter("max_z_velocity", 0.04)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("max_base_yaw_rate", 1.0)
        self.declare_parameter("max_base_linear", 0.35)

        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("rho_tolerance", 0.04)
        self.declare_parameter("z_tolerance", 0.05)
        self.declare_parameter("singularity_epsilon", 0.01)
        self.declare_parameter("yaw_first", False)
        self.declare_parameter("z_first", False)
        self.declare_parameter("enable_position_hold", False)
        self.declare_parameter("hold_enter_samples", 5)
        self.declare_parameter("hold_exit_yaw_tolerance", 0.06)
        self.declare_parameter("hold_exit_rho_tolerance", 0.06)
        self.declare_parameter("hold_exit_z_tolerance", 0.025)

        self.base_frame = self.get_parameter("base_frame").value
        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        joint_velocity_topic = self.get_parameter("joint_velocity_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        debug_topic = self.get_parameter("debug_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.yaw_joint_name = self.get_parameter("yaw_joint_name").value
        self.yaw_error_source = str(self.get_parameter("yaw_error_source").value)
        self.joint1_sign = float(self.get_parameter("joint1_sign").value)
        self.joint2_sign = float(self.get_parameter("joint2_sign").value)
        self.joint3_sign = float(self.get_parameter("joint3_sign").value)
        self.joint2_rho_per_rad = float(self.get_parameter("joint2_rho_per_rad").value)
        self.joint2_z_per_rad = float(self.get_parameter("joint2_z_per_rad").value)
        self.joint3_rho_per_rad = float(self.get_parameter("joint3_rho_per_rad").value)
        self.joint3_z_per_rad = float(self.get_parameter("joint3_z_per_rad").value)

        self.switching_rho = float(self.get_parameter("switching_rho").value)
        self.switching_alpha = float(self.get_parameter("switching_alpha").value)
        self.switching_mode = str(self.get_parameter("switching_mode").value)
        self.workspace_radius = float(self.get_parameter("workspace_radius").value)
        self.switching_ratio = float(self.get_parameter("switching_ratio").value)
        self.min_switching_rho = float(self.get_parameter("min_switching_rho").value)
        self.max_switching_rho = float(self.get_parameter("max_switching_rho").value)
        self.base_stop_rho = float(self.get_parameter("base_stop_rho").value)
        self.base_yaw_tolerance = float(self.get_parameter("base_yaw_tolerance").value)

        self.k_arm_yaw = float(self.get_parameter("k_arm_yaw").value)
        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_z = float(self.get_parameter("k_z").value)
        self.k_base_yaw = float(self.get_parameter("k_base_yaw").value)
        self.k_base_linear = float(self.get_parameter("k_base_linear").value)

        self.max_arm_yaw_velocity = float(self.get_parameter("max_arm_yaw_velocity").value)
        self.max_rho_velocity = float(self.get_parameter("max_rho_velocity").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.max_base_yaw_rate = float(self.get_parameter("max_base_yaw_rate").value)
        self.max_base_linear = float(self.get_parameter("max_base_linear").value)

        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.rho_tolerance = float(self.get_parameter("rho_tolerance").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.singularity_epsilon = float(self.get_parameter("singularity_epsilon").value)
        self.yaw_first = bool(self.get_parameter("yaw_first").value)
        self.z_first = bool(self.get_parameter("z_first").value)
        self.enable_position_hold = bool(self.get_parameter("enable_position_hold").value)
        self.hold_enter_samples = int(self.get_parameter("hold_enter_samples").value)
        self.hold_exit_yaw_tolerance = float(self.get_parameter("hold_exit_yaw_tolerance").value)
        self.hold_exit_rho_tolerance = float(self.get_parameter("hold_exit_rho_tolerance").value)
        self.hold_exit_z_tolerance = float(self.get_parameter("hold_exit_z_tolerance").value)
        self.k_wrist = float(self.get_parameter("k_wrist").value)
        self.max_wrist_velocity = float(self.get_parameter("max_wrist_velocity").value)
        self.link1 = float(self.get_parameter("link1").value)
        self.link2 = float(self.get_parameter("link2").value)

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.home_wrist_positions = [-0.5236, 1.5708, 0.0]
        self.current_joints: dict[str, float] = {}
        self.hold_active = False
        self.hold_aligned_count = 0
        self.hold_positions: list[float] = []
        self.last_singularity_log_ns = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.joint_pub = self.create_publisher(JointState, joint_velocity_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, debug_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)
        self.last_debug = "waiting for transform"
        self.debug_labels = [
            "mu",
            "arm_scale",
            "base_scale",
            "rho_w",
            "active_switching_rho",
            "arm_target_yaw",
            "ee_yaw",
            "joint_yaw_error",
            "ee_yaw_error",
            "joint1",
            "arm_yaw_error",
            "arm_target_rho",
            "ee_rho",
            "rho_error",
            "target_z",
            "ee_z",
            "z_error",
            "base_target_rho",
            "base_yaw_error",
            "aligned",
            "hold_active",
            "cmd_v",
            "cmd_w",
            "joint1_velocity_cmd",
            "joint2_velocity_cmd",
            "joint3_velocity_cmd",
            "jacobian_det",
        ]

        self.get_logger().info(
            f"Whole-body controller: target={self.target_frame}, base={self.base_frame}, "
            f"arm_base={self.arm_base_frame}, publishing {cmd_topic} and {joint_velocity_topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        required_joints = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
        missing = [name for name in required_joints if name not in self.current_joints]
        if missing:
            self.last_debug = f"waiting for joint states: {', '.join(missing)}"
            self.publish_stop()
            return

        try:
            arm_target_transform = self.tf_buffer.lookup_transform(
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
            self.last_debug = f"lookup failed: {exc}"
            self.publish_stop()
            return

        arm_target = arm_target_transform.transform.translation
        ee = ee_transform.transform.translation
        base_target = base_target_transform.transform.translation

        R_target = quaternion_to_matrix(arm_target_transform.transform.rotation)
        R_ee = quaternion_to_matrix(ee_transform.transform.rotation)

        offset_local = [0.008493, 0.017565, 0.0]
        # Current wrist center position in link0 frame
        wc_x = ee.x - (R_ee[0][0]*offset_local[0] + R_ee[0][1]*offset_local[1] + R_ee[0][2]*offset_local[2])
        wc_y = ee.y - (R_ee[1][0]*offset_local[0] + R_ee[1][1]*offset_local[1] + R_ee[1][2]*offset_local[2])
        wc_z = ee.z - (R_ee[2][0]*offset_local[0] + R_ee[2][1]*offset_local[1] + R_ee[2][2]*offset_local[2])

        # Target wrist center position in link0 frame
        wc_target_x = arm_target.x - (R_target[0][0]*offset_local[0] + R_target[0][1]*offset_local[1] + R_target[0][2]*offset_local[2])
        wc_target_y = arm_target.y - (R_target[1][0]*offset_local[0] + R_target[1][1]*offset_local[1] + R_target[1][2]*offset_local[2])
        wc_target_z = arm_target.z - (R_target[2][0]*offset_local[0] + R_target[2][1]*offset_local[1] + R_target[2][2]*offset_local[2])

        arm_target_yaw = atan2(wc_target_y, wc_target_x)
        ee_yaw = atan2(wc_y, wc_x)
        joint1 = self.current_joints[self.yaw_joint_name]
        joint_yaw_error = normalize_angle(arm_target_yaw - self.joint1_sign * joint1)
        ee_yaw_error = normalize_angle(arm_target_yaw - ee_yaw)
        if self.yaw_error_source == "ee" and ee_rho >= 0.25:
            arm_yaw_error = ee_yaw_error
        else:
            arm_yaw_error = joint_yaw_error
        yaw_aligned = abs(arm_yaw_error) <= self.yaw_tolerance

        arm_target_rho = hypot(wc_target_x, wc_target_y)
        ee_rho = hypot(wc_x, wc_y)
        target_z = wc_target_z
        ee_z = wc_z
        rho_error = arm_target_rho - ee_rho
        z_error = target_z - ee_z
        rho_aligned = abs(rho_error) <= self.rho_tolerance
        z_aligned = abs(z_error) <= self.z_tolerance
        aligned = yaw_aligned and rho_aligned and z_aligned

        mu, rho_w, active_switching_rho = self.compute_switching(arm_target_rho, target_z)
        arm_scale = mu
        base_scale = 0.0 if arm_target_rho <= self.base_stop_rho else 1.0 - mu

        # Simple wrist control: keep joints 4,5 at home, always rotate joint 6 by -90 deg
        q4_des = self.home_wrist_positions[0]
        q5_des = self.home_wrist_positions[1]
        q6_des = -1.5708  # -90 degrees to orient gripper for grasping

        q4_curr = self.current_joints["joint4"]
        q5_curr = self.current_joints["joint5"]
        q6_curr = self.current_joints["joint6"]

        e4 = normalize_angle(q4_des - q4_curr)
        e5 = normalize_angle(q5_des - q5_curr)
        e6 = normalize_angle(q6_des - q6_curr)

        joint4_velocity = clamp(self.k_wrist * e4, -self.max_wrist_velocity, self.max_wrist_velocity)
        joint5_velocity = clamp(self.k_wrist * e5, -self.max_wrist_velocity, self.max_wrist_velocity)
        joint6_velocity = clamp(self.k_wrist * e6, -self.max_wrist_velocity, self.max_wrist_velocity)

        self.update_hold_state(aligned, arm_yaw_error, rho_error, z_error)
        if self.hold_active:
            joint1_velocity = 0.0
            joint2_velocity = 0.0
            joint3_velocity = 0.0
            joint4_velocity = 0.0
            joint5_velocity = 0.0
            joint6_velocity = 0.0
            jacobian_det = 0.0
            twist = Twist()
            joint_msg = self.make_joint_command_msg(
                joint1_velocity,
                joint2_velocity,
                joint3_velocity,
                joint4_velocity,
                joint5_velocity,
                joint6_velocity,
                positions=self.hold_positions,
            )
        else:
            joint1_velocity = self.compute_arm_yaw_velocity(arm_yaw_error, yaw_aligned, arm_scale)
            joint2_velocity, joint3_velocity, jacobian_det = self.compute_rho_z_velocities(
                rho_error,
                z_error,
                yaw_aligned,
                z_aligned,
                aligned,
                arm_scale,
                self.current_joints["joint2"],
                self.current_joints["joint3"],
            )
            twist = self.compute_base_twist(base_target.x, base_target.y, arm_target_rho, base_scale)
            joint_msg = self.make_joint_command_msg(
                joint1_velocity,
                joint2_velocity,
                joint3_velocity,
                joint4_velocity,
                joint5_velocity,
                joint6_velocity,
            )

        self.joint_pub.publish(joint_msg)
        self.cmd_pub.publish(twist)
        self.publish_debug(
            mu,
            arm_scale,
            base_scale,
            rho_w,
            active_switching_rho,
            arm_target_yaw,
            ee_yaw,
            joint_yaw_error,
            ee_yaw_error,
            joint1,
            arm_yaw_error,
            arm_target_rho,
            ee_rho,
            rho_error,
            target_z,
            ee_z,
            z_error,
            hypot(base_target.x, base_target.y),
            atan2(base_target.y, base_target.x),
            aligned,
            self.hold_active,
            twist,
            joint1_velocity,
            joint2_velocity,
            joint3_velocity,
            jacobian_det,
        )
        self.last_debug = (
            f"mu={mu:.3f}, arm_scale={arm_scale:.3f}, base_scale={base_scale:.3f}, "
            f"rho_w={rho_w:.3f}, switch_rho={active_switching_rho:.3f}, "
            f"yaw_source={self.yaw_error_source}, "
            f"arm_yaw_error={arm_yaw_error:.3f}, rho_error={rho_error:.3f}, z_error={z_error:.3f}, "
            f"cmd=[{twist.linear.x:.3f}, {twist.angular.z:.3f}], "
            f"joint_vel=[{joint1_velocity:.3f}, {joint2_velocity:.3f}, {joint3_velocity:.3f}], "
            f"aligned={aligned}, hold={self.hold_active}"
        )

    def update_hold_state(self, aligned: bool, yaw_error: float, rho_error: float, z_error: float) -> None:
        if not self.enable_position_hold:
            self.hold_active = False
            self.hold_aligned_count = 0
            self.hold_positions = []
            return

        if self.hold_active:
            hold_valid = (
                abs(yaw_error) <= self.hold_exit_yaw_tolerance
                and abs(rho_error) <= self.hold_exit_rho_tolerance
                and abs(z_error) <= self.hold_exit_z_tolerance
            )
            if hold_valid:
                return
            self.hold_active = False
            self.hold_positions = []
            self.hold_aligned_count = 0

        if not aligned:
            self.hold_aligned_count = 0
            return

        if not all(name in self.current_joints for name in self.joint_names):
            self.hold_aligned_count = 0
            return

        self.hold_aligned_count += 1
        if self.hold_aligned_count >= self.hold_enter_samples:
            self.hold_positions = [self.current_joints[name] for name in self.joint_names]
            self.hold_active = True

    def compute_switching(self, arm_target_rho: float, target_z: float) -> tuple[float, float, float]:
        rho_w = sqrt(max(self.workspace_radius * self.workspace_radius - target_z * target_z, 0.0))
        if self.switching_mode == "workspace":
            active_switching_rho = clamp(
                self.switching_ratio * rho_w,
                self.min_switching_rho,
                self.max_switching_rho,
            )
        else:
            active_switching_rho = self.switching_rho

        mu = 0.5 * (1.0 - tanh(self.switching_alpha * (arm_target_rho - active_switching_rho)))
        return clamp(mu, 0.0, 1.0), rho_w, active_switching_rho

    def compute_arm_yaw_velocity(self, yaw_error: float, yaw_aligned: bool, arm_scale: float) -> float:
        if yaw_aligned:
            return 0.0
        return self.joint1_sign * arm_scale * clamp(
            self.k_arm_yaw * yaw_error,
            -self.max_arm_yaw_velocity,
            self.max_arm_yaw_velocity,
        )

    def compute_rho_z_velocities(
        self,
        rho_error: float,
        z_error: float,
        yaw_aligned: bool,
        z_aligned: bool,
        aligned: bool,
        arm_scale: float,
        q2: float,
        q3: float,
    ) -> tuple[float, float, float]:
        if aligned or (self.yaw_first and not yaw_aligned):
            return 0.0, 0.0, 0.0

        v_rho = arm_scale * clamp(self.k_rho * rho_error, -self.max_rho_velocity, self.max_rho_velocity)
        v_z = arm_scale * clamp(self.k_z * z_error, -self.max_z_velocity, self.max_z_velocity)
        if self.z_first and not z_aligned:
            v_rho = 0.0
        return self.solve_joint_velocities(v_rho, v_z, q2, q3)

    def compute_base_twist(self, target_x: float, target_y: float, arm_target_rho: float, base_scale: float) -> Twist:
        twist = Twist()
        if base_scale <= 0.0:
            return twist

        base_yaw_error = atan2(target_y, target_x)
        base_yaw_aligned = abs(base_yaw_error) <= self.base_yaw_tolerance
        twist.angular.z = base_scale * clamp(
            self.k_base_yaw * base_yaw_error,
            -self.max_base_yaw_rate,
            self.max_base_yaw_rate,
        )

        if base_yaw_aligned and arm_target_rho > self.base_stop_rho:
            twist.linear.x = base_scale * clamp(
                self.k_base_linear * (arm_target_rho - self.base_stop_rho),
                0.0,
                self.max_base_linear,
            )
        return twist

    def solve_joint_velocities(self, v_rho: float, v_z: float, q2: float, q3: float) -> tuple[float, float, float]:
        q2_phys = q2 * self.joint2_sign
        q3_phys = q3 * self.joint3_sign

        # Correct row assignment: Row 1 = d_rho (cos), Row 2 = d_z (-sin)
        j11 = self.link1 * cos(q2_phys) + self.link2 * cos(q2_phys + q3_phys)
        j12 = self.link2 * cos(q2_phys + q3_phys)
        j21 = -self.link1 * sin(q2_phys) - self.link2 * sin(q2_phys + q3_phys)
        j22 = -self.link2 * sin(q2_phys + q3_phys)

        det = j11 * j22 - j12 * j21

        if abs(det) < self.singularity_epsilon:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_singularity_log_ns > 1_000_000_000:
                self.get_logger().warn(f"Measured rho-z Jacobian is singular: det={det:.4f}. Sending zero velocity.")
                self.last_singularity_log_ns = now_ns
            return 0.0, 0.0, det

        q2_dot_phys = (j22 * v_rho - j12 * v_z) / det
        q3_dot_phys = (-j21 * v_rho + j11 * v_z) / det

        q2_dot = q2_dot_phys * self.joint2_sign
        q3_dot = q3_dot_phys * self.joint3_sign

        q2_dot = clamp(q2_dot, -self.max_joint_velocity, self.max_joint_velocity)
        q3_dot = clamp(q3_dot, -self.max_joint_velocity, self.max_joint_velocity)
        return q2_dot, q3_dot, det

    def publish_stop(self) -> None:
        self.joint_pub.publish(self.make_joint_command_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        self.cmd_pub.publish(Twist())

    def make_joint_command_msg(
        self,
        joint1_velocity: float,
        joint2_velocity: float,
        joint3_velocity: float,
        joint4_velocity: float = 0.0,
        joint5_velocity: float = 0.0,
        joint6_velocity: float = 0.0,
        positions: list[float] | None = None,
    ) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        if positions is not None:
            msg.position = positions
        msg.velocity = [
            joint1_velocity,
            joint2_velocity,
            joint3_velocity,
            joint4_velocity,
            joint5_velocity,
            joint6_velocity,
        ]
        return msg

    def publish_debug(
        self,
        mu: float,
        arm_scale: float,
        base_scale: float,
        rho_w: float,
        active_switching_rho: float,
        arm_target_yaw: float,
        ee_yaw: float,
        joint_yaw_error: float,
        ee_yaw_error: float,
        joint1: float,
        arm_yaw_error: float,
        arm_target_rho: float,
        ee_rho: float,
        rho_error: float,
        target_z: float,
        ee_z: float,
        z_error: float,
        base_target_rho: float,
        base_yaw_error: float,
        aligned: bool,
        hold_active: bool,
        twist: Twist,
        joint1_velocity: float,
        joint2_velocity: float,
        joint3_velocity: float,
        jacobian_det: float,
    ) -> None:
        msg = Float64MultiArray()
        msg.layout.dim.append(
            MultiArrayDimension(
                label=",".join(self.debug_labels),
                size=len(self.debug_labels),
                stride=len(self.debug_labels),
            )
        )
        msg.data = [
            mu,
            arm_scale,
            base_scale,
            rho_w,
            active_switching_rho,
            arm_target_yaw,
            ee_yaw,
            joint_yaw_error,
            ee_yaw_error,
            joint1,
            arm_yaw_error,
            arm_target_rho,
            ee_rho,
            rho_error,
            target_z,
            ee_z,
            z_error,
            base_target_rho,
            base_yaw_error,
            1.0 if aligned else 0.0,
            1.0 if hold_active else 0.0,
            twist.linear.x,
            twist.angular.z,
            joint1_velocity,
            joint2_velocity,
            joint3_velocity,
            jacobian_det,
        ]
        self.debug_pub.publish(msg)

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WholeBodyYawRhoZController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
