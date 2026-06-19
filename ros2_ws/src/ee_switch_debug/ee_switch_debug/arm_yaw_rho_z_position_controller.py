from math import atan2, cos, hypot, pi, tanh

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
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("yaw_error_source", "ee")
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

        self.arm_base_frame = str(self.get_parameter("arm_base_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.ee_frame = str(self.get_parameter("ee_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        joint_position_topic = str(self.get_parameter("joint_position_topic").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
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

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}
        self.position_command: list[float] | None = None
        self.previous_velocity = [0.0] * 6
        self.control_state: str | None = None
        self.last_update_time = None
        self.last_debug = "waiting for joint states"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.position_pub = self.create_publisher(JointState, joint_position_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
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
        except TransformException as exc:
            self.set_control_state("RETURN_HOME")
            desired_velocity = self.compute_home_velocities()
            limited_velocity = self.limit_acceleration(desired_velocity, dt)
            self.integrate_position_command(limited_velocity, dt)
            self.cmd_pub.publish(Twist())
            self.publish_position_command()
            self.last_debug = f"RETURN_HOME; base stopped; TF lookup failed: {exc}"
            return

        target = target_transform.transform.translation
        ee = ee_transform.transform.translation
        base_target = base_target_transform.transform.translation
        target_yaw = atan2(target.y, target.x)
        target_rho = hypot(target.x, target.y)
        ee_yaw = atan2(ee.y, ee.x)
        joint_yaw_error = normalize_angle(
            target_yaw - self.joint1_sign * self.current_joints["joint1"]
        )
        ee_yaw_error = normalize_angle(target_yaw - ee_yaw)
        yaw_error = (
            ee_yaw_error
            if self.yaw_error_source == "ee"
            else joint_yaw_error
        )
        rho_error = target_rho - hypot(ee.x, ee.y)
        z_error = target.z - ee.z

        self.update_control_state(target_rho, target.z, target_yaw)

        if self.control_state == "RETURN_HOME":
            home_velocity = self.compute_home_velocities()
            tracking_velocity = self.compute_joint_velocities(
                yaw_error,
                rho_error,
                z_error,
            )
            tracking_velocity.extend([0.0, 0.0, 0.0])
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
                if self.is_transition_target_safe(target.z, target_yaw)
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
                f"z {target.z:.3f}], home_error={home_error:.3f}, "
                f"mu={mu:.3f}, arm_mu={arm_mu:.3f}, "
                f"arm_blend={arm_blend:.3f}, "
                f"base_scale={base_scale:.3f}, "
                f"base=[vx {twist.linear.x:.3f}, wz {twist.angular.z:.3f}]"
            )
            return

        self.cmd_pub.publish(Twist())

        desired_velocity = self.compute_joint_velocities(yaw_error, rho_error, z_error)
        desired_velocity.extend([0.0, 0.0, 0.0])
        limited_velocity = self.limit_acceleration(desired_velocity, dt)
        self.integrate_position_command(limited_velocity, dt)

        self.publish_position_command()
        self.last_debug = (
            f"ARM_TRACK target=[rho {target_rho:.3f}, yaw {target_yaw:.3f}, "
            f"z {target.z:.3f}], error=[yaw {yaw_error:.3f}, "
            f"rho {rho_error:.3f}, z {z_error:.3f}], "
            f"qdot=[{limited_velocity[0]:.3f}, {limited_velocity[1]:.3f}, "
            f"{limited_velocity[2]:.3f}], qcmd=[{self.position_command[0]:.3f}, "
            f"{self.position_command[1]:.3f}, {self.position_command[2]:.3f}]"
        )

    def update_control_state(
        self,
        target_rho: float,
        target_z: float,
        target_yaw: float,
    ) -> None:
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
        determinant = (
            self.joint2_rho_per_rad * self.joint3_z_per_rad
            - self.joint3_rho_per_rad * self.joint2_z_per_rad
        )
        if abs(determinant) < self.singularity_epsilon:
            return [q1_dot, 0.0, 0.0]

        q2_dot = (
            self.joint3_z_per_rad * v_rho - self.joint3_rho_per_rad * v_z
        ) / determinant
        q3_dot = (
            -self.joint2_z_per_rad * v_rho + self.joint2_rho_per_rad * v_z
        ) / determinant
        return [
            q1_dot,
            self.joint2_sign
            * clamp(
                q2_dot,
                -self.max_joint2_velocity,
                self.max_joint2_velocity,
            ),
            self.joint3_sign
            * clamp(
                q3_dot,
                -self.max_joint3_velocity,
                self.max_joint3_velocity,
            ),
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
        msg.name = self.joint_names
        msg.position = self.position_command
        self.position_pub.publish(msg)

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
