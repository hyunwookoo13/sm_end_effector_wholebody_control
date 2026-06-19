from math import cos, hypot, sin

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ArmRhoZVelocityController(Node):
    def __init__(self) -> None:
        super().__init__("arm_rho_z_velocity_controller")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_velocity_topic", "/joint_velocity_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/arm_rho_z_debug")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)
        self.declare_parameter("elbow_sign", -1.0)
        self.declare_parameter("kinematic_model", "measured")
        self.declare_parameter("joint2_rho_per_rad", 0.2715)
        self.declare_parameter("joint2_z_per_rad", -0.1972)
        self.declare_parameter("joint3_rho_per_rad", 0.1521)
        self.declare_parameter("joint3_z_per_rad", -0.2631)
        self.declare_parameter("k_rho", 1.0)
        self.declare_parameter("k_z", 1.5)
        self.declare_parameter("rho_command_sign", 1.0)
        self.declare_parameter("z_command_sign", 1.0)
        self.declare_parameter("max_rho_velocity", 0.04)
        self.declare_parameter("max_z_velocity", 0.04)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("rho_tolerance", 0.04)
        self.declare_parameter("z_tolerance", 0.04)
        self.declare_parameter("singularity_epsilon", 0.01)
        self.declare_parameter("z_first", False)

        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        joint_velocity_topic = self.get_parameter("joint_velocity_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        debug_topic = self.get_parameter("debug_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.link1 = float(self.get_parameter("link1").value)
        self.link2 = float(self.get_parameter("link2").value)
        self.joint2_sign = float(self.get_parameter("joint2_sign").value)
        self.joint3_sign = float(self.get_parameter("joint3_sign").value)
        self.elbow_sign = float(self.get_parameter("elbow_sign").value)
        self.kinematic_model = self.get_parameter("kinematic_model").value
        self.joint2_rho_per_rad = float(self.get_parameter("joint2_rho_per_rad").value)
        self.joint2_z_per_rad = float(self.get_parameter("joint2_z_per_rad").value)
        self.joint3_rho_per_rad = float(self.get_parameter("joint3_rho_per_rad").value)
        self.joint3_z_per_rad = float(self.get_parameter("joint3_z_per_rad").value)
        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_z = float(self.get_parameter("k_z").value)
        self.rho_command_sign = float(self.get_parameter("rho_command_sign").value)
        self.z_command_sign = float(self.get_parameter("z_command_sign").value)
        self.max_rho_velocity = float(self.get_parameter("max_rho_velocity").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.rho_tolerance = float(self.get_parameter("rho_tolerance").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.singularity_epsilon = float(self.get_parameter("singularity_epsilon").value)
        self.z_first = bool(self.get_parameter("z_first").value)
        self.last_singularity_log_ns = 0

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}

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
            "target_rho",
            "ee_rho",
            "rho_error",
            "target_z",
            "ee_z",
            "z_error",
            "aligned",
            "v_rho_cmd",
            "v_z_cmd",
            "joint2_velocity_cmd",
            "joint3_velocity_cmd",
            "jacobian_det",
        ]

        self.get_logger().info(
            f"Arm rho-z velocity controller: target={self.target_frame}, EE={self.ee_frame}, "
            f"model={self.kinematic_model}, publishing {joint_velocity_topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        self.cmd_pub.publish(Twist())

        missing = [name for name in ("joint2", "joint3") if name not in self.current_joints]
        if missing:
            self.last_debug = f"waiting for joint states: {', '.join(missing)}"
            self.publish_stop()
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
        except TransformException as exc:
            self.last_debug = f"lookup failed: {exc}"
            self.publish_stop()
            return

        target_translation = target_transform.transform.translation
        ee_translation = ee_transform.transform.translation
        target_rho = hypot(target_translation.x, target_translation.y)
        ee_rho = hypot(ee_translation.x, ee_translation.y)
        target_z = target_translation.z
        ee_z = ee_translation.z
        rho_error = target_rho - ee_rho
        z_error = target_z - ee_z
        rho_aligned = abs(rho_error) <= self.rho_tolerance
        z_aligned = abs(z_error) <= self.z_tolerance
        aligned = rho_aligned and z_aligned

        if aligned:
            v_rho = 0.0
            v_z = 0.0
            joint2_velocity = 0.0
            joint3_velocity = 0.0
            jacobian_det = 0.0
        else:
            v_rho = self.rho_command_sign * clamp(
                self.k_rho * rho_error,
                -self.max_rho_velocity,
                self.max_rho_velocity,
            )
            v_z = self.z_command_sign * clamp(
                self.k_z * z_error,
                -self.max_z_velocity,
                self.max_z_velocity,
            )
            if self.z_first and not z_aligned:
                v_rho = 0.0
            joint2_velocity, joint3_velocity, jacobian_det = self.solve_joint_velocities(v_rho, v_z)

        self.publish_joint_velocity(joint2_velocity, joint3_velocity)
        self.publish_debug(
            target_rho,
            ee_rho,
            rho_error,
            target_z,
            ee_z,
            z_error,
            aligned,
            v_rho,
            v_z,
            joint2_velocity,
            joint3_velocity,
            jacobian_det,
        )
        self.last_debug = (
            f"target_rho={target_rho:.3f}, ee_rho={ee_rho:.3f}, rho_error={rho_error:.3f}, "
            f"target_z={target_z:.3f}, ee_z={ee_z:.3f}, z_error={z_error:.3f}, "
            f"aligned={aligned}, v_rho={v_rho:.3f}, v_z={v_z:.3f}, "
            f"joint2_vel={joint2_velocity:.3f}, joint3_vel={joint3_velocity:.3f}, "
            f"det={jacobian_det:.4f}"
        )

    def solve_joint_velocities(self, v_rho: float, v_z: float) -> tuple[float, float, float]:
        if self.kinematic_model == "measured":
            return self.solve_measured_joint_velocities(v_rho, v_z)
        return self.solve_paper_joint_velocities(v_rho, v_z)

    def solve_measured_joint_velocities(self, v_rho: float, v_z: float) -> tuple[float, float, float]:
        j11 = self.joint2_rho_per_rad
        j12 = self.joint3_rho_per_rad
        j21 = self.joint2_z_per_rad
        j22 = self.joint3_z_per_rad
        det = j11 * j22 - j12 * j21

        if abs(det) < self.singularity_epsilon:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_singularity_log_ns > 1_000_000_000:
                self.get_logger().warn(f"Measured rho-z Jacobian is singular: det={det:.4f}. Sending zero velocity.")
                self.last_singularity_log_ns = now_ns
            return 0.0, 0.0, det

        q2_dot = (j22 * v_rho - j12 * v_z) / det
        q3_dot = (-j21 * v_rho + j11 * v_z) / det
        q2_dot = clamp(q2_dot, -self.max_joint_velocity, self.max_joint_velocity)
        q3_dot = clamp(q3_dot, -self.max_joint_velocity, self.max_joint_velocity)
        return self.joint2_sign * q2_dot, self.joint3_sign * q3_dot, det

    def solve_paper_joint_velocities(self, v_rho: float, v_z: float) -> tuple[float, float, float]:
        q2 = self.current_joints["joint2"] / self.joint2_sign
        q3 = self.current_joints["joint3"] / self.joint3_sign
        q23 = q2 + self.elbow_sign * q3

        # Paper model: rho = L1*cos(q2) + L2*cos(q2 - q3), z = L1*sin(q2) + L2*sin(q2 - q3).
        # elbow_sign lets us flip the q3 convention if the USD joint direction differs.
        j11 = -self.link1 * sin(q2) - self.link2 * sin(q23)
        j12 = -self.link2 * self.elbow_sign * sin(q23)
        j21 = self.link1 * cos(q2) + self.link2 * cos(q23)
        j22 = self.link2 * self.elbow_sign * cos(q23)
        det = j11 * j22 - j12 * j21

        if abs(det) < self.singularity_epsilon:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_singularity_log_ns > 1_000_000_000:
                self.get_logger().warn(f"Near singularity for rho-z control: det={det:.4f}. Sending zero velocity.")
                self.last_singularity_log_ns = now_ns
            return 0.0, 0.0, det

        q2_dot = (j22 * v_rho - j12 * v_z) / det
        q3_dot = (-j21 * v_rho + j11 * v_z) / det
        q2_dot = clamp(q2_dot, -self.max_joint_velocity, self.max_joint_velocity)
        q3_dot = clamp(q3_dot, -self.max_joint_velocity, self.max_joint_velocity)
        return self.joint2_sign * q2_dot, self.joint3_sign * q3_dot, det

    def publish_stop(self) -> None:
        self.publish_joint_velocity(0.0, 0.0)

    def publish_joint_velocity(self, joint2_velocity: float, joint3_velocity: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.velocity = [
            0.0,
            joint2_velocity,
            joint3_velocity,
            0.0,
            0.0,
            0.0,
        ]
        self.joint_pub.publish(msg)

    def publish_debug(
        self,
        target_rho: float,
        ee_rho: float,
        rho_error: float,
        target_z: float,
        ee_z: float,
        z_error: float,
        aligned: bool,
        v_rho: float,
        v_z: float,
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
            target_rho,
            ee_rho,
            rho_error,
            target_z,
            ee_z,
            z_error,
            1.0 if aligned else 0.0,
            v_rho,
            v_z,
            joint2_velocity,
            joint3_velocity,
            jacobian_det,
        ]
        self.debug_pub.publish(msg)

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmRhoZVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
