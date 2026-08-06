from math import atan2, hypot, pi

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


class ArmYawRhoZVelocityController(Node):
    def __init__(self) -> None:
        super().__init__("arm_yaw_rho_z_velocity_controller")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_velocity_topic", "/joint_velocity_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/arm_yaw_rho_z_debug")
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
        self.declare_parameter("k_yaw", 1.5)
        self.declare_parameter("k_rho", 1.0)
        self.declare_parameter("k_z", 1.5)
        self.declare_parameter("max_yaw_velocity", 0.5)
        self.declare_parameter("max_rho_velocity", 0.04)
        self.declare_parameter("max_z_velocity", 0.04)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("yaw_tolerance", 0.03)
        self.declare_parameter("rho_tolerance", 0.04)
        self.declare_parameter("z_tolerance", 0.04)
        self.declare_parameter("singularity_epsilon", 0.01)
        self.declare_parameter("yaw_first", False)
        self.declare_parameter("z_first", False)

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
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.k_rho = float(self.get_parameter("k_rho").value)
        self.k_z = float(self.get_parameter("k_z").value)
        self.max_yaw_velocity = float(self.get_parameter("max_yaw_velocity").value)
        self.max_rho_velocity = float(self.get_parameter("max_rho_velocity").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.rho_tolerance = float(self.get_parameter("rho_tolerance").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.singularity_epsilon = float(self.get_parameter("singularity_epsilon").value)
        self.yaw_first = bool(self.get_parameter("yaw_first").value)
        self.z_first = bool(self.get_parameter("z_first").value)

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}
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
            "target_yaw",
            "ee_yaw",
            "joint_yaw_error",
            "ee_yaw_error",
            "joint1",
            "yaw_error",
            "target_rho",
            "ee_rho",
            "rho_error",
            "target_z",
            "ee_z",
            "z_error",
            "aligned",
            "joint1_velocity_cmd",
            "joint2_velocity_cmd",
            "joint3_velocity_cmd",
            "jacobian_det",
        ]

        self.get_logger().info(
            f"Arm yaw-rho-z velocity controller: target={self.target_frame}, EE={self.ee_frame}, "
            f"publishing {joint_velocity_topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        self.cmd_pub.publish(Twist())

        required_joints = (self.yaw_joint_name, "joint2", "joint3")
        missing = [name for name in required_joints if name not in self.current_joints]
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

        target_yaw = atan2(target_translation.y, target_translation.x)
        ee_yaw = atan2(ee_translation.y, ee_translation.x)
        joint1 = self.current_joints[self.yaw_joint_name]
        joint_yaw_error = normalize_angle(target_yaw - self.joint1_sign * joint1)
        ee_yaw_error = normalize_angle(target_yaw - ee_yaw)
        yaw_error = ee_yaw_error if self.yaw_error_source == "ee" else joint_yaw_error
        yaw_aligned = abs(yaw_error) <= self.yaw_tolerance

        target_rho = hypot(target_translation.x, target_translation.y)
        ee_rho = hypot(ee_translation.x, ee_translation.y)
        target_z = target_translation.z
        ee_z = ee_translation.z
        rho_error = target_rho - ee_rho
        z_error = target_z - ee_z
        rho_aligned = abs(rho_error) <= self.rho_tolerance
        z_aligned = abs(z_error) <= self.z_tolerance
        aligned = yaw_aligned and rho_aligned and z_aligned

        if yaw_aligned:
            joint1_velocity = 0.0
        else:
            joint1_velocity = self.joint1_sign * clamp(
                self.k_yaw * yaw_error,
                -self.max_yaw_velocity,
                self.max_yaw_velocity,
            )

        if aligned or (self.yaw_first and not yaw_aligned):
            joint2_velocity = 0.0
            joint3_velocity = 0.0
            jacobian_det = 0.0
        else:
            v_rho = clamp(self.k_rho * rho_error, -self.max_rho_velocity, self.max_rho_velocity)
            v_z = clamp(self.k_z * z_error, -self.max_z_velocity, self.max_z_velocity)
            if self.z_first and not z_aligned:
                v_rho = 0.0
            joint2_velocity, joint3_velocity, jacobian_det = self.solve_joint_velocities(v_rho, v_z)

        self.publish_joint_velocity(joint1_velocity, joint2_velocity, joint3_velocity)
        self.publish_debug(
            target_yaw,
            ee_yaw,
            joint_yaw_error,
            ee_yaw_error,
            joint1,
            yaw_error,
            target_rho,
            ee_rho,
            rho_error,
            target_z,
            ee_z,
            z_error,
            aligned,
            joint1_velocity,
            joint2_velocity,
            joint3_velocity,
            jacobian_det,
        )
        self.last_debug = (
            f"target_yaw={target_yaw:.3f}, ee_yaw={ee_yaw:.3f}, joint1={joint1:.3f}, "
            f"yaw_source={self.yaw_error_source}, yaw_error={yaw_error:.3f}, "
            f"rho_error={rho_error:.3f}, z_error={z_error:.3f}, aligned={aligned}, "
            f"joint_vel=[{joint1_velocity:.3f}, {joint2_velocity:.3f}, {joint3_velocity:.3f}], "
            f"det={jacobian_det:.4f}"
        )

    def solve_joint_velocities(self, v_rho: float, v_z: float) -> tuple[float, float, float]:
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

    def publish_stop(self) -> None:
        self.publish_joint_velocity(0.0, 0.0, 0.0)

    def publish_joint_velocity(self, joint1_velocity: float, joint2_velocity: float, joint3_velocity: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.velocity = [
            joint1_velocity,
            joint2_velocity,
            joint3_velocity,
            0.0,
            0.0,
            0.0,
        ]
        self.joint_pub.publish(msg)

    def publish_debug(
        self,
        target_yaw: float,
        ee_yaw: float,
        joint_yaw_error: float,
        ee_yaw_error: float,
        joint1: float,
        yaw_error: float,
        target_rho: float,
        ee_rho: float,
        rho_error: float,
        target_z: float,
        ee_z: float,
        z_error: float,
        aligned: bool,
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
            target_yaw,
            ee_yaw,
            joint_yaw_error,
            ee_yaw_error,
            joint1,
            yaw_error,
            target_rho,
            ee_rho,
            rho_error,
            target_z,
            ee_z,
            z_error,
            1.0 if aligned else 0.0,
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
    node = ArmYawRhoZVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
