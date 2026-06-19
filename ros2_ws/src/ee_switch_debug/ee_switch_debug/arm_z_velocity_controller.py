from math import sin

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class ArmZVelocityController(Node):
    def __init__(self) -> None:
        super().__init__("arm_z_velocity_controller")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_velocity_topic", "/joint_velocity_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/arm_z_debug")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)
        self.declare_parameter("k_z", 1.5)
        self.declare_parameter("max_z_velocity", 0.15)
        self.declare_parameter("max_joint_velocity", 0.4)
        self.declare_parameter("z_tolerance", 0.02)
        self.declare_parameter("singularity_epsilon", 0.05)

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
        self.k_z = float(self.get_parameter("k_z").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.z_tolerance = float(self.get_parameter("z_tolerance").value)
        self.singularity_epsilon = float(self.get_parameter("singularity_epsilon").value)

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
            "target_z",
            "ee_z",
            "z_error",
            "z_aligned",
            "v_z_cmd",
            "joint2_velocity_cmd",
            "joint3_velocity_cmd",
            "jacobian_det",
        ]

        self.get_logger().info(
            f"Arm z velocity controller: target={self.target_frame}, EE={self.ee_frame}, "
            f"publishing {joint_velocity_topic}"
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

        target_z = target_transform.transform.translation.z
        ee_z = ee_transform.transform.translation.z
        z_error = target_z - ee_z
        z_aligned = abs(z_error) <= self.z_tolerance

        if z_aligned:
            v_z = 0.0
            joint2_velocity = 0.0
            joint3_velocity = 0.0
            jacobian_det = 0.0
        else:
            v_z = clamp(self.k_z * z_error, -self.max_z_velocity, self.max_z_velocity)
            joint2_velocity, joint3_velocity, jacobian_det = self.solve_joint_velocities(v_z)

        self.publish_joint_velocity(joint2_velocity, joint3_velocity)
        self.publish_debug(target_z, ee_z, z_error, z_aligned, v_z, joint2_velocity, joint3_velocity, jacobian_det)
        self.last_debug = (
            f"target_z={target_z:.3f}, ee_z={ee_z:.3f}, z_error={z_error:.3f}, "
            f"aligned={z_aligned}, v_z={v_z:.3f}, "
            f"joint2_vel={joint2_velocity:.3f}, joint3_vel={joint3_velocity:.3f}, "
            f"det={jacobian_det:.4f}"
        )

    def solve_joint_velocities(self, v_z: float) -> tuple[float, float, float]:
        q2 = self.current_joints["joint2"] / self.joint2_sign
        q3 = self.current_joints["joint3"] / self.joint3_sign
        det = self.link1 * self.link2 * sin(q3)

        if abs(det) < self.singularity_epsilon:
            self.get_logger().warn(f"Near singularity for z control: det={det:.4f}. Sending zero velocity.")
            return 0.0, 0.0, det

        # Solve [rho_dot=0, z_dot=v_z] using the 2-link rho-z Jacobian.
        j11 = -self.link1 * sin(q2) - self.link2 * sin(q2 + q3)
        j12 = -self.link2 * sin(q2 + q3)
        q2_dot = -j12 * v_z / det
        q3_dot = j11 * v_z / det

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
        target_z: float,
        ee_z: float,
        z_error: float,
        z_aligned: bool,
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
            target_z,
            ee_z,
            z_error,
            1.0 if z_aligned else 0.0,
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
    node = ArmZVelocityController()
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
