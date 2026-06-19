from math import acos, atan2, hypot, sin, tanh

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class TargetArmIkController(Node):
    def __init__(self) -> None:
        super().__init__("target_arm_ik_controller")

        self.declare_parameter("base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("output_topic", "/target_joint_command")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("switching_point", 0.5)
        self.declare_parameter("alpha", 5.0)
        self.declare_parameter("shoulder_z", 0.1715)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)
        self.declare_parameter("min_reach", 0.05)
        self.declare_parameter("scale_by_mu", True)
        self.declare_parameter("joint1_sign", 1.0)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)
        self.declare_parameter("joint_kp", 0.35)
        self.declare_parameter("max_joint_step", 0.04)
        self.declare_parameter("ee_stop_distance", 0.03)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        output_topic = self.get_parameter("output_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.switching_point = float(self.get_parameter("switching_point").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.shoulder_z = float(self.get_parameter("shoulder_z").value)
        self.link1 = float(self.get_parameter("link1").value)
        self.link2 = float(self.get_parameter("link2").value)
        self.min_reach = float(self.get_parameter("min_reach").value)
        self.scale_by_mu = bool(self.get_parameter("scale_by_mu").value)
        self.joint1_sign = float(self.get_parameter("joint1_sign").value)
        self.joint2_sign = float(self.get_parameter("joint2_sign").value)
        self.joint3_sign = float(self.get_parameter("joint3_sign").value)
        self.joint_kp = float(self.get_parameter("joint_kp").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.ee_stop_distance = float(self.get_parameter("ee_stop_distance").value)
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.joint_pub = self.create_publisher(JointState, output_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)

        self.last_debug = "waiting for transform"

        self.get_logger().info(
            f"Arm IK controller: {self.base_frame} -> {self.target_frame}, "
            f"EE={self.ee_frame}, publishing {output_topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        try:
            target_transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            ee_transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.last_debug = f"lookup failed: {exc}"
            return

        missing_joints = [name for name in self.joint_names if name not in self.current_joints]
        if missing_joints:
            self.last_debug = f"waiting for joint states: {', '.join(missing_joints)}"
            return

        t = target_transform.transform.translation
        ee = ee_transform.transform.translation
        ee_error = ((t.x - ee.x) ** 2 + (t.y - ee.y) ** 2 + (t.z - ee.z) ** 2) ** 0.5
        rho_t = hypot(t.x, t.y)
        mu = 0.5 * (1.0 - tanh(self.alpha * (rho_t - self.switching_point)))
        weight = mu if self.scale_by_mu else 1.0

        yaw = atan2(t.y, t.x)
        z = t.z - self.shoulder_z
        rho = clamp(rho_t, self.min_reach, self.link1 + self.link2 - 1e-3)

        q2, q3 = self.solve_planar_2link(rho, z)
        q1 = self.joint1_sign * yaw
        q2 = self.joint2_sign * q2
        q3 = self.joint3_sign * q3
        ik_target = {
            "joint1": q1,
            "joint2": q2,
            "joint3": q3,
        }

        if ee_error <= self.ee_stop_distance:
            command = [self.current_joints[name] for name in self.joint_names]
        else:
            command = []
            for name in self.joint_names:
                current = self.current_joints[name]
                desired = ik_target.get(name, current)
                step = clamp(
                    weight * self.joint_kp * (desired - current),
                    -self.max_joint_step,
                    self.max_joint_step,
                )
                command.append(current + step)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = command
        self.joint_pub.publish(msg)

        self.last_debug = (
            f"target=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), "
            f"ee=({ee.x:.3f}, {ee.y:.3f}, {ee.z:.3f}), "
            f"err={ee_error:.3f}, rho_t={rho_t:.3f}, mu={mu:.3f}, "
            f"q=[{msg.position[0]:.2f}, {msg.position[1]:.2f}, "
            f"{msg.position[2]:.2f}], hold456=[{msg.position[3]:.2f}, "
            f"{msg.position[4]:.2f}, {msg.position[5]:.2f}]"
        )

    def solve_planar_2link(self, rho: float, z: float) -> tuple[float, float]:
        r2 = rho**2 + z**2
        c2 = (r2 - self.link1**2 - self.link2**2) / (2.0 * self.link1 * self.link2)
        q3 = acos(clamp(c2, -1.0, 1.0))
        beta = atan2(self.link2 * sin(q3), self.link1 + self.link2 * c2)
        alpha = atan2(z, rho)
        q2 = alpha - beta
        return q2, q3

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetArmIkController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
