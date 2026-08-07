from math import hypot

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


class ArmJointEffectProbe(Node):
    def __init__(self) -> None:
        super().__init__("arm_joint_effect_probe")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_velocity_topic", "/joint_velocity_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("probe_joint", "joint2")
        self.declare_parameter("probe_velocity", 0.12)
        self.declare_parameter("probe_duration", 0.6)
        self.declare_parameter("settle_duration", 0.2)

        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        joint_velocity_topic = self.get_parameter("joint_velocity_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.probe_joint = self.get_parameter("probe_joint").value
        self.probe_velocity = float(self.get_parameter("probe_velocity").value)
        self.probe_duration = float(self.get_parameter("probe_duration").value)
        self.settle_duration = float(self.get_parameter("settle_duration").value)

        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.current_joints: dict[str, float] = {}
        self.start_sample: tuple[float, float, float] | None = None
        self.probe_start_time: rclpy.time.Time | None = None
        self.stop_start_time: rclpy.time.Time | None = None
        self.phase = "waiting"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.joint_pub = self.create_publisher(JointState, joint_velocity_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)

        if self.probe_joint not in self.joint_names:
            raise ValueError(f"probe_joint must be one of {self.joint_names}")

        self.get_logger().info(
            f"Probing {self.probe_joint} at {self.probe_velocity:.3f} rad/s for "
            f"{self.probe_duration:.2f}s. Base is held at zero cmd_vel."
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        self.cmd_pub.publish(Twist())

        if self.probe_joint not in self.current_joints:
            self.publish_joint_velocity(0.0)
            return

        sample = self.read_sample()
        if sample is None:
            self.publish_joint_velocity(0.0)
            return

        now = self.get_clock().now()

        if self.phase == "waiting":
            self.start_sample = sample
            self.probe_start_time = now
            self.phase = "probing"
            rho, z, q = sample
            self.get_logger().info(f"start: rho={rho:.4f}, z={z:.4f}, {self.probe_joint}={q:.4f}")

        if self.phase == "probing":
            assert self.probe_start_time is not None
            elapsed = (now - self.probe_start_time).nanoseconds * 1e-9
            if elapsed < self.probe_duration:
                self.publish_joint_velocity(self.probe_velocity)
                return
            self.stop_start_time = now
            self.phase = "settling"
            self.publish_joint_velocity(0.0)
            return

        if self.phase == "settling":
            assert self.stop_start_time is not None
            elapsed = (now - self.stop_start_time).nanoseconds * 1e-9
            self.publish_joint_velocity(0.0)
            if elapsed < self.settle_duration:
                return
            self.report(sample)
            self.phase = "done"
            return

        self.publish_joint_velocity(0.0)

    def read_sample(self) -> tuple[float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None

        t = transform.transform.translation
        rho = hypot(t.x, t.y)
        z = t.z
        q = self.current_joints[self.probe_joint]
        return rho, z, q

    def report(self, end_sample: tuple[float, float, float]) -> None:
        assert self.start_sample is not None
        start_rho, start_z, start_q = self.start_sample
        end_rho, end_z, end_q = end_sample
        delta_q = end_q - start_q
        delta_rho = end_rho - start_rho
        delta_z = end_z - start_z
        rho_per_rad = delta_rho / delta_q if abs(delta_q) > 1e-6 else 0.0
        z_per_rad = delta_z / delta_q if abs(delta_q) > 1e-6 else 0.0

        self.get_logger().info(
            f"end: rho={end_rho:.4f}, z={end_z:.4f}, {self.probe_joint}={end_q:.4f}"
        )
        self.get_logger().info(
            f"delta: d_rho={delta_rho:.4f}, d_z={delta_z:.4f}, d_q={delta_q:.4f}, "
            f"d_rho/dq={rho_per_rad:.4f}, d_z/dq={z_per_rad:.4f}"
        )
        self.get_logger().info("Probe complete. Stop this node with Ctrl+C before running the next probe.")

    def publish_joint_velocity(self, probe_velocity: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.velocity = [0.0] * len(self.joint_names)
        msg.velocity[self.joint_names.index(self.probe_joint)] = probe_velocity
        self.joint_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmJointEffectProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
            node.publish_joint_velocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
