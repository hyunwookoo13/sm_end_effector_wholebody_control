from math import atan2, pi

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


class ArmYawVelocityController(Node):
    def __init__(self) -> None:
        super().__init__("arm_yaw_velocity_controller")

        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("target_frame", "target_out")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("joint_velocity_topic", "/joint_velocity_command")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/arm_yaw_debug")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("yaw_joint_name", "joint1")
        self.declare_parameter("joint1_sign", 1.0)
        self.declare_parameter("k_yaw", 1.5)
        self.declare_parameter("max_yaw_velocity", 0.5)
        self.declare_parameter("yaw_tolerance", 0.03)

        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        joint_velocity_topic = self.get_parameter("joint_velocity_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        debug_topic = self.get_parameter("debug_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.yaw_joint_name = self.get_parameter("yaw_joint_name").value
        self.joint1_sign = float(self.get_parameter("joint1_sign").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.max_yaw_velocity = float(self.get_parameter("max_yaw_velocity").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)

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
            "target_x",
            "target_y",
            "target_yaw",
            "joint1",
            "yaw_error",
            "yaw_aligned",
            "joint1_velocity_cmd",
        ]

        self.get_logger().info(
            f"Arm yaw velocity controller: {self.arm_base_frame} -> {self.target_frame}, "
            f"publishing {joint_velocity_topic}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        self.cmd_pub.publish(Twist())

        if self.yaw_joint_name not in self.current_joints:
            self.last_debug = f"waiting for joint state: {self.yaw_joint_name}"
            self.publish_stop()
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.last_debug = f"lookup failed: {exc}"
            self.publish_stop()
            return

        t = transform.transform.translation
        target_yaw = atan2(t.y, t.x)
        joint1 = self.current_joints[self.yaw_joint_name]
        yaw_error = normalize_angle(target_yaw - self.joint1_sign * joint1)
        yaw_aligned = abs(yaw_error) <= self.yaw_tolerance

        if yaw_aligned:
            joint1_velocity = 0.0
        else:
            joint1_velocity = self.joint1_sign * clamp(
                self.k_yaw * yaw_error,
                -self.max_yaw_velocity,
                self.max_yaw_velocity,
            )

        self.publish_joint_velocity(joint1_velocity)
        self.publish_debug(t.x, t.y, target_yaw, joint1, yaw_error, yaw_aligned, joint1_velocity)
        self.last_debug = (
            f"target=({t.x:.3f}, {t.y:.3f}), target_yaw={target_yaw:.3f}, "
            f"joint1={joint1:.3f}, yaw_error={yaw_error:.3f}, "
            f"aligned={yaw_aligned}, joint1_vel={joint1_velocity:.3f}"
        )

    def publish_stop(self) -> None:
        self.publish_joint_velocity(0.0)

    def publish_joint_velocity(self, joint1_velocity: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.velocity = [
            joint1_velocity,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.joint_pub.publish(msg)

    def publish_debug(
        self,
        target_x: float,
        target_y: float,
        target_yaw: float,
        joint1: float,
        yaw_error: float,
        yaw_aligned: bool,
        joint1_velocity: float,
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
            target_x,
            target_y,
            target_yaw,
            joint1,
            yaw_error,
            1.0 if yaw_aligned else 0.0,
            joint1_velocity,
        ]
        self.debug_pub.publish(msg)

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmYawVelocityController()
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
