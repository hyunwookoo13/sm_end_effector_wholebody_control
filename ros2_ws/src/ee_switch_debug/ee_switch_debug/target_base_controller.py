from math import atan2, hypot, tanh

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class TargetBaseController(Node):
    def __init__(self) -> None:
        super().__init__("target_base_controller")

        self.declare_parameter("base_frame", "chassis_link")
        self.declare_parameter("target_frame", "target_out")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("switching_point", 0.5)
        self.declare_parameter("alpha", 5.0)
        self.declare_parameter("k_linear", 1.6)
        self.declare_parameter("k_angular", 2.0)
        self.declare_parameter("max_linear", 1.6)
        self.declare_parameter("max_angular", 2.0)
        self.declare_parameter("stop_distance", 0.05)

        self.base_frame = self.get_parameter("base_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        cmd_topic = self.get_parameter("cmd_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.switching_point = float(self.get_parameter("switching_point").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.k_linear = float(self.get_parameter("k_linear").value)
        self.k_angular = float(self.get_parameter("k_angular").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)

        self.last_debug = "waiting for transform"

        self.get_logger().info(
            f"Base-only controller: {self.base_frame} -> {self.target_frame}, publishing {cmd_topic}"
        )

    def on_timer(self) -> None:
        twist = Twist()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.last_debug = f"lookup failed: {exc}"
            self.cmd_pub.publish(twist)
            return

        t = transform.transform.translation
        rho_t = hypot(t.x, t.y)
        mu = 0.5 * (1.0 - tanh(self.alpha * (rho_t - self.switching_point)))
        base_weight = 1.0 - mu

        target_yaw = atan2(t.y, t.x)
        if rho_t > self.stop_distance:
            twist.linear.x = base_weight * self.k_linear * t.x
            twist.angular.z = base_weight * self.k_angular * target_yaw

        twist.linear.x = clamp(twist.linear.x, -self.max_linear, self.max_linear)
        twist.angular.z = clamp(twist.angular.z, -self.max_angular, self.max_angular)

        self.cmd_pub.publish(twist)
        self.last_debug = (
            f"x={t.x:.3f}, y={t.y:.3f}, rho_t={rho_t:.3f}, mu={mu:.3f}, "
            f"cmd_v={twist.linear.x:.3f}, cmd_w={twist.angular.z:.3f}"
        )

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetBaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
