from math import atan2, hypot

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class MobileTargetController(Node):
    def __init__(self) -> None:
        super().__init__("mobile_target_controller")

        self.declare_parameter("base_frame", "chassis_link")
        self.declare_parameter("target_frame", "target_out")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("debug_topic", "/mobile_target_debug")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("yaw_only", True)
        self.declare_parameter("k_yaw", 2.5)
        self.declare_parameter("k_linear", 0.8)
        self.declare_parameter("max_yaw_rate", 1.0)
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("yaw_tolerance", 0.05)
        self.declare_parameter("stop_distance", 0.15)
        self.declare_parameter("arrival_distance", 0.25)

        self.base_frame = self.get_parameter("base_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        cmd_topic = self.get_parameter("cmd_topic").value
        debug_topic = self.get_parameter("debug_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.yaw_only = bool(self.get_parameter("yaw_only").value)
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.k_linear = float(self.get_parameter("k_linear").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.yaw_tolerance = float(self.get_parameter("yaw_tolerance").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.arrival_distance = float(self.get_parameter("arrival_distance").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, debug_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)
        self.last_debug = "waiting for transform"
        self.debug_labels = [
            "target_x",
            "target_y",
            "target_z",
            "rho_xy",
            "yaw_error",
            "yaw_aligned",
            "arrived",
            "cmd_v",
            "cmd_w",
        ]

        self.get_logger().info(
            f"Mobile target controller: {self.base_frame} -> {self.target_frame}, "
            f"yaw_only={self.yaw_only}, publishing {cmd_topic}"
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
        rho_xy = hypot(t.x, t.y)
        yaw_error = atan2(t.y, t.x)
        yaw_aligned = abs(yaw_error) <= self.yaw_tolerance
        arrived = rho_xy <= self.arrival_distance

        if not arrived and rho_xy > self.stop_distance:
            twist.angular.z = clamp(
                self.k_yaw * yaw_error,
                -self.max_yaw_rate,
                self.max_yaw_rate,
            )

            if not self.yaw_only and yaw_aligned:
                twist.linear.x = clamp(
                    self.k_linear * (rho_xy - self.stop_distance),
                    0.0,
                    self.max_linear,
                )

        self.cmd_pub.publish(twist)
        self.publish_debug(t.x, t.y, t.z, rho_xy, yaw_error, yaw_aligned, arrived, twist)
        self.last_debug = (
            f"target=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), "
            f"rho_xy={rho_xy:.3f}, yaw_error={yaw_error:.3f}, "
            f"aligned={yaw_aligned}, arrived={arrived}, cmd_v={twist.linear.x:.3f}, "
            f"cmd_w={twist.angular.z:.3f}"
        )

    def publish_debug(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        rho_xy: float,
        yaw_error: float,
        yaw_aligned: bool,
        arrived: bool,
        twist: Twist,
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
            target_z,
            rho_xy,
            yaw_error,
            1.0 if yaw_aligned else 0.0,
            1.0 if arrived else 0.0,
            twist.linear.x,
            twist.angular.z,
        ]
        self.debug_pub.publish(msg)

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MobileTargetController()
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
