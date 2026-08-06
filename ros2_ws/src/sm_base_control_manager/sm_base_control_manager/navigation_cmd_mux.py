import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


def selected_source(mode: str) -> str | None:
    normalized = str(mode).strip().upper()
    if normalized == "NAVIGATION":
        return "navigation"
    if normalized == "MANIPULATION":
        return "manipulation"
    if normalized == "RETREAT":
        return "retreat"
    if normalized == "HYBRID":
        return "hybrid"
    return None


def smooth_blend_value(navigation: float, manipulation: float, weight: float) -> float:
    precision_weight = max(0.0, min(1.0, float(weight)))
    return (1.0 - precision_weight) * float(navigation) + precision_weight * float(manipulation)


def slew_value(current: float, target: float, max_rate: float, dt: float) -> float:
    max_delta = max(0.0, float(max_rate)) * max(0.0, float(dt))
    delta = float(target) - float(current)
    if delta > max_delta:
        return float(current) + max_delta
    if delta < -max_delta:
        return float(current) - max_delta
    return float(target)


class NavigationCmdMux(Node):
    def __init__(self) -> None:
        super().__init__("navigation_cmd_mux")
        self.declare_parameter("navigation_cmd_topic", "/cmd_vel_navigation")
        self.declare_parameter("manipulation_cmd_topic", "/cmd_vel_manipulation")
        self.declare_parameter("retreat_cmd_topic", "/cmd_vel_retreat")
        self.declare_parameter("output_cmd_topic", "/cmd_vel")
        self.declare_parameter("mode_topic", "/base_control_mode")
        self.declare_parameter("hybrid_blend_topic", "/base_control_blend")
        self.declare_parameter("command_timeout_sec", 0.25)
        self.declare_parameter("rate_hz", 40.0)
        self.declare_parameter("max_linear_velocity_mps", 1.50)
        self.declare_parameter("max_angular_velocity_rps", 1.40)
        self.declare_parameter("max_linear_acceleration_mps2", 2.00)
        self.declare_parameter("max_angular_acceleration_rps2", 2.00)

        self.mode = "STOP"
        self.hybrid_weight = 0.0
        self.commands: dict[str, tuple[Twist, int] | None] = {
            "navigation": None,
            "manipulation": None,
            "retreat": None,
        }
        self.command_timeout_ns = int(
            max(0.01, float(self.get_parameter("command_timeout_sec").value)) * 1e9
        )
        self.max_linear_velocity = max(
            0.0, float(self.get_parameter("max_linear_velocity_mps").value)
        )
        self.max_angular_velocity = max(
            0.0, float(self.get_parameter("max_angular_velocity_rps").value)
        )
        self.max_linear_acceleration = max(
            0.0, float(self.get_parameter("max_linear_acceleration_mps2").value)
        )
        self.max_angular_acceleration = max(
            0.0, float(self.get_parameter("max_angular_acceleration_rps2").value)
        )
        self.last_output = Twist()
        self.last_publish_ns = self.get_clock().now().nanoseconds
        self.publisher = self.create_publisher(
            Twist,
            str(self.get_parameter("output_cmd_topic").value),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("navigation_cmd_topic").value),
            lambda msg: self._store_command("navigation", msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("manipulation_cmd_topic").value),
            lambda msg: self._store_command("manipulation", msg),
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("retreat_cmd_topic").value),
            lambda msg: self._store_command("retreat", msg),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("mode_topic").value),
            self._on_mode,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter("hybrid_blend_topic").value),
            self._on_hybrid_weight,
            10,
        )
        rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._publish_selected)

    def _store_command(self, source: str, msg: Twist) -> None:
        self.commands[source] = (msg, self.get_clock().now().nanoseconds)

    def _on_mode(self, msg: String) -> None:
        self.mode = msg.data.strip().upper()

    def _on_hybrid_weight(self, msg: Float32) -> None:
        self.hybrid_weight = max(0.0, min(1.0, float(msg.data)))

    def _fresh_command(self, source: str, now_ns: int) -> Twist | None:
        command = self.commands.get(source)
        if command is None:
            return None
        msg, stamp_ns = command
        if now_ns - stamp_ns > self.command_timeout_ns:
            return None
        return msg

    def _hybrid_command(self, now_ns: int) -> Twist:
        navigation = self._fresh_command("navigation", now_ns)
        manipulation = self._fresh_command("manipulation", now_ns)
        if navigation is None and manipulation is None:
            return Twist()
        if manipulation is None:
            return navigation
        if navigation is None:
            return manipulation

        output = Twist()
        output.linear.x = smooth_blend_value(
            navigation.linear.x, manipulation.linear.x, self.hybrid_weight
        )
        output.linear.y = smooth_blend_value(
            navigation.linear.y, manipulation.linear.y, self.hybrid_weight
        )
        output.angular.z = smooth_blend_value(
            navigation.angular.z, manipulation.angular.z, self.hybrid_weight
        )
        return output

    def _limit_output(self, target: Twist, now_ns: int) -> Twist:
        dt = max(0.0, min(0.2, (now_ns - self.last_publish_ns) * 1e-9))
        target_linear = max(
            -self.max_linear_velocity,
            min(self.max_linear_velocity, float(target.linear.x)),
        )
        target_angular = max(
            -self.max_angular_velocity,
            min(self.max_angular_velocity, float(target.angular.z)),
        )
        output = Twist()
        output.linear.x = slew_value(
            self.last_output.linear.x,
            target_linear,
            self.max_linear_acceleration,
            dt,
        )
        output.angular.z = slew_value(
            self.last_output.angular.z,
            target_angular,
            self.max_angular_acceleration,
            dt,
        )
        self.last_output = output
        self.last_publish_ns = now_ns
        return output

    def _publish_selected(self) -> None:
        source = selected_source(self.mode)
        now_ns = self.get_clock().now().nanoseconds
        if source == "hybrid":
            target = self._hybrid_command(now_ns)
        elif source is None:
            self.last_output = Twist()
            self.last_publish_ns = now_ns
            self.publisher.publish(Twist())
            return
        else:
            target = self._fresh_command(source, now_ns) or Twist()
        self.publisher.publish(self._limit_output(target, now_ns))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationCmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
