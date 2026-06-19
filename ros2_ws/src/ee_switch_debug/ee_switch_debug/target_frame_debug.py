from math import sqrt, tanh
from typing import List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class TargetFrameDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("target_frame_debug")

        self.declare_parameter("base_frame", "base")
        self.declare_parameter(
            "target_frames",
            ["target_in", "target_mid", "target_out"],
        )
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("workspace_radius", 1.0)
        self.declare_parameter("switching_point", 0.5)
        self.declare_parameter("alpha", 5.0)

        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.target_frames: List[str] = [
            frame
            for frame in self.get_parameter("target_frames")
            .get_parameter_value()
            .string_array_value
        ]
        rate_hz = self.get_parameter("rate_hz").get_parameter_value().double_value
        self.workspace_radius = (
            self.get_parameter("workspace_radius").get_parameter_value().double_value
        )
        self.switching_point = (
            self.get_parameter("switching_point").get_parameter_value().double_value
        )
        self.alpha = self.get_parameter("alpha").get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.1), self.on_timer)

        self.get_logger().info(
            f"Reading target frames relative to '{self.base_frame}': {', '.join(self.target_frames)}"
        )

    def on_timer(self) -> None:
        lines = []
        for target_frame in self.target_frames:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    target_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as exc:
                lines.append(f"{target_frame}: lookup failed ({exc})")
                continue

            t = transform.transform.translation
            rho_t = sqrt(t.x**2 + t.y**2)
            rho_w = sqrt(max(self.workspace_radius**2 - t.z**2, 0.0))
            mu = 0.5 * (1.0 - tanh(self.alpha * (rho_t - self.switching_point)))
            lines.append(
                f"{target_frame}: x={t.x:.3f}, y={t.y:.3f}, z={t.z:.3f}, "
                f"rho_t={rho_t:.3f}, rho_w={rho_w:.3f}, mu={mu:.3f}"
            )

        self.get_logger().info(" | ".join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetFrameDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
