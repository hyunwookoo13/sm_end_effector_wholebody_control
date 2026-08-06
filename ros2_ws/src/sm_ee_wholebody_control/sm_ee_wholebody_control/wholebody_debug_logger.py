import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class WholebodyDebugLogger(Node):
    def __init__(self) -> None:
        super().__init__("wholebody_debug_logger")

        self.declare_parameter("debug_topic", "/wholebody_debug")
        self.declare_parameter(
            "output_dir",
            "/home/kiro/Desktop/hw_ws/ros2_ws/debug_logs",
        )
        self.declare_parameter("file_prefix", "wholebody_debug")
        self.declare_parameter("flush_every", 10)

        debug_topic = self.get_parameter("debug_topic").value
        output_dir = Path(self.get_parameter("output_dir").value).expanduser()
        file_prefix = self.get_parameter("file_prefix").value
        self.flush_every = int(self.get_parameter("flush_every").value)

        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = output_dir / f"{file_prefix}_{timestamp}.csv"
        self.file = self.csv_path.open("w", newline="")
        self.writer = None
        self.labels = None
        self.sample_count = 0
        self.start_time_ns = self.get_clock().now().nanoseconds

        self.create_subscription(Float64MultiArray, debug_topic, self.on_debug, 10)
        self.get_logger().info(f"Logging {debug_topic} to {self.csv_path}")

    def on_debug(self, msg: Float64MultiArray) -> None:
        now_ns = self.get_clock().now().nanoseconds
        ros_time_s = now_ns * 1e-9
        elapsed_s = (now_ns - self.start_time_ns) * 1e-9
        labels = self.get_labels(msg)

        if self.writer is None:
            self.labels = labels
            self.writer = csv.writer(self.file)
            self.writer.writerow(["ros_time_s", "elapsed_s", "sample", *self.labels])
        elif labels != self.labels:
            self.get_logger().warn("Debug labels changed during logging. Keeping original header.")

        row = [ros_time_s, elapsed_s, self.sample_count, *msg.data[: len(self.labels)]]
        self.writer.writerow(row)
        self.sample_count += 1

        if self.flush_every > 0 and self.sample_count % self.flush_every == 0:
            self.file.flush()

    def get_labels(self, msg: Float64MultiArray) -> list[str]:
        if msg.layout.dim and msg.layout.dim[0].label:
            labels = [label.strip() for label in msg.layout.dim[0].label.split(",")]
            if len(labels) == len(msg.data):
                return labels

        return [f"value_{index}" for index in range(len(msg.data))]

    def close(self) -> None:
        if not self.file.closed:
            self.file.flush()
            self.file.close()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WholebodyDebugLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info(f"Saved {node.sample_count} samples to {node.csv_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
