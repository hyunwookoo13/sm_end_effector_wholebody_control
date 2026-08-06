import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


class NaturalLanguageTaskConsole(Node):
    """Publish typed natural-language commands to the task parser."""

    def __init__(self) -> None:
        super().__init__("natural_language_task_console")

        self.declare_parameter("natural_language_topic", "/natural_language_task")
        self.declare_parameter("status_topic", "/natural_language_task_status")
        self.declare_parameter("wait_for_parser", True)
        self.declare_parameter("subscriber_wait_sec", 5.0)

        self.natural_language_topic = str(self.get_parameter("natural_language_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.wait_for_parser = bool(self.get_parameter("wait_for_parser").value)
        self.subscriber_wait_sec = float(self.get_parameter("subscriber_wait_sec").value)

        self.publisher = self.create_publisher(String, self.natural_language_topic, 10)
        self.create_subscription(String, self.status_topic, self.on_status, 10)

    def wait_until_ready(self) -> None:
        if not self.wait_for_parser:
            return
        deadline = time.monotonic() + self.subscriber_wait_sec
        while rclpy.ok() and self.count_subscribers(self.natural_language_topic) == 0:
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f"No subscribers on {self.natural_language_topic}; "
                    "commands will still be published."
                )
                return
            time.sleep(0.1)

    def publish_command(self, text: str) -> None:
        self.publisher.publish(String(data=text))
        self.get_logger().info(f"Published natural-language task: {text}")

    def on_status(self, msg: String) -> None:
        self.get_logger().info(f"Parser status: {msg.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaturalLanguageTaskConsole()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.wait_until_ready()
        print("Type a pick-and-place command, then press Enter.")
        print("Example: can to box / Korean input is also supported.")
        print("Type 'exit' or 'quit' to stop.")
        while rclpy.ok():
            try:
                text = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not text:
                continue
            if text.lower() in {"exit", "quit", "q"}:
                break
            node.publish_command(text)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
