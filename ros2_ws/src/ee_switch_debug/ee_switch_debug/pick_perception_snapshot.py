from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, String


class PickPerceptionSnapshot(Node):
    """Own one immutable ROI/grasp bundle for the active PICK task."""

    LIVE = "LIVE"
    WAITING_ROI = "WAITING_ROI"
    WAITING_GRASP = "WAITING_GRASP"
    LOCKED = "LOCKED"

    def __init__(self) -> None:
        super().__init__("pick_perception_snapshot")

        self.declare_parameter(
            "live_roi_topic",
            "/sm_florence_2_vlm/roi_pointcloud_live",
        )
        self.declare_parameter(
            "output_roi_topic",
            "/sm_florence_2_vlm/roi_pointcloud",
        )
        self.declare_parameter(
            "live_grasp_topic",
            "/sm_grasping/grasp_best_live",
        )
        self.declare_parameter(
            "output_grasp_topic",
            "/sm_grasping/grasp_best",
        )
        self.declare_parameter(
            "live_opening_topic",
            "/sm_grasping/grasp_openings_live",
        )
        self.declare_parameter(
            "output_opening_topic",
            "/sm_grasping/grasp_openings",
        )
        self.declare_parameter("task_command_topic", "/arm_task_command")
        self.declare_parameter("task_state_topic", "/arm_task_state")
        self.declare_parameter("grasp_settle_sec", 0.4)
        self.declare_parameter("publish_rate_hz", 15.0)

        live_roi_topic = str(self.get_parameter("live_roi_topic").value)
        output_roi_topic = str(self.get_parameter("output_roi_topic").value)
        live_grasp_topic = str(self.get_parameter("live_grasp_topic").value)
        output_grasp_topic = str(self.get_parameter("output_grasp_topic").value)
        live_opening_topic = str(self.get_parameter("live_opening_topic").value)
        output_opening_topic = str(self.get_parameter("output_opening_topic").value)
        task_command_topic = str(self.get_parameter("task_command_topic").value)
        task_state_topic = str(self.get_parameter("task_state_topic").value)
        settle_sec = max(0.0, float(self.get_parameter("grasp_settle_sec").value))
        publish_rate_hz = max(
            0.1,
            float(self.get_parameter("publish_rate_hz").value),
        )

        self.state = self.LIVE
        self.grasp_settle_ns = int(settle_sec * 1_000_000_000)
        self.locked_roi: PointCloud2 | None = None
        self.locked_grasp: PoseStamped | None = None
        self.locked_opening: Float32MultiArray | None = None
        self.pending_opening: Float32MultiArray | None = None
        self.roi_snapshot_time_ns = 0

        self.roi_pub = self.create_publisher(PointCloud2, output_roi_topic, 10)
        self.grasp_pub = self.create_publisher(PoseStamped, output_grasp_topic, 10)
        self.opening_pub = self.create_publisher(
            Float32MultiArray,
            output_opening_topic,
            10,
        )
        self.create_subscription(PointCloud2, live_roi_topic, self.on_live_roi, 10)
        self.create_subscription(
            PoseStamped,
            live_grasp_topic,
            self.on_live_grasp,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            live_opening_topic,
            self.on_live_opening,
            10,
        )
        self.create_subscription(
            String,
            task_command_topic,
            self.on_task_command,
            10,
        )
        self.create_subscription(
            String,
            task_state_topic,
            self.on_task_state,
            10,
        )
        self.create_timer(1.0 / publish_rate_hz, self.on_timer)

        self.get_logger().info(
            "PICK perception snapshot relay ready: "
            f"{live_roi_topic} -> {output_roi_topic}, "
            f"{live_grasp_topic} -> {output_grasp_topic}"
        )

    def on_task_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "PICK":
            self.begin_pick()
        elif command == "RESET":
            self.release("RESET command")

    def on_task_state(self, msg: String) -> None:
        state = msg.data.strip().upper()
        if state in ("PICK:HOLD", "PICK:DONE"):
            self.release(state)

    def begin_pick(self) -> None:
        self.state = self.WAITING_ROI
        self.locked_roi = None
        self.locked_grasp = None
        self.locked_opening = None
        self.pending_opening = None
        self.roi_snapshot_time_ns = 0
        self.get_logger().warn(
            "PICK perception boundary: discarded previous ROI/grasp bundle; "
            "waiting for first fresh ROI"
        )

    def release(self, reason: str) -> None:
        self.state = self.LIVE
        self.locked_roi = None
        self.locked_grasp = None
        self.locked_opening = None
        self.pending_opening = None
        self.roi_snapshot_time_ns = 0
        self.get_logger().info(f"Released PICK perception bundle: {reason}")

    def on_live_roi(self, msg: PointCloud2) -> None:
        if self.state == self.LIVE:
            self.roi_pub.publish(deepcopy(msg))
            return
        if self.state != self.WAITING_ROI:
            return
        if not self.has_points(msg):
            self.get_logger().debug("Ignored empty ROI while waiting for PICK snapshot")
            return

        self.locked_roi = deepcopy(msg)
        self.roi_snapshot_time_ns = self.get_clock().now().nanoseconds
        self.state = self.WAITING_GRASP
        self.publish_locked_roi()
        self.get_logger().warn(
            "Locked first post-PICK ROI; waiting for grasp generated from snapshot"
        )

    def on_live_opening(self, msg: Float32MultiArray) -> None:
        if self.state == self.LIVE:
            self.opening_pub.publish(deepcopy(msg))
        elif self.state == self.WAITING_GRASP:
            self.pending_opening = deepcopy(msg)

    def on_live_grasp(self, msg: PoseStamped) -> None:
        if self.state == self.LIVE:
            self.grasp_pub.publish(deepcopy(msg))
            return
        if self.state != self.WAITING_GRASP:
            return
        if not msg.header.frame_id:
            self.get_logger().debug("Ignored grasp with empty frame_id")
            return

        elapsed_ns = (
            self.get_clock().now().nanoseconds - self.roi_snapshot_time_ns
        )
        if elapsed_ns < self.grasp_settle_ns:
            self.get_logger().debug(
                "Ignored in-flight grasp during post-PICK settle window"
            )
            return

        self.locked_grasp = deepcopy(msg)
        self.locked_opening = (
            deepcopy(self.pending_opening)
            if self.pending_opening is not None
            else Float32MultiArray(data=[])
        )
        self.state = self.LOCKED
        self.publish_locked_bundle()
        position = self.locked_grasp.pose.position
        self.get_logger().warn(
            "Locked PICK perception bundle: "
            f"grasp=[{position.x:.3f}, {position.y:.3f}, {position.z:.3f}]"
        )

    def on_timer(self) -> None:
        if self.state == self.WAITING_GRASP:
            self.publish_locked_roi()
        elif self.state == self.LOCKED:
            self.publish_locked_bundle()

    def publish_locked_roi(self) -> None:
        if self.locked_roi is None:
            return
        roi = deepcopy(self.locked_roi)
        roi.header.stamp = self.get_clock().now().to_msg()
        self.roi_pub.publish(roi)

    def publish_locked_bundle(self) -> None:
        self.publish_locked_roi()
        if self.locked_grasp is not None:
            grasp = deepcopy(self.locked_grasp)
            grasp.header.stamp = self.get_clock().now().to_msg()
            self.grasp_pub.publish(grasp)
        if self.locked_opening is not None:
            self.opening_pub.publish(deepcopy(self.locked_opening))

    @staticmethod
    def has_points(msg: PointCloud2) -> bool:
        return int(msg.width) * int(msg.height) > 0 and len(msg.data) > 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickPerceptionSnapshot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
