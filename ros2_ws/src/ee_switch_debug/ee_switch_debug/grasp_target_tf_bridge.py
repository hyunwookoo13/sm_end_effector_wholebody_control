import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class GraspTargetTfBridge(Node):
    """Publish the best grasp pose in the whole-body control frame."""

    def __init__(self) -> None:
        super().__init__("grasp_target_tf_bridge")

        self.declare_parameter("input_topic", "/sm_grasping/grasp_best")
        self.declare_parameter("parent_frame", "chassis_link")
        self.declare_parameter("target_frame", "grasp_position_target")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("timeout_sec", 0.5)
        self.declare_parameter("latch_target", False)
        self.declare_parameter("freeze_on_arm_track", False)
        self.declare_parameter("state_topic", "/arm_safety_state")
        self.declare_parameter("freeze_on_pick_command", False)
        self.declare_parameter("task_command_topic", "/arm_task_command")
        self.declare_parameter("task_state_topic", "/arm_task_state")
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.08)

        self.input_topic = self.get_parameter("input_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.latch_target = bool(self.get_parameter("latch_target").value)
        self.freeze_on_arm_track = bool(
            self.get_parameter("freeze_on_arm_track").value
        )
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.freeze_on_pick_command = bool(
            self.get_parameter("freeze_on_pick_command").value
        )
        self.task_command_topic = str(
            self.get_parameter("task_command_topic").value
        )
        self.task_state_topic = str(self.get_parameter("task_state_topic").value)
        self.offset_x = float(self.get_parameter("offset_x").value)
        self.offset_y = float(self.get_parameter("offset_y").value)
        self.offset_z = float(self.get_parameter("offset_z").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.latest_transform: TransformStamped | None = None
        self.latest_update_time = self.get_clock().now()
        self.target_latched = False
        self.target_frozen = False
        self.control_state = ""
        self.pick_freeze_requested = False
        self.pick_snapshot_active = False
        self.last_debug = "waiting for grasp_best"

        self.create_subscription(PoseStamped, self.input_topic, self.on_grasp_best, 10)
        if self.freeze_on_arm_track:
            self.create_subscription(String, self.state_topic, self.on_control_state, 10)
        if self.freeze_on_pick_command:
            self.create_subscription(
                String,
                self.task_command_topic,
                self.on_task_command,
                10,
            )
            self.create_subscription(
                String,
                self.task_state_topic,
                self.on_task_state,
                10,
            )
        self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.create_timer(1.0, self.on_log_timer)

        self.get_logger().info(
            f"Publishing grasp position TF: {self.parent_frame} -> {self.target_frame}, "
            f"source={self.input_topic}, offset=[{self.offset_x:.3f}, {self.offset_y:.3f}, {self.offset_z:.3f}], "
            f"latch_target={self.latch_target}, "
            f"freeze_on_arm_track={self.freeze_on_arm_track}, "
            f"freeze_on_pick_command={self.freeze_on_pick_command}"
        )

    def on_grasp_best(self, msg: PoseStamped) -> None:
        if self.target_latched or self.target_frozen:
            if self.pick_snapshot_active:
                self.get_logger().debug(
                    f"Ignored later grasp update; {self.target_frame} is locked "
                    "to the first valid post-PICK sample"
                )
            return

        source_frame = msg.header.frame_id
        if not source_frame:
            self.last_debug = "grasp_best has an empty frame_id"
            return

        source_point = (
            float(msg.pose.position.x) + self.offset_x,
            float(msg.pose.position.y) + self.offset_y,
            float(msg.pose.position.z) + self.offset_z,
        )

        if source_frame == self.parent_frame:
            target_point = source_point
            target_rotation = self.normalize_quaternion(msg.pose.orientation)
        else:
            try:
                parent_from_source = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    source_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
            except TransformException as exc:
                self.last_debug = (
                    f"waiting for transform {self.parent_frame} <- {source_frame}: {exc}"
                )
                return

            rotation = parent_from_source.transform.rotation
            rotated_point = self.rotate_vector(source_point, rotation)
            translation = parent_from_source.transform.translation
            target_point = (
                rotated_point[0] + float(translation.x),
                rotated_point[1] + float(translation.y),
                rotated_point[2] + float(translation.z),
            )
            target_rotation = self.normalize_quaternion(
                self.multiply_quaternions(
                    parent_from_source.transform.rotation,
                    msg.pose.orientation,
                )
            )

        transform = TransformStamped()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = target_point[0]
        transform.transform.translation.y = target_point[1]
        transform.transform.translation.z = target_point[2]
        transform.transform.rotation.x = target_rotation[0]
        transform.transform.rotation.y = target_rotation[1]
        transform.transform.rotation.z = target_rotation[2]
        transform.transform.rotation.w = target_rotation[3]

        self.latest_transform = transform
        self.latest_update_time = self.get_clock().now()
        self.target_latched = self.latch_target
        self.last_debug = (
            f"x={transform.transform.translation.x:.3f}, "
            f"y={transform.transform.translation.y:.3f}, "
            f"z={transform.transform.translation.z:.3f} in {self.parent_frame}"
        )
        if self.pick_freeze_requested:
            self.target_frozen = True
            self.pick_snapshot_active = True
            self.pick_freeze_requested = False
            rotation = self.latest_transform.transform.rotation
            self.get_logger().info(
                f"Locked first post-PICK {self.target_frame}: {self.last_debug}, "
                f"q=[{rotation.x:.4f}, {rotation.y:.4f}, "
                f"{rotation.z:.4f}, {rotation.w:.4f}]"
            )
        if self.target_latched:
            self.get_logger().info(f"Latched {self.target_frame}: {self.last_debug}")

    def begin_pick_snapshot(self) -> None:
        """Invalidate prior task data and wait for one fresh grasp sample."""
        self.latest_transform = None
        self.target_latched = False
        self.target_frozen = False
        self.pick_snapshot_active = False
        self.pick_freeze_requested = True
        self.latest_update_time = self.get_clock().now()
        self.last_debug = "waiting for first valid grasp after PICK"
        self.get_logger().warn(
            f"Invalidated previous {self.target_frame}; "
            "waiting for first valid grasp after PICK"
        )

    def on_task_command(self, msg: String) -> None:
        command = msg.data.strip().upper()
        if command == "PICK":
            self.begin_pick_snapshot()
        elif command == "RESET":
            self.release_pick_snapshot("RESET command")

    def on_task_state(self, msg: String) -> None:
        state = msg.data.strip().upper()
        if state in ("PICK:HOLD", "PICK:DONE"):
            self.release_pick_snapshot(state)

    def release_pick_snapshot(self, reason: str) -> None:
        if not (self.pick_snapshot_active or self.pick_freeze_requested):
            return
        self.pick_snapshot_active = False
        self.pick_freeze_requested = False
        self.target_frozen = False
        self.latest_update_time = self.get_clock().now()
        self.get_logger().info(
            f"Released PICK snapshot for {self.target_frame}: {reason}"
        )

    def on_control_state(self, msg: String) -> None:
        state = msg.data.strip()
        if state == self.control_state:
            return
        self.control_state = state

        if state == "ARM_TRACK":
            if self.pick_freeze_requested:
                self.last_debug = "ARM_TRACK waiting for first valid grasp after PICK"
                self.get_logger().warn(self.last_debug)
                return
            if self.latest_transform is None:
                self.last_debug = "ARM_TRACK requested freeze, but no grasp target exists"
                self.get_logger().warn(self.last_debug)
                return
            self.target_frozen = True
            self.get_logger().info(
                f"Frozen {self.target_frame} for ARM_TRACK: {self.last_debug}"
            )
        elif (
            state == "RETURN_HOME"
            and self.target_frozen
            and not self.pick_snapshot_active
        ):
            self.target_frozen = False
            self.latest_update_time = self.get_clock().now()
            self.get_logger().info(
                f"Released {self.target_frame}; live grasp updates resumed"
            )

    def on_timer(self) -> None:
        if self.latest_transform is None:
            return

        age = (self.get_clock().now() - self.latest_update_time).nanoseconds * 1e-9
        if not (self.target_latched or self.target_frozen) and age > self.timeout_sec:
            return

        self.latest_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.latest_transform)

    @staticmethod
    def rotate_vector(vector, quaternion):
        x, y, z = vector
        qx = float(quaternion.x)
        qy = float(quaternion.y)
        qz = float(quaternion.z)
        qw = float(quaternion.w)

        tx = 2.0 * (qy * z - qz * y)
        ty = 2.0 * (qz * x - qx * z)
        tz = 2.0 * (qx * y - qy * x)
        return (
            x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx,
        )

    @staticmethod
    def multiply_quaternions(left, right):
        lx = float(left.x)
        ly = float(left.y)
        lz = float(left.z)
        lw = float(left.w)
        rx = float(right.x)
        ry = float(right.y)
        rz = float(right.z)
        rw = float(right.w)
        return (
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        )

    @staticmethod
    def normalize_quaternion(quaternion):
        if isinstance(quaternion, tuple):
            x, y, z, w = quaternion
        else:
            x = float(quaternion.x)
            y = float(quaternion.y)
            z = float(quaternion.z)
            w = float(quaternion.w)

        norm = (x * x + y * y + z * z + w * w) ** 0.5
        if norm < 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (x / norm, y / norm, z / norm, w / norm)

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspTargetTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
