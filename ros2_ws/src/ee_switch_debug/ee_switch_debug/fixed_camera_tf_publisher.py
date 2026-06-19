import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FixedCameraTfPublisher(Node):
    """Publish a corrected world -> fixed camera TF for Isaac-mounted cameras."""

    def __init__(self) -> None:
        super().__init__("fixed_camera_tf_publisher")

        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("child_frame", "rsd455_color_optical_frame")
        self.declare_parameter("isaac_x", -0.04447)
        self.declare_parameter("isaac_y", 3.46295)
        self.declare_parameter("isaac_z", 1.25048)
        self.declare_parameter("mapping", "isaac_y_to_ros_x")
        self.declare_parameter("qx", 0.0)
        self.declare_parameter("qy", 0.0)
        self.declare_parameter("qz", 0.0)
        self.declare_parameter("qw", 1.0)
        self.declare_parameter("rate_hz", 30.0)

        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.isaac_x = float(self.get_parameter("isaac_x").value)
        self.isaac_y = float(self.get_parameter("isaac_y").value)
        self.isaac_z = float(self.get_parameter("isaac_z").value)
        self.mapping = str(self.get_parameter("mapping").value)
        self.qx = float(self.get_parameter("qx").value)
        self.qy = float(self.get_parameter("qy").value)
        self.qz = float(self.get_parameter("qz").value)
        self.qw = float(self.get_parameter("qw").value)
        rate_hz = float(self.get_parameter("rate_hz").value)

        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)

        ros_x, ros_y, ros_z = self.map_translation()
        self.get_logger().info(
            f"Publishing corrected camera TF {self.parent_frame} -> {self.child_frame}: "
            f"isaac=({self.isaac_x:.5f}, {self.isaac_y:.5f}, {self.isaac_z:.5f}), "
            f"ros=({ros_x:.5f}, {ros_y:.5f}, {ros_z:.5f}), "
            f"q=({self.qx:.4f}, {self.qy:.4f}, {self.qz:.4f}, {self.qw:.4f})"
        )

    def map_translation(self) -> tuple[float, float, float]:
        if self.mapping == "identity":
            return self.isaac_x, self.isaac_y, self.isaac_z
        if self.mapping == "isaac_y_to_ros_x":
            return self.isaac_y, -self.isaac_x, self.isaac_z
        if self.mapping == "isaac_y_to_ros_x_flip_y":
            return self.isaac_y, self.isaac_x, self.isaac_z
        self.get_logger().warn(f"Unknown mapping={self.mapping}; using identity.")
        return self.isaac_x, self.isaac_y, self.isaac_z

    def on_timer(self) -> None:
        ros_x, ros_y, ros_z = self.map_translation()

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = ros_x
        msg.transform.translation.y = ros_y
        msg.transform.translation.z = ros_z
        msg.transform.rotation.x = self.qx
        msg.transform.rotation.y = self.qy
        msg.transform.rotation.z = self.qz
        msg.transform.rotation.w = self.qw
        self.broadcaster.sendTransform(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FixedCameraTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
