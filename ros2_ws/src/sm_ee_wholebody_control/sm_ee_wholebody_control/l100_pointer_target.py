import math
from typing import Dict, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class L100PointerTarget(Node):
    def __init__(self) -> None:
        super().__init__("l100_pointer_target")

        self.declare_parameter("input_topic", "/joint_command")
        self.declare_parameter("parent_frame", "world")
        self.declare_parameter("target_frame", "target_l100_cmd")
        self.declare_parameter("yaw_joint", "joint5")
        self.declare_parameter("pitch_joint", "joint6")
        self.declare_parameter("trigger_joint", "rh_r1_joint")
        self.declare_parameter("trigger_threshold", 0.2)
        self.declare_parameter("speed", 0.2)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_z", 1.0)
        self.declare_parameter("yaw_scale", 1.0)
        self.declare_parameter("pitch_scale", 1.0)

        self.input_topic = self.get_parameter("input_topic").value
        self.parent_frame = self.get_parameter("parent_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.yaw_joint = self.get_parameter("yaw_joint").value
        self.pitch_joint = self.get_parameter("pitch_joint").value
        self.trigger_joint = self.get_parameter("trigger_joint").value
        self.trigger_threshold = float(self.get_parameter("trigger_threshold").value)
        self.speed = float(self.get_parameter("speed").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.pitch_scale = float(self.get_parameter("pitch_scale").value)

        self.position = [
            float(self.get_parameter("initial_x").value),
            float(self.get_parameter("initial_y").value),
            float(self.get_parameter("initial_z").value),
        ]
        self.zero_yaw: Optional[float] = None
        self.zero_pitch: Optional[float] = None
        self.latest_joints: Dict[str, float] = {}
        self.last_time = self.get_clock().now()

        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(JointState, self.input_topic, self.on_joint_state, 10)
        self.create_timer(0.02, self.on_timer)

        self.get_logger().info(
            f"Publishing {self.parent_frame} -> {self.target_frame}; "
            f"yaw={self.yaw_joint}, pitch={self.pitch_joint}, trigger={self.trigger_joint}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joints = dict(zip(msg.name, msg.position))

        if self.zero_yaw is None and self.yaw_joint in self.latest_joints:
            self.zero_yaw = self.latest_joints[self.yaw_joint]
        if self.zero_pitch is None and self.pitch_joint in self.latest_joints:
            self.zero_pitch = self.latest_joints[self.pitch_joint]

    def on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if self.has_required_joints():
            trigger = self.latest_joints[self.trigger_joint] > self.trigger_threshold
            if trigger:
                yaw = (self.latest_joints[self.yaw_joint] - self.zero_yaw) * self.yaw_scale
                pitch = (self.latest_joints[self.pitch_joint] - self.zero_pitch) * self.pitch_scale

                direction = [
                    math.cos(pitch) * math.cos(yaw),
                    math.cos(pitch) * math.sin(yaw),
                    math.sin(pitch),
                ]
                for index in range(3):
                    self.position[index] += direction[index] * self.speed * dt

        self.publish_target_transform(now)

    def has_required_joints(self) -> bool:
        return (
            self.zero_yaw is not None
            and self.zero_pitch is not None
            and self.yaw_joint in self.latest_joints
            and self.pitch_joint in self.latest_joints
            and self.trigger_joint in self.latest_joints
        )

    def publish_target_transform(self, stamp) -> None:
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.target_frame
        transform.transform.translation.x = self.position[0]
        transform.transform.translation.y = self.position[1]
        transform.transform.translation.z = self.position[2]
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = L100PointerTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
