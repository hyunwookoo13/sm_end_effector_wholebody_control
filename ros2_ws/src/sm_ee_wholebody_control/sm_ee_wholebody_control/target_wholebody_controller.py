from math import acos, atan2, hypot, pi, sin, tanh

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    return (angle + pi) % (2.0 * pi) - pi


class TargetWholebodyController(Node):
    def __init__(self) -> None:
        super().__init__("target_wholebody_controller")

        self.declare_parameter("target_frame", "target_in")
        self.declare_parameter("base_control_frame", "chassis_link")
        self.declare_parameter("arm_base_frame", "link0")
        self.declare_parameter("ee_frame", "end_effector_link")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("cmd_topic", "/cmd_vel")
        self.declare_parameter("joint_command_topic", "/target_joint_command")
        self.declare_parameter("debug_topic", "/wholebody_debug")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("switching_point", 0.5)
        self.declare_parameter("alpha", 5.0)
        self.declare_parameter("base_k_linear", 0.6)
        self.declare_parameter("base_k_angular", 1.5)
        self.declare_parameter("max_linear", 0.25)
        self.declare_parameter("max_angular", 0.6)
        self.declare_parameter("base_stop_distance", 0.05)
        self.declare_parameter("shoulder_z", 0.1715)
        self.declare_parameter("link1", 0.247)
        self.declare_parameter("link2", 0.45)
        self.declare_parameter("min_reach", 0.05)
        self.declare_parameter("joint_kp", 0.35)
        self.declare_parameter("max_joint_step", 0.04)
        self.declare_parameter("ee_stop_distance", 0.03)
        self.declare_parameter("arm_k_rho", 4.0)
        self.declare_parameter("arm_k_z", 3.0)
        self.declare_parameter("arm_k_theta", 4.5)
        self.declare_parameter("arm_lpf_alpha", 0.3)
        self.declare_parameter("arm_yaw_lpf_alpha", 0.1)
        self.declare_parameter("max_rho_velocity", 0.35)
        self.declare_parameter("max_z_velocity", 0.25)
        self.declare_parameter("max_arm_yaw_velocity", 0.8)
        self.declare_parameter("max_joint_velocity", 1.2)
        self.declare_parameter("joint1_sign", 1.0)
        self.declare_parameter("joint2_sign", 1.0)
        self.declare_parameter("joint3_sign", 1.0)

        self.target_frame = self.get_parameter("target_frame").value
        self.base_control_frame = self.get_parameter("base_control_frame").value
        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        joint_state_topic = self.get_parameter("joint_state_topic").value
        cmd_topic = self.get_parameter("cmd_topic").value
        joint_command_topic = self.get_parameter("joint_command_topic").value
        debug_topic = self.get_parameter("debug_topic").value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.switching_point = float(self.get_parameter("switching_point").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.base_k_linear = float(self.get_parameter("base_k_linear").value)
        self.base_k_angular = float(self.get_parameter("base_k_angular").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.base_stop_distance = float(self.get_parameter("base_stop_distance").value)
        self.shoulder_z = float(self.get_parameter("shoulder_z").value)
        self.link1 = float(self.get_parameter("link1").value)
        self.link2 = float(self.get_parameter("link2").value)
        self.min_reach = float(self.get_parameter("min_reach").value)
        self.joint_kp = float(self.get_parameter("joint_kp").value)
        self.max_joint_step = float(self.get_parameter("max_joint_step").value)
        self.ee_stop_distance = float(self.get_parameter("ee_stop_distance").value)
        self.arm_k_rho = float(self.get_parameter("arm_k_rho").value)
        self.arm_k_z = float(self.get_parameter("arm_k_z").value)
        self.arm_k_theta = float(self.get_parameter("arm_k_theta").value)
        self.arm_lpf_alpha = float(self.get_parameter("arm_lpf_alpha").value)
        self.arm_yaw_lpf_alpha = float(self.get_parameter("arm_yaw_lpf_alpha").value)
        self.max_rho_velocity = float(self.get_parameter("max_rho_velocity").value)
        self.max_z_velocity = float(self.get_parameter("max_z_velocity").value)
        self.max_arm_yaw_velocity = float(self.get_parameter("max_arm_yaw_velocity").value)
        self.max_joint_velocity = float(self.get_parameter("max_joint_velocity").value)
        self.joint1_sign = float(self.get_parameter("joint1_sign").value)
        self.joint2_sign = float(self.get_parameter("joint2_sign").value)
        self.joint3_sign = float(self.get_parameter("joint3_sign").value)

        self.current_joints: dict[str, float] = {}
        self.last_update_ns = None
        self.arm_state_initialized = False
        self.rho_m = 0.0
        self.z_m = 0.0
        self.thm = 0.0
        self.dist_trajectory_old = None
        self.dist_trajectory_v = 0.0
        self.arm_yaw_target_old = None
        self.wm_lpf = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, joint_state_topic, self.on_joint_state, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.joint_pub = self.create_publisher(JointState, joint_command_topic, 10)
        self.debug_pub = self.create_publisher(Float64MultiArray, debug_topic, 10)
        self.timer = self.create_timer(max(1.0 / max(rate_hz, 0.1), 0.01), self.on_timer)
        self.log_timer = self.create_timer(1.0, self.on_log_timer)
        self.last_debug = "waiting for transform"
        self.debug_labels = [
            "rho_t",
            "mu",
            "cmd_v",
            "cmd_w",
            "ee_err",
            "base_target_x",
            "base_target_y",
            "arm_target_z",
            "rho_m",
            "z_m",
            "thm",
            "v_rho",
            "v_z",
            "wm",
            "max_dq",
            "joint1_cmd",
            "joint2_cmd",
            "joint3_cmd",
        ]

        self.get_logger().info(
            f"Wholebody controller target={self.target_frame}, "
            f"base_frame={self.base_control_frame}, arm_frame={self.arm_base_frame}"
        )

    def on_joint_state(self, msg: JointState) -> None:
        self.current_joints.update(dict(zip(msg.name, msg.position)))

    def on_timer(self) -> None:
        twist = Twist()
        missing_joints = [name for name in self.joint_names if name not in self.current_joints]
        if missing_joints:
            self.last_debug = f"waiting for joint states: {', '.join(missing_joints)}"
            self.cmd_pub.publish(twist)
            return

        try:
            target_from_chassis = self.tf_buffer.lookup_transform(
                self.base_control_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            target_from_arm = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.target_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            ee_from_arm = self.tf_buffer.lookup_transform(
                self.arm_base_frame,
                self.ee_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.last_debug = f"lookup failed: {exc}"
            self.cmd_pub.publish(twist)
            return

        base_t = target_from_chassis.transform.translation
        arm_t = target_from_arm.transform.translation
        ee = ee_from_arm.transform.translation
        dt = self.get_dt()

        rho_t = hypot(base_t.x, base_t.y)
        mu = 0.5 * (1.0 - tanh(self.alpha * (rho_t - self.switching_point)))
        base_weight = 1.0 - mu

        target_yaw = atan2(base_t.y, base_t.x)
        if rho_t > self.base_stop_distance:
            twist.linear.x = base_weight * self.base_k_linear * base_t.x
            twist.angular.z = base_weight * self.base_k_angular * target_yaw

        twist.linear.x = clamp(twist.linear.x, -self.max_linear, self.max_linear)
        twist.angular.z = clamp(twist.angular.z, -self.max_angular, self.max_angular)
        self.cmd_pub.publish(twist)

        ee_error = (
            (arm_t.x - ee.x) ** 2 + (arm_t.y - ee.y) ** 2 + (arm_t.z - ee.z) ** 2
        ) ** 0.5
        joint_command, arm_debug = self.make_joint_command(arm_t, ee, mu, dt)
        self.joint_pub.publish(joint_command)
        self.publish_debug(
            rho_t=rho_t,
            mu=mu,
            twist=twist,
            ee_error=ee_error,
            base_t=base_t,
            arm_t=arm_t,
            joint_command=joint_command,
            arm_debug=arm_debug,
        )

        self.last_debug = (
            f"rho_t={rho_t:.3f}, mu={mu:.3f}, cmd_v={twist.linear.x:.3f}, "
            f"cmd_w={twist.angular.z:.3f}, ee_err={ee_error:.3f}, "
            f"rho_m={self.rho_m:.3f}, z_m={self.z_m:.3f}, "
            f"v_rho={arm_debug['v_rho']:.3f}, v_z={arm_debug['v_z']:.3f}, "
            f"wm={arm_debug['wm']:.3f}, "
            f"q123=[{joint_command.position[0]:.2f}, {joint_command.position[1]:.2f}, "
            f"{joint_command.position[2]:.2f}]"
        )

    def get_dt(self) -> float:
        now_ns = self.get_clock().now().nanoseconds
        if self.last_update_ns is None:
            self.last_update_ns = now_ns
            return max(1.0 / max(float(self.get_parameter("rate_hz").value), 0.1), 0.01)

        dt = (now_ns - self.last_update_ns) * 1e-9
        self.last_update_ns = now_ns
        return clamp(dt, 0.001, 0.1)

    def make_joint_command(self, target, ee, mu: float, dt: float) -> tuple[JointState, dict[str, float]]:
        self.initialize_arm_state(ee)

        rho_target = clamp(hypot(target.x, target.y), self.min_reach, self.link1 + self.link2 - 1e-3)
        z_target = target.z - self.shoulder_z
        yaw_target = atan2(target.y, target.x)

        if self.dist_trajectory_old is None:
            self.dist_trajectory_old = rho_target
        dist_trajectory_dev = (rho_target - self.dist_trajectory_old) / dt
        self.dist_trajectory_old = rho_target
        self.dist_trajectory_v += self.arm_lpf_alpha * (
            dist_trajectory_dev - self.dist_trajectory_v
        )

        erho = rho_target - self.rho_m
        v_rho = mu * (self.dist_trajectory_v + self.arm_k_rho * erho)
        z_error = z_target - self.z_m
        v_z = self.arm_k_z * z_error

        if self.arm_yaw_target_old is None:
            self.arm_yaw_target_old = yaw_target
        yaw_target_diff = normalize_angle(yaw_target - self.arm_yaw_target_old)
        yaw_target_dev = yaw_target_diff / dt
        self.arm_yaw_target_old = yaw_target

        yaw_error = normalize_angle(yaw_target - self.thm)
        wmc = mu * (yaw_target_dev + self.arm_k_theta * yaw_error)
        self.wm_lpf += self.arm_yaw_lpf_alpha * (wmc - self.wm_lpf)
        wm = self.wm_lpf

        v_rho = clamp(v_rho, -self.max_rho_velocity, self.max_rho_velocity)
        v_z = clamp(v_z, -self.max_z_velocity, self.max_z_velocity)
        wm = clamp(wm, -self.max_arm_yaw_velocity, self.max_arm_yaw_velocity)
        v_rho, v_z, wm, max_dq = self.limit_joint_velocity(v_rho, v_z, wm, dt)

        self.rho_m, self.z_m = self.clamp_arm_state(
            self.rho_m + v_rho * dt,
            self.z_m + v_z * dt,
        )
        self.thm = normalize_angle(self.thm + wm * dt)

        q2, q3 = self.solve_planar_2link(self.rho_m, self.z_m)
        command = [
            self.joint1_sign * self.thm,
            self.joint2_sign * q2,
            self.joint3_sign * q3,
            self.current_joints["joint4"],
            self.current_joints["joint5"],
            self.current_joints["joint6"],
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = command
        return msg, {
            "rho_target": rho_target,
            "z_target": z_target,
            "yaw_target": yaw_target,
            "v_rho": v_rho,
            "v_z": v_z,
            "wm": wm,
            "max_dq": max_dq,
        }

    def initialize_arm_state(self, ee) -> None:
        if self.arm_state_initialized:
            return

        self.rho_m, self.z_m = self.clamp_arm_state(
            hypot(ee.x, ee.y),
            ee.z - self.shoulder_z,
        )
        if abs(self.joint1_sign) > 1e-9:
            self.thm = normalize_angle(self.current_joints["joint1"] / self.joint1_sign)
        else:
            self.thm = atan2(ee.y, ee.x)
        self.arm_state_initialized = True

    def clamp_arm_state(self, rho: float, z: float) -> tuple[float, float]:
        max_reach = self.link1 + self.link2 - 1e-3
        rho = max(rho, self.min_reach)
        reach = hypot(rho, z)
        if reach > max_reach:
            scale = max_reach / reach
            rho *= scale
            z *= scale
        return rho, z

    def limit_joint_velocity(
        self,
        v_rho: float,
        v_z: float,
        wm: float,
        dt: float,
    ) -> tuple[float, float, float, float]:
        max_dq = 0.0
        for _ in range(3):
            rho_try, z_try = self.clamp_arm_state(
                self.rho_m + v_rho * dt,
                self.z_m + v_z * dt,
            )
            thm_try = normalize_angle(self.thm + wm * dt)
            q2_try, q3_try = self.solve_planar_2link(rho_try, z_try)
            q_try = [
                self.joint1_sign * thm_try,
                self.joint2_sign * q2_try,
                self.joint3_sign * q3_try,
            ]
            q_now = [self.current_joints[name] for name in self.joint_names[:3]]
            dq = [
                abs(normalize_angle(q_try[0] - q_now[0]) / dt),
                abs((q_try[1] - q_now[1]) / dt),
                abs((q_try[2] - q_now[2]) / dt),
            ]
            max_dq = max(dq)
            if max_dq <= self.max_joint_velocity:
                break

            scale = self.max_joint_velocity / max(max_dq, 1e-9) * 0.85
            v_rho *= scale
            v_z *= scale
            wm *= scale

        return v_rho, v_z, wm, max_dq

    def publish_debug(
        self,
        *,
        rho_t,
        mu,
        twist,
        ee_error,
        base_t,
        arm_t,
        joint_command,
        arm_debug,
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
            rho_t,
            mu,
            twist.linear.x,
            twist.angular.z,
            ee_error,
            base_t.x,
            base_t.y,
            arm_t.z,
            self.rho_m,
            self.z_m,
            self.thm,
            arm_debug["v_rho"],
            arm_debug["v_z"],
            arm_debug["wm"],
            arm_debug["max_dq"],
            joint_command.position[0],
            joint_command.position[1],
            joint_command.position[2],
        ]
        self.debug_pub.publish(msg)

    def solve_planar_2link(self, rho: float, z: float) -> tuple[float, float]:
        r2 = rho**2 + z**2
        c2 = (r2 - self.link1**2 - self.link2**2) / (2.0 * self.link1 * self.link2)
        q3 = acos(clamp(c2, -1.0, 1.0))
        beta = atan2(self.link2 * sin(q3), self.link1 + self.link2 * c2)
        alpha = atan2(z, rho)
        q2 = alpha - beta
        return q2, q3

    def on_log_timer(self) -> None:
        self.get_logger().info(self.last_debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetWholebodyController()
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
