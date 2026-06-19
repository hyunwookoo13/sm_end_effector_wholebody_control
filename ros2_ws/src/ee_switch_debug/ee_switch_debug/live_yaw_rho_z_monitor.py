from collections import defaultdict, deque
from threading import Lock, Thread
from time import monotonic

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


PLOT_GROUPS = [
    ("1. Base Direction", ["base_yaw_error", "cmd_w"]),
    ("2. Base Distance / Workspace", ["base_target_rho", "arm_target_rho", "ee_rho", "rho_w", "active_switching_rho"]),
    ("3. Switching", ["mu", "arm_scale", "base_scale", "hold_active"]),
    ("4. Arm Yaw", ["arm_yaw_error", "ee_yaw_error", "joint_yaw_error", "joint1_velocity_cmd"]),
    ("5. Arm Z", ["z_error", "target_z", "ee_z"]),
    ("6. Arm Reach", ["rho_error", "arm_target_rho", "ee_rho"]),
]

STATUS_LIMITS = {
    "base_yaw_error": 0.08,
    "rho_error": 0.04,
    "z_error": 0.05,
    "arm_yaw_error": 0.03,
}


class LiveYawRhoZMonitor(Node):
    def __init__(self) -> None:
        super().__init__("live_yaw_rho_z_monitor")

        self.declare_parameter("debug_topic", "/wholebody_yaw_rho_z_debug")
        self.declare_parameter("window_sec", 30.0)
        self.declare_parameter("update_ms", 100)

        self.debug_topic = self.get_parameter("debug_topic").value
        self.window_sec = float(self.get_parameter("window_sec").value)
        self.update_ms = int(self.get_parameter("update_ms").value)

        self.lock = Lock()
        self.start_time = monotonic()
        self.times: deque[float] = deque()
        self.values: dict[str, deque[float]] = defaultdict(deque)
        self.labels: list[str] = []
        self.sample_count = 0
        self.last_message_time = 0.0

        self.create_subscription(Float64MultiArray, self.debug_topic, self.on_debug, 10)
        self.get_logger().info(f"Live monitor listening to {self.debug_topic}")

    def on_debug(self, msg: Float64MultiArray) -> None:
        labels = self.get_labels(msg)
        elapsed = monotonic() - self.start_time

        with self.lock:
            self.labels = labels
            self.times.append(elapsed)
            for label, value in zip(labels, msg.data):
                self.values[label].append(float(value))
            self.trim_old_locked()
            self.sample_count += 1
            self.last_message_time = monotonic()

    def get_labels(self, msg: Float64MultiArray) -> list[str]:
        if msg.layout.dim and msg.layout.dim[0].label:
            labels = [label.strip() for label in msg.layout.dim[0].label.split(",")]
            if len(labels) == len(msg.data):
                return labels
        return [f"value_{index}" for index in range(len(msg.data))]

    def trim_old_locked(self) -> None:
        if not self.times:
            return
        cutoff = self.times[-1] - self.window_sec
        while self.times and self.times[0] < cutoff:
            self.times.popleft()
            for series in self.values.values():
                if series:
                    series.popleft()

    def snapshot(self) -> tuple[list[float], dict[str, list[float]], int, float]:
        with self.lock:
            times = list(self.times)
            values = {name: list(series) for name, series in self.values.items()}
            age = monotonic() - self.last_message_time if self.last_message_time else -1.0
            return times, values, self.sample_count, age


def run_gui(node: LiveYawRhoZMonitor) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError as exc:
        node.get_logger().error(f"matplotlib is required for GUI monitoring: {exc}")
        return

    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True)
    axes_flat = axes.flatten()
    fig.suptitle("Whole-body control live monitor")
    lines_by_name = {}

    for ax, (title, names) in zip(axes_flat, PLOT_GROUPS):
        ax.set_title(title)
        ax.grid(True)
        for name in names:
            (line,) = ax.plot([], [], label=name)
            lines_by_name[name] = line
        ax.legend(loc="upper right")

    axes_flat[-1].set_xlabel("elapsed_s")
    status_text = fig.text(0.01, 0.01, "waiting for data", fontsize=10)

    def update(_frame):
        times, values, sample_count, age = node.snapshot()
        if not times:
            status_text.set_text("waiting for data")
            return list(lines_by_name.values()) + [status_text]

        for name, line in lines_by_name.items():
            y = values.get(name, [])
            if len(y) == len(times):
                line.set_data(times, y)

        x_min = max(0.0, times[-1] - node.window_sec)
        x_max = max(node.window_sec, times[-1])
        for ax, (_title, names) in zip(axes_flat, PLOT_GROUPS):
            ax.set_xlim(x_min, x_max)
            visible_values = []
            for name in names:
                visible_values.extend(values.get(name, []))
            if visible_values:
                y_min = min(visible_values)
                y_max = max(visible_values)
                if abs(y_max - y_min) < 1e-6:
                    margin = 0.1 if abs(y_max) < 1e-6 else abs(y_max) * 0.1
                else:
                    margin = (y_max - y_min) * 0.15
                ax.set_ylim(y_min - margin, y_max + margin)

        latest = {name: series[-1] for name, series in values.items() if series}
        status_text.set_text(make_status_text(sample_count, age, latest))
        return list(lines_by_name.values()) + [status_text]

    animation = FuncAnimation(fig, update, interval=node.update_ms, blit=False, cache_frame_data=False)
    fig._live_monitor_animation = animation
    plt.tight_layout(rect=(0, 0.03, 1, 0.96))
    plt.show()


def ok_flag(value: float, limit: float) -> str:
    if value != value:
        return "?"
    return "OK" if abs(value) <= limit else "CHECK"


def make_status_text(sample_count: int, age: float, latest: dict[str, float]) -> str:
    base_yaw = latest.get("base_yaw_error", float("nan"))
    arm_yaw = latest.get("arm_yaw_error", float("nan"))
    rho = latest.get("rho_error", float("nan"))
    z = latest.get("z_error", float("nan"))
    mu = latest.get("mu", float("nan"))
    base_scale = latest.get("base_scale", float("nan"))
    arm_scale = latest.get("arm_scale", float("nan"))
    rho_w = latest.get("rho_w", float("nan"))
    hold_active = latest.get("hold_active", float("nan"))
    cmd_v = latest.get("cmd_v", float("nan"))
    cmd_w = latest.get("cmd_w", float("nan"))

    return (
        f"samples={sample_count} age={age:.2f}s | "
        f"base_dir {ok_flag(base_yaw, STATUS_LIMITS['base_yaw_error'])}: {base_yaw:.3f}, "
        f"arm_yaw {ok_flag(arm_yaw, STATUS_LIMITS['arm_yaw_error'])}: {arm_yaw:.3f}, "
        f"reach {ok_flag(rho, STATUS_LIMITS['rho_error'])}: {rho:.3f}, "
        f"z {ok_flag(z, STATUS_LIMITS['z_error'])}: {z:.3f} | "
        f"mu={mu:.3f} arm={arm_scale:.3f} base={base_scale:.3f} rho_w={rho_w:.3f} hold={hold_active:.0f} | "
        f"cmd_v={cmd_v:.3f} cmd_w={cmd_w:.3f}"
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiveYawRhoZMonitor()
    spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
