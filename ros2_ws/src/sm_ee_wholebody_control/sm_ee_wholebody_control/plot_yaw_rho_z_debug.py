import argparse
import csv
from pathlib import Path
from statistics import mean


PLOT_GROUPS = [
    ("switching", ["mu", "arm_scale", "base_scale", "hold_active"]),
    ("workspace", ["base_target_rho", "arm_target_rho", "ee_rho", "rho_w", "active_switching_rho"]),
    ("base", ["base_target_rho", "base_yaw_error", "cmd_v", "cmd_w"]),
    ("arm_error", ["arm_yaw_error", "ee_yaw_error", "joint_yaw_error", "rho_error", "z_error"]),
    ("arm_state", ["arm_target_rho", "ee_rho", "target_z", "ee_z"]),
    ("joint_velocity", ["joint1_velocity_cmd", "joint2_velocity_cmd", "joint3_velocity_cmd"]),
    ("status", ["aligned", "jacobian_det"]),
]


SUMMARY_COLUMNS = [
    "mu",
    "arm_scale",
    "base_scale",
    "rho_w",
    "active_switching_rho",
    "base_target_rho",
    "base_yaw_error",
    "cmd_v",
    "cmd_w",
    "arm_yaw_error",
    "ee_yaw_error",
    "joint_yaw_error",
    "rho_error",
    "z_error",
    "joint1_velocity_cmd",
    "joint2_velocity_cmd",
    "joint3_velocity_cmd",
    "aligned",
    "hold_active",
]


def read_csv(path: Path) -> dict[str, list[float]]:
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        if not reader.fieldnames:
            raise RuntimeError(f"No CSV header found: {path}")

        columns = {name: [] for name in reader.fieldnames}
        for row in reader:
            for name, value in row.items():
                try:
                    columns[name].append(float(value))
                except (TypeError, ValueError):
                    columns[name].append(float("nan"))

    return columns


def valid_values(columns: dict[str, list[float]], name: str) -> list[float]:
    return [value for value in columns.get(name, []) if value == value]


def summarize(columns: dict[str, list[float]]) -> list[str]:
    lines = []
    sample_count = len(next(iter(columns.values()), []))
    duration = 0.0
    if "elapsed_s" in columns and columns["elapsed_s"]:
        duration = columns["elapsed_s"][-1] - columns["elapsed_s"][0]

    lines.append(f"samples: {sample_count}")
    lines.append(f"duration_s: {duration:.3f}")

    for name in SUMMARY_COLUMNS:
        values = valid_values(columns, name)
        if not values:
            continue
        lines.append(
            f"{name}: min={min(values):.4f}, mean={mean(values):.4f}, "
            f"max={max(values):.4f}, final={values[-1]:.4f}"
        )

    return lines


def plot(columns: dict[str, list[float]], out_dir: Path, stem: str) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    x = columns.get("elapsed_s") or list(range(len(next(iter(columns.values()), []))))
    output_paths = []

    for group_name, names in PLOT_GROUPS:
        available = [name for name in names if name in columns]
        if not available:
            continue

        fig, ax = plt.subplots(figsize=(11, 5))
        for name in available:
            ax.plot(x, columns[name], label=name)
        ax.set_title(group_name)
        ax.set_xlabel("elapsed_s")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()

        output_path = out_dir / f"{stem}_{group_name}.png"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)
        output_paths.append(output_path)

    return output_paths


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot and summarize /wholebody_yaw_rho_z_debug CSV logs.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    csv_path = args.csv_path.expanduser()
    out_dir = (args.out_dir or csv_path.parent).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    columns = read_csv(csv_path)
    summary_lines = summarize(columns)
    summary_path = out_dir / f"{csv_path.stem}_summary.txt"
    summary_path.write_text("\n".join(summary_lines) + "\n")

    print("\n".join(summary_lines))
    print(f"summary: {summary_path}")

    try:
        output_paths = plot(columns, out_dir, csv_path.stem)
    except ImportError:
        print("matplotlib is not installed. Summary was still generated.")
        return

    for output_path in output_paths:
        print(f"plot: {output_path}")


if __name__ == "__main__":
    main()
