import argparse
import csv
from pathlib import Path
from statistics import mean


PLOT_GROUPS = [
    ("switching", ["rho_t", "mu"]),
    ("base_cmd", ["cmd_v", "cmd_w"]),
    ("arm_error", ["ee_err", "rho_err", "z_err", "yaw_err"]),
    ("arm_state", ["rho_m", "z_m", "thm"]),
    ("arm_velocity", ["v_rho", "v_z", "wm", "max_dq"]),
    ("joint_cmd", ["joint1_cmd", "joint2_cmd", "joint3_cmd"]),
]


def read_csv(path: Path) -> tuple[list[str], dict[str, list[float]]]:
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

    return list(columns.keys()), columns


def summarize(columns: dict[str, list[float]]) -> list[str]:
    lines = []
    sample_count = len(next(iter(columns.values()), []))
    duration = 0.0
    if "elapsed_s" in columns and columns["elapsed_s"]:
        duration = columns["elapsed_s"][-1] - columns["elapsed_s"][0]

    lines.append(f"samples: {sample_count}")
    lines.append(f"duration_s: {duration:.3f}")

    for name in [
        "rho_t",
        "mu",
        "cmd_v",
        "cmd_w",
        "ee_err",
        "rho_err",
        "z_err",
        "v_rho",
        "v_z",
        "wm",
        "max_dq",
    ]:
        values = [value for value in columns.get(name, []) if value == value]
        if not values:
            continue
        lines.append(
            f"{name}: min={min(values):.4f}, mean={mean(values):.4f}, max={max(values):.4f}, final={values[-1]:.4f}"
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

        fig, ax = plt.subplots(figsize=(10, 5))
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
    parser = argparse.ArgumentParser(description="Plot and summarize /wholebody_debug CSV logs.")
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    csv_path = args.csv_path.expanduser()
    out_dir = (args.out_dir or csv_path.parent).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    _, columns = read_csv(csv_path)
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
