from __future__ import annotations

from datetime import date, datetime
from pathlib import Path
from typing import Any
import subprocess

try:
    import yaml
except ImportError as exc:
    raise SystemExit("PyYAML is required. Run: pip install -r requirements.txt") from exc

ROOT = Path(__file__).resolve().parents[1]
RESEARCH = ROOT / "research"


def load_yaml(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def dump_yaml(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False, width=100)


def today_iso() -> str:
    return date.today().isoformat()


def iso_week() -> str:
    y, w, _ = date.today().isocalendar()
    return f"{y}-W{w:02d}"


def git_commit() -> str:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, stderr=subprocess.DEVNULL, text=True
        ).strip()
    except Exception:
        return "NOT_A_GIT_REPOSITORY"


def git_dirty() -> bool | None:
    try:
        output = subprocess.check_output(
            ["git", "status", "--porcelain"], cwd=ROOT, stderr=subprocess.DEVNULL, text=True
        )
        return bool(output.strip())
    except Exception:
        return None


def next_numeric_id(existing: list[str], prefix: str, width: int = 3) -> str:
    nums = []
    for item in existing:
        if item.startswith(prefix):
            try:
                nums.append(int(item[len(prefix):]))
            except ValueError:
                pass
    return f"{prefix}{(max(nums, default=0) + 1):0{width}d}"


def next_task_id(existing: list[str]) -> str:
    stem = date.today().strftime("TASK-%Y%m%d-")
    nums = []
    for item in existing:
        if item.startswith(stem):
            try:
                nums.append(int(item.rsplit("-", 1)[1]))
            except ValueError:
                pass
    return f"{stem}{max(nums, default=0) + 1:02d}"


def utc_timestamp() -> str:
    return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"
