from __future__ import annotations

from pathlib import Path
from common import RESEARCH, iso_week, today_iso

week = iso_week()
out = RESEARCH / f"05_progress/weekly/{week}.md"
if out.exists():
    print(f"Already exists: {out}")
    raise SystemExit(0)

template = (RESEARCH / "05_progress/weekly/WEEK_TEMPLATE.md").read_text(encoding="utf-8")
template = template.replace("YYYY-WXX", week)
template = template.replace("# YYYY-WXX", f"# {week}")
out.write_text(template + f"\n<!-- created: {today_iso()} -->\n", encoding="utf-8")
print(out)
