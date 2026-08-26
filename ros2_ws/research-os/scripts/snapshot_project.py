from __future__ import annotations

from pathlib import Path
import hashlib
import json
from common import ROOT, today_iso, utc_timestamp, git_commit, git_dirty

paths = [
    ROOT / "PROJECT.yaml",
    ROOT / "AGENTS.md",
    ROOT / "research/00_constitution/research_questions.yaml",
    ROOT / "research/00_constitution/core_claims.yaml",
    ROOT / "research/01_requirements/requirements.yaml",
    ROOT / "research/02_architecture/interfaces.yaml",
    ROOT / "research/03_experiments/experiment_registry.yaml",
    ROOT / "research/03_experiments/evidence_registry.yaml",
    ROOT / "research/05_progress/current_status.yaml",
    ROOT / "research/06_paper/evidence_map.yaml",
    ROOT / "research/07_presentation/slide_manifest.yaml",
]

entries = []
for path in paths:
    data = path.read_bytes()
    entries.append({
        "path": str(path.relative_to(ROOT)),
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
    })

manifest = {
    "schema_version": "1.0",
    "generated_at": utc_timestamp(),
    "git_commit": git_commit(),
    "git_dirty": git_dirty(),
    "files": entries,
}
out_dir = ROOT / "exports/reports"
out_dir.mkdir(parents=True, exist_ok=True)
out = out_dir / f"research_snapshot_{today_iso()}.json"
out.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(out)
