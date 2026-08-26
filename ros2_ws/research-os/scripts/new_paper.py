from __future__ import annotations

import argparse
from common import RESEARCH, dump_yaml, load_yaml, next_numeric_id

parser = argparse.ArgumentParser(description="Register a paper in the research literature map.")
parser.add_argument("--title", required=True)
parser.add_argument("--family", required=True)
parser.add_argument("--priority", choices=["high", "medium", "low"], default="medium")
parser.add_argument("--url", default="TBD")
args = parser.parse_args()

registry_path = RESEARCH / "10_literature/paper_registry.yaml"
registry = load_yaml(registry_path) or {"schema_version": "1.0", "papers": []}
existing = [x.get("id", "") for x in registry.get("papers", [])]
paper_id = next_numeric_id(existing, "PAPER-", 3)
slug = "-".join("".join(ch.lower() if ch.isalnum() else " " for ch in args.title).split())[:60]
review_rel = f"reviews/{paper_id}-{slug}.md"

registry.setdefault("papers", []).append({
    "id": paper_id,
    "title": args.title,
    "primary_family": args.family,
    "priority": args.priority,
    "status": "queued",
    "url": args.url,
    "review": review_rel,
    "requirements_affected": [],
    "closest_overlap": "TBD",
})
dump_yaml(registry_path, registry)

out = RESEARCH / "10_literature" / review_rel
out.parent.mkdir(parents=True, exist_ok=True)
template = (RESEARCH / "10_literature/PAPER_REVIEW_TEMPLATE.md").read_text(encoding="utf-8")
template = template.replace("PAPER-XXX", paper_id).replace("Paper Title", args.title)
out.write_text(template, encoding="utf-8")
print(out)
