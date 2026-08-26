from __future__ import annotations

import argparse
from common import RESEARCH, dump_yaml, load_yaml, next_numeric_id

parser = argparse.ArgumentParser(description="Register a new experiment and protocol.")
parser.add_argument("--title", required=True)
parser.add_argument("--hypothesis", required=True)
parser.add_argument("--claim", action="append", default=[])
parser.add_argument("--requirement", action="append", default=[])
parser.add_argument("--owner", default="Hyunwoo Gu")
args = parser.parse_args()

registry_path = RESEARCH / "03_experiments/experiment_registry.yaml"
registry = load_yaml(registry_path) or {"schema_version": "1.0", "experiments": []}
existing = [x.get("id", "") for x in registry.get("experiments", [])]
exp_id = next_numeric_id(existing, "EXP-", 3)
slug = "-".join("".join(ch.lower() if ch.isalnum() else " " for ch in args.title).split())[:50]
protocol_rel = f"protocols/{exp_id}-{slug}.md"

registry.setdefault("experiments", []).append({
    "id": exp_id,
    "title": args.title,
    "hypothesis": args.hypothesis,
    "type": "TBD",
    "status": "planned",
    "linked_claims": args.claim,
    "linked_requirements": args.requirement,
    "protocol": protocol_rel,
    "result_manifest": None,
    "evidence": [],
    "owner": args.owner,
})
dump_yaml(registry_path, registry)

protocol = RESEARCH / "03_experiments" / protocol_rel
protocol.parent.mkdir(parents=True, exist_ok=True)
protocol.write_text(f"""# {exp_id} — {args.title}\n\n## 1. Hypothesis\n\n{args.hypothesis}\n\n## 2. Linked Claims and Requirements\n\n- Claims: {', '.join(args.claim) or 'TBD'}\n- Requirements: {', '.join(args.requirement) or 'TBD'}\n\n## 3. Independent Variables\n\nTBD\n\n## 4. Controlled Variables\n\nTBD\n\n## 5. Baselines\n\nTBD\n\n## 6. Dataset / Scene / Episode Selection\n\nTBD\n\n## 7. Seeds and Repeats\n\nTBD\n\n## 8. Metrics\n\nTBD\n\n## 9. Execution Command\n\n```bash\n# TBD\n```\n\n## 10. Artifact Paths\n\nTBD\n\n## 11. Completion Criteria\n\nTBD\n\n## 12. Invalid Run Criteria\n\nTBD\n\n## 13. Analysis Plan\n\nTBD\n\n## 14. Risks and Confounds\n\nTBD\n\n## 15. Result Summary\n\nNot executed.\n""", encoding="utf-8")
print(protocol)
