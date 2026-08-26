from __future__ import annotations

import argparse
from pathlib import Path
from common import RESEARCH, dump_yaml, load_yaml, next_task_id, today_iso

parser = argparse.ArgumentParser(description="Create a traceable Research OS task.")
parser.add_argument("--title", required=True)
parser.add_argument("--requirement", action="append", default=[])
parser.add_argument("--claim", action="append", default=[])
parser.add_argument("--architecture", action="append", default=[])
parser.add_argument("--owner", default="Hyunwoo Gu")
args = parser.parse_args()

registry_path = RESEARCH / "08_tasks/task_registry.yaml"
registry = load_yaml(registry_path) or {"schema_version": "1.0", "tasks": []}
existing = [x.get("task_id", "") for x in registry.get("tasks", [])]
task_id = next_task_id(existing)

payload = {
    "schema_version": "1.0",
    "task_id": task_id,
    "title": args.title,
    "status": "planned",
    "owner": args.owner,
    "created_at": today_iso(),
    "updated_at": today_iso(),
    "objective": "TBD — state the observable outcome, not an implementation activity.",
    "related_requirements": args.requirement,
    "related_claims": args.claim,
    "related_architecture": args.architecture,
    "related_experiments": [],
    "current_design": "TBD",
    "planned_change": "TBD",
    "reason": "TBD",
    "assumptions": [],
    "out_of_scope": [],
    "done_conditions": ["TBD"],
    "tests": [{"name": "TBD", "command": "TBD", "expected": "TBD"}],
    "paper_impact": {"sections": [], "expected_update": "TBD"},
    "presentation_impact": {"slides": [], "expected_update": "TBD"},
    "completion": {
        "changed_files": [], "test_results": [], "unresolved": [],
        "evidence_created": [], "next_task": None,
    },
}

out = RESEARCH / f"08_tasks/active/{task_id}.yaml"
dump_yaml(out, payload)
registry.setdefault("tasks", []).append({
    "task_id": task_id,
    "title": args.title,
    "status": "planned",
    "owner": args.owner,
    "path": str(out.relative_to(RESEARCH.parent)),
    "related_requirements": args.requirement,
})
dump_yaml(registry_path, registry)
print(out)
