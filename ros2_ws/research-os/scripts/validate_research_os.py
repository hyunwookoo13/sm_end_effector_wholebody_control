from __future__ import annotations

from pathlib import Path
import sys
from common import ROOT, RESEARCH, load_yaml

REQUIRED_FILES = [
    "PROJECT.yaml",
    "AGENTS.md",
    "research/00_constitution/research_charter.md",
    "research/00_constitution/research_questions.yaml",
    "research/00_constitution/core_claims.yaml",
    "research/01_requirements/requirements.yaml",
    "research/01_requirements/traceability.yaml",
    "research/02_architecture/module_registry.yaml",
    "research/03_experiments/experiment_registry.yaml",
    "research/03_experiments/evidence_registry.yaml",
    "research/05_progress/current_status.yaml",
    "research/06_paper/evidence_map.yaml",
    "research/07_presentation/slide_manifest.yaml",
    "research/08_tasks/task_registry.yaml",
    "research/10_literature/paper_registry.yaml",
]

errors: list[str] = []
warnings: list[str] = []

for rel in REQUIRED_FILES:
    if not (ROOT / rel).exists():
        errors.append(f"Missing required file: {rel}")

if errors:
    for e in errors:
        print(f"ERROR: {e}")
    raise SystemExit(1)

rqs_data = load_yaml(RESEARCH / "00_constitution/research_questions.yaml") or {}
claims_data = load_yaml(RESEARCH / "00_constitution/core_claims.yaml") or {}
reqs_data = load_yaml(RESEARCH / "01_requirements/requirements.yaml") or {}
mods_data = load_yaml(RESEARCH / "02_architecture/module_registry.yaml") or {}
exps_data = load_yaml(RESEARCH / "03_experiments/experiment_registry.yaml") or {}
ev_data = load_yaml(RESEARCH / "03_experiments/evidence_registry.yaml") or {}
trace_data = load_yaml(RESEARCH / "01_requirements/traceability.yaml") or {}
paper_map = load_yaml(RESEARCH / "06_paper/evidence_map.yaml") or {}
slide_data = load_yaml(RESEARCH / "07_presentation/slide_manifest.yaml") or {}
task_data = load_yaml(RESEARCH / "08_tasks/task_registry.yaml") or {}
lit_data = load_yaml(RESEARCH / "10_literature/paper_registry.yaml") or {}


def ids(items, key="id"):
    return [x.get(key) for x in items if isinstance(x, dict) and x.get(key)]


def check_unique(name, values):
    seen = set()
    for value in values:
        if value in seen:
            errors.append(f"Duplicate {name}: {value}")
        seen.add(value)

rqs = rqs_data.get("research_questions", [])
claims = claims_data.get("claims", [])
reqs = reqs_data.get("requirements", [])
mods = mods_data.get("modules", [])
exps = exps_data.get("experiments", [])
evidence = ev_data.get("evidence", [])
slides = slide_data.get("slides", [])
tasks = task_data.get("tasks", [])
papers = lit_data.get("papers", [])

rq_ids = set(ids(rqs))
claim_ids = set(ids(claims))
req_ids = set(ids(reqs))
mod_ids = set(ids(mods))
exp_ids = set(ids(exps))
evidence_ids = set(ids(evidence))
slide_ids = set(ids(slides))
task_ids = set(ids(tasks, "task_id"))
paper_ids = set(ids(papers))

for name, values in [
    ("RQ", ids(rqs)), ("Claim", ids(claims)), ("Requirement", ids(reqs)),
    ("Module", ids(mods)), ("Experiment", ids(exps)), ("Evidence", ids(evidence)),
    ("Slide", ids(slides)), ("Task", ids(tasks, "task_id")), ("Paper", ids(papers)),
]:
    check_unique(name, values)

for req in reqs:
    for rq in req.get("linked_rq", []) or []:
        if rq not in rq_ids:
            errors.append(f"{req.get('id')} links unknown RQ: {rq}")

for claim in claims:
    for req in claim.get("linked_requirements", []) or []:
        if req not in req_ids:
            errors.append(f"{claim.get('id')} links unknown Requirement: {req}")
    for ev in claim.get("evidence", []) or []:
        if ev not in evidence_ids:
            errors.append(f"{claim.get('id')} links unknown Evidence: {ev}")
    if claim.get("state") == "supported" and not claim.get("evidence"):
        errors.append(f"Supported claim has no evidence: {claim.get('id')}")

for exp in exps:
    for claim in exp.get("linked_claims", []) or []:
        if claim not in claim_ids:
            errors.append(f"{exp.get('id')} links unknown Claim: {claim}")
    for req in exp.get("linked_requirements", []) or []:
        if req not in req_ids:
            errors.append(f"{exp.get('id')} links unknown Requirement: {req}")
    for ev in exp.get("evidence", []) or []:
        if ev not in evidence_ids:
            errors.append(f"{exp.get('id')} links unknown Evidence: {ev}")

for ev in evidence:
    source = ev.get("source_experiment")
    if source and source not in exp_ids:
        errors.append(f"{ev.get('id')} links unknown Experiment: {source}")
    for claim in ev.get("supports_claims", []) or []:
        if claim not in claim_ids:
            errors.append(f"{ev.get('id')} supports unknown Claim: {claim}")

for link in trace_data.get("links", []) or []:
    rid = link.get("requirement")
    if rid not in req_ids:
        errors.append(f"Traceability links unknown Requirement: {rid}")
    for mid in link.get("architecture", []) or []:
        if mid not in mod_ids:
            errors.append(f"Traceability for {rid} links unknown Module: {mid}")
    for eid in link.get("experiments", []) or []:
        if eid not in exp_ids:
            errors.append(f"Traceability for {rid} links unknown Experiment: {eid}")
    for cid in link.get("claims", []) or []:
        if cid not in claim_ids:
            errors.append(f"Traceability for {rid} links unknown Claim: {cid}")
    for sid in link.get("slides", []) or []:
        if sid not in slide_ids:
            errors.append(f"Traceability for {rid} links unknown Slide: {sid}")

for link in paper_map.get("links", []) or []:
    claim = link.get("claim")
    if claim not in claim_ids:
        errors.append(f"Paper map links unknown Claim: {claim}")
    for ev in link.get("evidence", []) or []:
        if ev not in evidence_ids:
            errors.append(f"Paper map for {claim} links unknown Evidence: {ev}")

for task in tasks:
    for req in task.get("related_requirements", []) or []:
        if req not in req_ids:
            errors.append(f"{task.get('task_id')} links unknown Requirement: {req}")

active_must = [r for r in reqs if r.get("priority") == "must" and r.get("status") == "active"]
traced_reqs = {x.get("requirement") for x in trace_data.get("links", []) or []}
for req in active_must:
    if req.get("id") not in traced_reqs:
        warnings.append(f"Active MUST requirement has no traceability entry yet: {req.get('id')}")

for warning in warnings:
    print(f"WARNING: {warning}")

if errors:
    for error in errors:
        print(f"ERROR: {error}")
    print(f"\nValidation failed with {len(errors)} error(s) and {len(warnings)} warning(s).")
    raise SystemExit(1)

print(f"Research OS validation passed: {len(warnings)} warning(s).")
print(f"RQ={len(rq_ids)}, Claims={len(claim_ids)}, Requirements={len(req_ids)}, Modules={len(mod_ids)}, Experiments={len(exp_ids)}, Evidence={len(evidence_ids)}, Slides={len(slide_ids)}, Tasks={len(task_ids)}, Papers={len(paper_ids)}")
