from __future__ import annotations

from pathlib import Path
from common import ROOT, RESEARCH, load_yaml, today_iso, iso_week, git_commit, git_dirty, dump_yaml

project = load_yaml(ROOT / "PROJECT.yaml") or {}
rqs = load_yaml(RESEARCH / "00_constitution/research_questions.yaml") or {}
reqs = load_yaml(RESEARCH / "01_requirements/requirements.yaml") or {}
status = load_yaml(RESEARCH / "05_progress/current_status.yaml") or {}
roadmap = load_yaml(RESEARCH / "05_progress/roadmap.yaml") or {}
experiments = load_yaml(RESEARCH / "03_experiments/experiment_registry.yaml") or {}
evidence = load_yaml(RESEARCH / "03_experiments/evidence_registry.yaml") or {}
claims = load_yaml(RESEARCH / "00_constitution/core_claims.yaml") or {}
tasks = load_yaml(RESEARCH / "08_tasks/task_registry.yaml") or {}
slides = load_yaml(RESEARCH / "07_presentation/slide_manifest.yaml") or {}

def bullets(values):
    if not values:
        return "- None"
    return "\n".join(f"- {v}" for v in values)

active_requirements = [
    f"{r['id']} [{r.get('priority')}] — {r.get('statement')}"
    for r in reqs.get("requirements", []) if r.get("status") in {"active", "blocked"}
]
active_tasks = [
    f"{t.get('task_id')} [{t.get('status')}] — {t.get('title')}"
    for t in tasks.get("tasks", []) if t.get("status") not in {"completed", "retired"}
]
exp_lines = [
    f"{e.get('id')} [{e.get('status')}] — {e.get('title')}"
    for e in experiments.get("experiments", [])
]
evidence_lines = [
    f"{e.get('id')} [{e.get('status')}] — {e.get('title')}"
    for e in evidence.get("evidence", [])
]
claim_lines = [
    f"{c.get('id')} [{c.get('state')}] — {c.get('statement')}"
    for c in claims.get("claims", [])
]

p = project.get("project", {})
north = project.get("north_star", {}).get("statement", "TBD")
text = f"""# Research Meeting Packet — {today_iso()}\n\n## 1. North Star\n\n**Project:** {p.get('working_title', p.get('name', 'TBD'))}\n\n> {north}\n\n**Phase:** {status.get('phase', 'TBD')}  \n**Health:** {status.get('health', 'TBD')}  \n**Git commit:** `{git_commit()}`  \n**Dirty:** `{git_dirty()}`\n\n## 2. Research Questions\n\n{bullets([f"{x.get('id')} — {x.get('question')}" for x in rqs.get('research_questions', [])])}\n\n## 3. Active Requirements\n\n{bullets(active_requirements)}\n\n## 4. Architecture and Roadmap\n\n- Architecture: `research/02_architecture/system_architecture.md`\n- Current focus:\n{bullets(status.get('current_focus', []))}\n\n- Roadmap phases:\n{bullets([f"{x.get('phase_id')} [{x.get('status')}] — {x.get('name')} ({x.get('window')})" for x in roadmap.get('roadmap', [])])}\n\n## 5. Progress\n\n### Completed\n{bullets(status.get('progress_summary', {}).get('completed', []))}\n\n### In progress\n{bullets(status.get('progress_summary', {}).get('in_progress', []))}\n\n### Not started\n{bullets(status.get('progress_summary', {}).get('not_started', []))}\n\n## 6. Tasks\n\n{bullets(active_tasks)}\n\n## 7. Experiments and Evidence\n\n### Experiments\n{bullets(exp_lines)}\n\n### Evidence\n{bullets(evidence_lines)}\n\n## 8. Claim Readiness\n\n{bullets(claim_lines)}\n\n## 9. Blockers and Decisions Needed\n\n### Blockers\n{bullets(status.get('blockers', []))}\n\n### Open decisions\n{bullets(status.get('open_decisions', []))}\n\n## 10. Cumulative Slide Contract\n\n- Slide count: {len(slides.get('slides', []))}\n- Mode: {slides.get('deck', {}).get('mode', 'TBD')}\n- Preserve previous structure: {slides.get('deck', {}).get('preserve_previous_structure', 'TBD')}\n\n## 11. Next Steps\n\n{bullets(status.get('current_focus', []))}\n"""

report_dir = ROOT / "exports/reports"
report_dir.mkdir(parents=True, exist_ok=True)
out = report_dir / f"meeting_packet_{today_iso()}.md"
out.write_text(text, encoding="utf-8")

snapshot = dict(slides)
snapshot.setdefault("generated", {})
snapshot["generated"].update({
    "date": today_iso(), "week": iso_week(), "git_commit": git_commit(), "git_dirty": git_dirty(),
    "meeting_packet": str(out.relative_to(ROOT)),
})
slide_dir = ROOT / "exports/slides"
slide_dir.mkdir(parents=True, exist_ok=True)
slide_out = slide_dir / f"slide_manifest_snapshot_{today_iso()}.yaml"
dump_yaml(slide_out, snapshot)

print(out)
print(slide_out)
