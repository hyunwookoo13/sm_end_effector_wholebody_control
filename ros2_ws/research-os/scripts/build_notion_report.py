from __future__ import annotations

from common import ROOT, RESEARCH, load_yaml, today_iso, iso_week, git_commit

status = load_yaml(RESEARCH / "05_progress/current_status.yaml") or {}
experiments = load_yaml(RESEARCH / "03_experiments/experiment_registry.yaml") or {}
evidence = load_yaml(RESEARCH / "03_experiments/evidence_registry.yaml") or {}
claims = load_yaml(RESEARCH / "00_constitution/core_claims.yaml") or {}

def bullets(values):
    return "\n".join(f"- {v}" for v in values) if values else "- None"

text = f"""# {iso_week()} 연구 진행 보고\n\n## 한 문장 상태\n\n{status.get('phase', 'TBD')} — health: **{status.get('health', 'TBD')}**\n\n## 이번 주 완료\n\n{bullets(status.get('progress_summary', {}).get('completed', []))}\n\n## 진행 중\n\n{bullets(status.get('progress_summary', {}).get('in_progress', []))}\n\n## 미완료 및 원인\n\n{bullets(status.get('progress_summary', {}).get('not_started', []))}\n\n## 최신 실험\n\n{bullets([f"{x.get('id')} [{x.get('status')}] {x.get('title')}" for x in experiments.get('experiments', [])])}\n\n## Evidence\n\n{bullets([f"{x.get('id')} [{x.get('status')}] {x.get('title')}" for x in evidence.get('evidence', [])])}\n\n## Claim 상태\n\n{bullets([f"{x.get('id')} [{x.get('state')}]" for x in claims.get('claims', [])])}\n\n## Blockers\n\n{bullets(status.get('blockers', []))}\n\n## 결정이 필요한 사항\n\n{bullets(status.get('open_decisions', []))}\n\n## 다음 우선순위\n\n{bullets(status.get('current_focus', []))}\n\n## 추적 정보\n\n- Generated: {today_iso()}\n- Git commit: `{git_commit()}`\n- Source: GitHub Research OS\n"""

out_dir = ROOT / "exports/notion"
out_dir.mkdir(parents=True, exist_ok=True)
out = out_dir / f"notion_weekly_{iso_week()}.md"
out.write_text(text, encoding="utf-8")
print(out)
