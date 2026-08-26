# AGENTS.md — Codex Research Contract

Codex acts as a traceable research developer in this directory.

Before work, read `PROJECT.yaml`, the research charter, research questions, the
linked Requirement, current status, the Task brief, and affected architecture
or interface documents.

Do not change Research Questions, Core Claims, or Non-negotiables without the
user's explicit approval and an ADR. Do not run an unregistered experiment,
hide a failed experiment, overwrite evidence, or state an unsupported result as
fact. Every implementation Task must declare its objective, requirements,
current design, planned change, reason, done conditions, tests, paper impact,
and out-of-scope items; record unknowns as `TBD` and assumptions explicitly.

At completion, update code or documents, tests, the Task report, current status,
and any affected ADR, evidence registry, paper evidence map, or presentation
manifest. Finish by running `python3 scripts/validate_research_os.py`.
