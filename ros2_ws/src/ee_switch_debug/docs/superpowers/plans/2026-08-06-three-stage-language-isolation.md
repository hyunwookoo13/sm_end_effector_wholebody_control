# Three-Stage Language Isolation Implementation Plan

> **Archived pre-Phase-1 plan — do not execute as written.** Phase 1 moved the
> parser to `ros2_ws/src/sm_natural_language_task/sm_natural_language_task/natural_language_task_parser.py`
> and its focused tests to `ros2_ws/src/sm_natural_language_task/test/`.
> Paths and `ee_switch_debug` test commands below are retained only
> as historical implementation evidence; use the current package paths for any
> follow-up work.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make cross-target attribute copying impossible by extracting source spans once and translating each span in an isolated Gemma request.

**Architecture:** `parse_with_ollama` performs one source-extraction request, validates literal non-overlapping spans, then performs one pick-only and one place-only translation request. It returns the existing combined payload, so ROS topics and every downstream component remain unchanged.

**Tech Stack:** Python 3.10, ROS 2 Humble, Ollama chat API, Gemma 3 4B, pytest

## Global Constraints

- Modify only the natural-language parser, its focused tests, and README.
- Preserve direct JSON input and `/pick_place_task` output.
- Do not modify perception, navigation, switching, or arm control.
- Do not add object or attribute aliases.
- Run all model requests before motion begins.

---

### Task 1: Isolate Extraction and Translation Requests

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py`
- Test: `ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py`

**Interfaces:**
- Produces: `_request_ollama_json(system_prompt, user_text, response_schema, num_predict=64) -> dict[str, Any] | None`
- Produces: `extract_source_spans(text) -> dict[str, Any] | None`
- Produces: `translate_visual_query(source) -> str`
- Preserves: `parse_with_ollama(text) -> dict[str, Any] | None`

- [ ] **Step 1: Write failing request-isolation tests**

Replace the single-request prompt test with a fake Ollama sequence that returns
an extraction payload followed by `apple` and `yellow box` translations. Assert
that exactly three requests were made, the second user message is only `사과`,
and the third user message is only `노란색 박스`.

```python
result = parser.parse_with_ollama("사과를 노란색 박스에 넣어줘")
assert result["pick"] == "apple"
assert result["place"] == "yellow box"
assert len(captured_bodies) == 3
assert captured_bodies[1]["messages"][1]["content"] == "사과"
assert captured_bodies[2]["messages"][1]["content"] == "노란색 박스"
```

- [ ] **Step 2: Verify RED**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
```

Expected: the isolation test fails because the parser makes one request.

- [ ] **Step 3: Add a shared Ollama JSON request helper**

Move the common request construction, POST, response decoding, and JSON content
extraction from `parse_with_ollama` into `_request_ollama_json`. Preserve
`keep_alive=30m`, `temperature=0`, `num_ctx=2048`, and the existing timeout.

- [ ] **Step 4: Add extraction-only and translation-only helpers**

`extract_source_spans` must request only `pick_source`, `place_source`,
`needs_clarification`, and `reason`. `translate_visual_query` must request only
`{"query": ...}` and receive exactly one source phrase as its user message.

- [ ] **Step 5: Compose the three stages**

In `parse_with_ollama`, wait for warmup, extract, return clarification unchanged
when required, reject invalid source spans before translation, translate the two
sources independently, and return the existing combined payload.

- [ ] **Step 6: Verify GREEN and commit**

Run the Task 1 test command. Expected: every parser test passes.

```bash
git add \
  src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git commit -m "fix: isolate pick and place translation requests"
```

---

### Task 2: Integrated Verification

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Documents: three-stage interpretation and one-time startup latency
- Verifies: unchanged downstream behavior

- [ ] **Step 1: Update README wording**

State that source extraction is followed by two isolated translations and that
the 2.3-3.0 second language delay occurs only before motion.

- [ ] **Step 2: Build and run all tests**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  sm_florence_2_vlm_ros2 sm_grasping_ros2 ee_switch_debug
source install/setup.bash
colcon test --packages-select ee_switch_debug
colcon test-result --all --verbose
/usr/bin/python3 -m pytest -q \
  src/sm_florence_2_vlm_ros2/test src/sm_grasping_ros2/test
ament_flake8 \
  src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git diff --check
```

- [ ] **Step 3: Run live Gemma acceptance**

Assert all five design commands, including both unqualified apple commands,
produce the exact expected pick/place queries and valid source spans. Record
per-command latency.

- [ ] **Step 4: Commit documentation and confirm scope**

```bash
git add \
  src/ee_switch_debug/README_PICK_PLACE.md \
  src/ee_switch_debug/docs/superpowers/specs/2026-08-06-pick-place-attribute-isolation-design.md \
  src/ee_switch_debug/docs/superpowers/plans/2026-08-06-three-stage-language-isolation.md
git commit -m "docs: describe isolated language translation"
git status --short
```

Expected: only the unrelated local `dds_setting/` remains untracked.
