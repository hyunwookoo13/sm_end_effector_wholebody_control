# Open-Vocabulary Natural-Language Grounding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make local Gemma the primary natural-language task interpreter and pass previously unseen visual object queries to YOLOE without a fixed object-name allowlist.

**Architecture:** The parser asks `gemma3:1b` for one concise English pick query and one concise English place query, validates only their structure, and publishes the existing `/pick_place_task` JSON. The current task manager and YOLOE pipeline ground those queries before navigation, so Nav2, precision control, and arm control remain unchanged.

**Tech Stack:** ROS 2 Humble, Python 3.10, `rclpy`, Ollama `/api/chat`, Gemma 3 1B, YOLOE, pytest, colcon.

## Global Constraints

- Do not modify Nav2 configuration, velocity limits, base/arm switching, grasp control, or the pick/place state machine.
- Do not reintroduce top-down grasp code.
- Call Gemma once per command; do not run language inference continuously.
- Do not authorize motion from language output alone; existing fresh detection and fresh grasp gates remain authoritative.
- Do not silently substitute an alias result after an LLM timeout, malformed response, or clarification response.
- Keep direct JSON input for deterministic debugging.
- Preserve the untracked `ros2_ws/dds_setting/` directory without committing it.

---

### Task 1: Open-vocabulary query validation

**Files:**
- Create: `ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py`
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py`

**Interfaces:**
- Consumes: model payloads shaped as `dict[str, Any] | None`.
- Produces: `normalize_visual_query(value: Any) -> str` and an updated `validate_result(payload)` that accepts new English visual noun phrases without `allowed_objects`.

- [ ] **Step 1: Write failing validation tests**

```python
def make_parser():
    parser = NaturalLanguageTaskParser.__new__(NaturalLanguageTaskParser)
    parser.max_query_length = 80
    return parser


def test_validate_result_accepts_unregistered_visual_queries():
    result = make_parser().validate_result(
        {"pick": "orange", "place": "pink box", "needs_clarification": False}
    )
    assert result == (True, "ok", "orange", "pink box")


def test_validate_result_accepts_another_unseen_object():
    result = make_parser().validate_result(
        {"pick": "ripe banana", "place": "wooden tray", "needs_clarification": False}
    )
    assert result == (True, "ok", "ripe banana", "wooden tray")


def test_validate_result_rejects_unsafe_or_equal_queries():
    parser = make_parser()
    assert not parser.validate_result({"pick": "orange\n/cmd_vel", "place": "pink box"})[0]
    assert not parser.validate_result({"pick": "orange", "place": "orange"})[0]
```

- [ ] **Step 2: Run the focused test and verify failure**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
```

Expected: FAIL because the current parser requires `allowed_objects`.

- [ ] **Step 3: Implement bounded structural validation**

Add a `max_query_length` parameter with default `80`. Normalize whitespace and lowercase text, require an ASCII letter, and permit only letters, digits, spaces, periods, apostrophes, underscores, and hyphens.

```python
def normalize_visual_query(self, value: Any) -> str:
    query = re.sub(r"\s+", " ", str(value).strip().lower())
    if not query or len(query) > self.max_query_length:
        return ""
    if not re.fullmatch(r"[a-z0-9][a-z0-9 ._'\-]*", query):
        return ""
    return query if re.search(r"[a-z]", query) else ""
```

Use it in `validate_result`, remove allowlist membership checks, and retain clarification and same-target rejection.

- [ ] **Step 4: Run focused tests and commit**

```bash
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py \
  src/ee_switch_debug/test/test_natural_language_color_aliases.py
git add ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git commit -m "feat: accept open-vocabulary visual task queries"
```

Expected: all tests PASS before the commit.

### Task 2: Gemma-first interpretation and failure isolation

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py`
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py`

**Interfaces:**
- Consumes: `/natural_language_task` strings and Ollama JSON responses.
- Produces: `/pick_place_task` only after valid Gemma output, and explicit `/natural_language_task_status` failures.

- [ ] **Step 1: Add failing dispatch-order tests**

```python
def test_natural_language_uses_ollama_before_rules():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: {
        "pick": "orange", "place": "pink box", "needs_clarification": False
    }
    parser.parse_with_rules = lambda text: (_ for _ in ()).throw(AssertionError("rules called"))
    parser.on_natural_language_task(String(data="오렌지를 분홍색 박스에 넣어줘"))
    assert json.loads(parser.task_pub.messages[-1].data) == {
        "pick": "orange", "place": "pink box"
    }


def test_ollama_timeout_does_not_fall_back_to_rules():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: (_ for _ in ()).throw(TimeoutError("timeout"))
    parser.on_natural_language_task(String(data="오렌지를 분홍색 박스에 넣어줘"))
    assert parser.task_pub.messages == []
    status = json.loads(parser.status_pub.messages[-1].data)
    assert status["reason"].startswith("nlp_unavailable")
```

Also test that `needs_clarification=true` publishes no task and direct JSON bypasses Ollama.

- [ ] **Step 2: Run the tests and verify the current rule-first order fails**

```bash
/usr/bin/python3 -m pytest -q src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
```

- [ ] **Step 3: Implement Gemma-first dispatch**

Use this callback order:

1. Direct JSON when the input begins with `{`.
2. Ollama when `use_ollama=true`.
3. Alias rules only when Ollama is explicitly disabled and `use_rule_fallback=true`.
4. Ollama exception publishes `nlp_unavailable: <exception>` and returns.
5. Invalid or ambiguous model output publishes no task and never falls back silently.

Remove `allowed_objects`; remove its filtering from rule compatibility; set `use_rule_fallback` default to `False`.

- [ ] **Step 4: Update the Ollama contract**

Prompt Gemma for concise lowercase English visual noun phrases, preserve visual attributes, and request clarification for missing, pronoun-only, non-pick/place, or multi-step commands. Require:

```json
{"pick":"orange","place":"pink box","needs_clarification":false,"reason":""}
```

Keep JSON mode and temperature zero. Add `"keep_alive": "30m"` to prevent repeated cold starts.

- [ ] **Step 5: Run tests and commit**

```bash
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py \
  src/ee_switch_debug/test/test_natural_language_color_aliases.py
git add ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git commit -m "feat: make Gemma primary for natural-language tasks"
```

Expected: all tests PASS.

### Task 3: Launch defaults and grounding regressions

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py`
- Modify: `ros2_ws/src/ee_switch_debug/test/test_pick_place_precache.py`
- Modify: `ros2_ws/src/sm_florence_2_vlm_ros2/test/test_yoloe_utils.py`
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Consumes: arbitrary validated `pick` and `place` strings.
- Produces: the existing YOLOE target payload and unchanged `FIND_PICK` no-motion gate.

- [ ] **Step 1: Add open-query propagation tests**

```python
def test_open_vocabulary_queries_reach_perception_without_rewriting():
    payload = PickPlaceTaskManager.build_perception_payload("orange", "pink box")
    assert json.loads(payload) == {
        "target_objects": ["orange", "pink box"],
        "roi_target_objects": ["orange"],
    }


def test_find_pick_does_not_navigate_without_fresh_grounding():
    manager = PickPlaceTaskManager.__new__(PickPlaceTaskManager)
    manager.phase = "FIND_PICK"
    manager.pick_object = "orange"
    manager.pick_transform = None
    manager.last_debug = ""
    manager.request_navigation = lambda *args: (_ for _ in ()).throw(
        AssertionError("navigation requested before grounding")
    )
    manager.advance_phase()
    assert manager.phase == "FIND_PICK"
    assert manager.last_debug == "waiting for pick grasp: orange"
```

Add `assert expand_yoloe_prompts(["banana"])[0] == "banana"` to verify an unseen class enters YOLOE unchanged.

- [ ] **Step 2: Run focused grounding tests**

```bash
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_pick_place_precache.py \
  src/sm_florence_2_vlm_ros2/test/test_yoloe_utils.py
```

Expected: PASS without production controller changes.

- [ ] **Step 3: Update launch defaults**

Remove the `allowed_objects` launch parameter. Add `use_rule_task_parser` with default `false` and pass it to `use_rule_fallback`. Keep `use_local_llm=true`, `local_llm_model=gemma3:1b`, and the existing Ollama URL.

- [ ] **Step 4: Update runtime documentation**

Document that Gemma is primary, unseen objects need no alias, YOLOE grounding is required before motion, language failures cause no motion, and `use_rule_task_parser:=true use_local_llm:=false` is compatibility-only. Add `오렌지를 분홍색 박스에 넣어줘` as an acceptance command.

- [ ] **Step 5: Verify and commit**

```bash
/usr/bin/python3 -m py_compile \
  src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py \
  src/ee_switch_debug/test/test_pick_place_precache.py \
  src/sm_florence_2_vlm_ros2/test/test_yoloe_utils.py
git add ros2_ws/src/ee_switch_debug/launch/florence_long_range_pick_place_control.launch.py \
  ros2_ws/src/ee_switch_debug/test/test_pick_place_precache.py \
  ros2_ws/src/sm_florence_2_vlm_ros2/test/test_yoloe_utils.py \
  ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md
git commit -m "feat: connect open language queries to grounded tasks"
```

Expected: syntax and tests PASS before the commit.

### Task 4: Full verification and live Gemma smoke test

**Files:**
- No production file changes expected.

**Interfaces:**
- Consumes: completed parser and existing ROS packages.
- Produces: build/test evidence and one live Gemma parse for the orange command.

- [ ] **Step 1: Build the three workspace packages**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  sm_florence_2_vlm_ros2 sm_grasping_ros2 ee_switch_debug
```

Expected: three packages finish with exit zero.

- [ ] **Step 2: Run all tests**

```bash
source install/setup.bash
colcon test --packages-select ee_switch_debug --event-handlers console_cohesion+
colcon test-result --all --verbose
/usr/bin/python3 -m pytest -q \
  src/sm_florence_2_vlm_ros2/test src/sm_grasping_ros2/test
```

Expected: zero errors and zero failures.

- [ ] **Step 3: Verify the live model**

```bash
curl -fsS http://127.0.0.1:11434/api/tags
```

Invoke `parse_with_ollama` in a short ROS-aware Python process with `오렌지를 분홍색 박스에 넣어줘`. Expected:

```json
{"pick":"orange","place":"pink box","needs_clarification":false}
```

If Ollama or the model is unavailable, report the environment blocker; do not substitute rule parsing.

- [ ] **Step 4: Verify repository scope**

Confirm that no Nav2, controller, arm, velocity, or top-down files changed and that `ros2_ws/dds_setting/` remains the only unrelated untracked path.

- [ ] **Step 5: Isaac Sim acceptance handoff**

Test in order without changing motion tuning:

```text
빨간 캔을 노란색 박스에 넣어줘
오렌지를 분홍색 박스에 넣어줘
사과를 노란색 박스에 넣어줘
```

Record parser status, YOLOE target updates, task-manager phases, and the final physical placement.
