# Pick/Place Attribute Isolation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Prevent destination attributes from leaking into an unqualified pick query while preserving open-vocabulary natural-language input.

**Architecture:** Extend the single Ollama response with literal `pick_source` and `place_source` spans copied from the original command. Validate that those spans are present and non-overlapping before publishing the existing `pick`/`place` task JSON; keep every perception, navigation, and manipulation interface unchanged.

**Tech Stack:** Python 3.10, ROS 2 Humble `rclpy`, Ollama chat API, Gemma 3 4B, pytest, ament_flake8

## Global Constraints

- Modify only the natural-language parser, its focused tests, and user documentation.
- Do not modify YOLOE, grasp generation, the task manager, Nav2, precision control, base/arm switching, or manipulation controllers.
- Do not add object, color, or material aliases.
- Keep direct JSON input backward compatible.
- Keep one Ollama request per natural-language command.
- Reject invalid source ownership before publishing `/pick_place_task`.

---

### Task 1: Validate Literal Pick and Place Source Spans

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py`
- Test: `ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py`

**Interfaces:**
- Consumes: original command `text: str` and Ollama `payload: dict[str, Any] | None`
- Produces: `validate_source_spans(text, payload) -> tuple[bool, str]`
- Preserves: `validate_result(payload) -> tuple[bool, str, str | None, str | None]`

- [ ] **Step 1: Write failing source-span validation tests**

Add focused tests using the real parser helper without a ROS node:

```python
def test_source_spans_accept_distinct_phrases_from_original_command():
    parser = make_parser_without_ros_node()
    result = parser.validate_source_spans(
        "사과를 노란색 박스에 넣어줘",
        {
            "pick_source": "사과",
            "place_source": "노란색 박스",
            "pick": "apple",
            "place": "yellow box",
        },
    )
    assert result == (True, "ok")


def test_source_spans_reject_invented_phrase():
    parser = make_parser_without_ros_node()
    result = parser.validate_source_spans(
        "사과를 노란색 박스에 넣어줘",
        {"pick_source": "빨간 사과", "place_source": "노란색 박스"},
    )
    assert result == (False, "pick source is not present in command")


def test_source_spans_reject_overlap():
    parser = make_parser_without_ros_node()
    result = parser.validate_source_spans(
        "노란색 박스를 옮겨줘",
        {"pick_source": "노란색 박스", "place_source": "박스"},
    )
    assert result == (False, "pick and place source spans overlap")
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py \
  -k source_spans
```

Expected: FAIL because `NaturalLanguageTaskParser` has no
`validate_source_spans` method.

- [ ] **Step 3: Implement the minimal source-span validator**

Add a helper that treats English input case-insensitively, accepts Korean case
particles inside a literal source span, and rejects missing or overlapping
spans:

```python
def validate_source_spans(
    self,
    text: str,
    payload: dict[str, Any] | None,
) -> tuple[bool, str]:
    if not isinstance(payload, dict):
        return False, "parser returned no JSON object"

    command = text.casefold()
    spans = []
    for field, label in (
        ("pick_source", "pick"),
        ("place_source", "place"),
    ):
        source = str(payload.get(field, "")).strip()
        if not source:
            return False, f"missing {label} source"
        start = command.find(source.casefold())
        if start < 0:
            return False, f"{label} source is not present in command"
        spans.append((start, start + len(source)))

    pick_span, place_span = spans
    if pick_span[0] < place_span[1] and place_span[0] < pick_span[1]:
        return False, "pick and place source spans overlap"
    return True, "ok"
```

- [ ] **Step 4: Run focused tests and verify GREEN**

Run the command from Step 2.

Expected: all selected source-span tests PASS.

- [ ] **Step 5: Commit the validator and tests**

```bash
git add \
  src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git commit -m "test: validate natural language source ownership"
```

---

### Task 2: Require Independent Source Extraction in the Ollama Path

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py`
- Test: `ros2_ws/src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py`

**Interfaces:**
- Consumes: the Task 1 `validate_source_spans` helper
- Produces: Ollama schema fields `pick_source`, `place_source`, `pick`, `place`, `needs_clarification`, and `reason`
- Publishes: unchanged `{"pick": ..., "place": ...}` on `/pick_place_task`

- [ ] **Step 1: Write failing request-schema and publication-gate tests**

Extend the existing fake Ollama response test to require both source fields in
the JSON schema and source-isolation language in the system prompt:

```python
required = set(captured["body"]["format"]["required"])
assert {"pick_source", "place_source"} <= required
system_prompt = captured["body"]["messages"][0]["content"]
assert "exact character substrings" in system_prompt
assert "Never copy" in system_prompt
```

Add a message-level rejection test:

```python
def test_ollama_result_with_invented_source_publishes_no_task():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: {
        "pick_source": "노란색 사과",
        "place_source": "노란색 박스",
        "pick": "yellow apple",
        "place": "yellow box",
        "needs_clarification": False,
        "reason": "",
    }

    parser.on_natural_language_task(String(data="사과를 노란색 박스에 넣어줘"))

    assert parser.task_pub.messages == []
    status = json.loads(parser.status_pub.messages[-1].data)
    assert status["reason"] == "pick source is not present in command"
```

- [ ] **Step 2: Run the focused tests and verify RED**

Run:

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 -m pytest -q \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
```

Expected: FAIL because the request schema does not require source fields and
the message path does not call `validate_source_spans`.

- [ ] **Step 3: Update the prompt and constrained JSON schema**

Require literal source phrases and independent translation without concrete
object examples:

```python
system_prompt = (
    "Extract the pick object and destination from one Korean or English "
    "pick-and-place command. pick_source and place_source MUST be exact "
    "character substrings copied from the user's original command, in the "
    "original language; never translate these two source fields and omit only "
    "Korean particles. Translate pick_source and place_source independently "
    "into lowercase English visual noun phrases. Never copy a color, material, "
    "state, or object class from one source phrase to the other, and never "
    "invent an omitted attribute. Return JSON only."
)
```

Add `pick_source` and `place_source` string properties to `response_schema` and
include both in its `required` list.

- [ ] **Step 4: Gate Ollama results before generic query validation**

In `on_natural_language_task`, after a successful Ollama response and before
`validate_result`, call:

```python
if source == "ollama":
    sources_valid, source_reason = self.validate_source_spans(text, parsed)
    if not sources_valid:
        self.publish_status(False, source_reason, source=source, text=text)
        self.get_logger().warn(
            f"Natural language task rejected: {source_reason}; text={text!r}"
        )
        return
```

Do not run this gate for direct JSON or explicit rule-compatibility input.

- [ ] **Step 5: Run parser tests and verify GREEN**

Run the command from Step 2.

Expected: all parser tests PASS, including unchanged direct JSON behavior.

- [ ] **Step 6: Commit the Ollama-path change**

```bash
git add \
  src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git commit -m "fix: isolate pick and place language attributes"
```

---

### Task 3: Document and Verify the Integrated Behavior

**Files:**
- Modify: `ros2_ws/src/ee_switch_debug/README_PICK_PLACE.md`

**Interfaces:**
- Documents: natural-language source isolation and rejection behavior
- Verifies: unchanged ROS package build and control-path regression tests

- [ ] **Step 1: Document the ownership behavior**

Add a concise note to the natural-language section:

```text
The parser first separates the original pick and place phrases, then translates
them independently. A destination attribute is not copied to an unqualified
pick object: `사과를 노란색 박스에 넣어줘` becomes
`{"pick":"apple","place":"yellow box"}`.
```

- [ ] **Step 2: Build all affected packages**

```bash
cd /home/kiro/Desktop/hw_ws/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  sm_florence_2_vlm_ros2 sm_grasping_ros2 ee_switch_debug
```

Expected: 3 packages finish successfully.

- [ ] **Step 3: Run all automated regression tests and lint**

```bash
source install/setup.bash
colcon test --packages-select ee_switch_debug
colcon test-result --all --verbose
/usr/bin/python3 -m pytest -q \
  src/sm_florence_2_vlm_ros2/test \
  src/sm_grasping_ros2/test
ament_flake8 \
  src/ee_switch_debug/ee_switch_debug/natural_language_task_parser.py \
  src/ee_switch_debug/test/test_open_vocabulary_natural_language_parser.py
git diff --check
```

Expected: zero build errors, test failures, lint errors, or whitespace errors.

- [ ] **Step 4: Run live Gemma acceptance cases**

Call the installed parser's `parse_with_ollama` and
`validate_source_spans` methods for every exact acceptance command in the
design. Assert the expected pick/place tuple and print per-command latency.

Expected:

```text
사과를 노란색 박스에 넣어줘 -> apple, yellow box
사과를 분홍색 박스에 넣어줘 -> apple, pink box
오렌지를 분홍색 박스에 넣어줘 -> orange, pink box
빨간 캔을 노란색 박스에 넣어줘 -> red can, yellow box
잘 익은 바나나를 나무 트레이에 올려줘 -> ripe banana, wooden tray
```

- [ ] **Step 5: Confirm file scope and commit documentation**

```bash
git status --short
git diff --name-only HEAD~2
git add src/ee_switch_debug/README_PICK_PLACE.md
git commit -m "docs: explain language attribute isolation"
```

Expected: only the parser, its focused test, README, design, and plan are part
of this change. Preserve the unrelated untracked `dds_setting/` directory.
