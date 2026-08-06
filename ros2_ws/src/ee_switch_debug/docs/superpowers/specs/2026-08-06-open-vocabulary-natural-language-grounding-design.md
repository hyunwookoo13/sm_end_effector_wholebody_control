# Open-Vocabulary Natural-Language Grounding Design

## Goal

Accept Korean or English pick-and-place instructions without requiring every
object phrase to be registered in an alias table. A local Gemma model extracts
the pick and place visual queries, YOLOE grounds those queries in the camera
images, and the existing navigation and manipulation pipeline executes only
after fresh perception confirms the targets.

For example, the previously rejected command
`오렌지를 분홍색 박스에 넣어줘` must become:

```json
{
  "pick": "orange",
  "place": "pink box",
  "needs_clarification": false
}
```

Neither `orange` nor its Korean spelling should need to be present in a fixed
robot object allowlist.

## Scope

This change is limited to natural-language task interpretation, task-query
validation, and perception grounding. It must preserve the restored stable
behavior of:

- Nav2 long-range obstacle-aware navigation;
- the Nav2-to-precision-control handoff;
- base and arm command switching;
- grasp selection and stale-result rejection;
- the existing pick, transport, place, and return-home state machine.

The first validation scene contains red and blue cans, an apple, an orange,
yellow and pink boxes, and the existing distractors. The implementation must
not encode that scene inventory as the accepted language vocabulary.

Relational references such as `the second cup from the left`, pronouns such as
`put that there`, multi-object batch instructions, and actions other than one
pick followed by one place are outside this first scope. Such commands must
request clarification rather than guess.

## Considered Approaches

1. Expand aliases and the fixed object allowlist. This is fast and
   deterministic but rejected as the primary path because every new object or
   paraphrase requires a code change and it does not satisfy open-vocabulary
   natural-language input.
2. Detect a complete scene inventory before interpreting the instruction, then
   ask the language model to select from that inventory. This provides strong
   grounding but requires broad continuous detection and adds substantial
   inference cost and latency.
3. Use Gemma once to convert the instruction into concise English visual
   queries, then ask YOLOE to ground only those queries. This is selected
   because it preserves the current pipeline, keeps inference bounded, and
   supports previously unseen object names.

## Data Flow

1. The console publishes the original instruction on `/natural_language_task`.
2. The natural-language parser sends the instruction to local Ollama using
   `gemma3:1b`, JSON output mode, and temperature zero.
3. Gemma returns exactly one pick query, one place query, and a clarification
   flag. Queries are short, lowercase English noun phrases suitable for visual
   prompting.
4. The parser performs structural and safety validation without checking the
   queries against a fixed object-name list.
5. A valid task is published on `/pick_place_task` using the existing
   `{"pick": ..., "place": ...}` interface.
6. The existing task manager publishes both queries to the two YOLOE nodes,
   with only the pick query enabled for ROI point-cloud generation.
7. YOLOE uses the queries as open-vocabulary prompts. Existing class aliases
   and confuser prompts may improve detection, but an alias is not required for
   a query to enter perception.
8. Navigation and manipulation begin only after fresh perception produces a
   valid target for the current task. Existing fresh-grasp and source-selection
   gates remain authoritative.
9. If grounding fails, the robot remains stopped and publishes a clear task
   status instead of falling back to an unrelated object.

## Language-Model Contract

Gemma is the primary interpreter for natural-language input. Direct JSON input
remains available for debugging. The rule/alias parser may remain behind an
explicit compatibility parameter, but it is disabled by default and must not
silently replace failed LLM interpretation.

The required response schema is:

```json
{
  "pick": "concise English visual query",
  "place": "concise English visual query",
  "needs_clarification": false,
  "reason": ""
}
```

The prompt instructs the model to preserve visually meaningful attributes such
as color, material, and object class while removing command verbs and Korean
particles. It must set `needs_clarification` when either target is missing,
when the reference is unresolved, or when the instruction is not a single
pick-and-place task.

The Ollama request runs once per user command, not continuously. The model is
kept resident between commands to avoid repeated cold starts. A timeout or
invalid response produces an explicit language-service error and no robot
motion.

## Query Validation

Validation permits new object names but still rejects unsafe or unusable
outputs. Each query must:

- be a non-empty string of bounded length;
- contain a visual noun phrase rather than an action or sentence;
- be free of control characters and topic or command syntax;
- differ from the other query;
- accompany `needs_clarification=false`.

Validation does not claim that a syntactically valid object exists. Existence
is decided by perception grounding. This separates language safety from visual
truth and avoids treating an LLM response as a detection.

## Perception-Grounded Motion Gate

The robot must not move merely because Gemma produced valid JSON. The current
task manager remains responsible for waiting for current-task perception.

- Pick navigation requires a fresh pick detection associated with the current
  query.
- Precision pick and arm descent require a fresh valid grasp from that
  detection.
- Place navigation requires a fresh or safely cached place detection from the
  current task.
- Detection timeout, low-confidence grounding, or missing depth leaves the
  robot stopped and reports which query could not be grounded.
- Previous task detections and grasp poses remain invalid after a new command.

This gate is the replacement for the language-layer object allowlist. It
allows open vocabulary without authorizing motion toward an unobserved target.

## Failure Handling and Status

The parser and task manager expose failures at their actual boundary:

- `nlp_unavailable`: Ollama cannot be reached or times out;
- `invalid_task_json`: the model response violates the response schema;
- `needs_clarification`: the instruction is ambiguous or incomplete;
- `pick_not_grounded`: YOLOE cannot confirm the requested pick target;
- `place_not_grounded`: YOLOE cannot confirm the requested place target;
- existing grasp, navigation, and manipulation failures retain their current
  status names.

No failure path substitutes a different class, reuses a previous task, or
starts navigation speculatively.

## Performance

Gemma runs once when a command arrives and is idle during navigation and arm
control. YOLOE receives only the two extracted target queries, so this design
does not introduce broad scene scanning or continuous language inference. The
existing detector rates, Nav2 rates, and manipulation controller rates remain
unchanged.

The parser records language-model latency and source in task status so startup
latency can be separated from navigation or arm-control latency.

## Tests

Unit tests use mocked Ollama responses and do not require a live model:

- an unseen object command maps `오렌지` to `orange` and `분홍색 박스` to
  `pink box` without aliases;
- other unseen nouns such as `banana` pass structural validation;
- colors and visually relevant modifiers remain in the query;
- a missing pick or place target requests clarification;
- pronouns and unsupported multi-step commands request clarification;
- malformed JSON, timeout, and an empty query publish failure and no task;
- equal pick and place queries are rejected;
- direct JSON remains available for deterministic debugging;
- optional compatibility rules cannot override a failed or ambiguous Gemma
  result unless explicitly enabled.

Integration tests verify:

- arbitrary valid queries reach both YOLOE target topics;
- no navigation command is issued before current-task grounding;
- a missing target times out while the base remains stopped;
- a fresh detection and grasp enter the unchanged existing task state machine;
- red-can and apple tasks continue to follow the current stable behavior;
- stale detections from a previous command cannot start a new task.

Simulation acceptance uses at least the following commands:

```text
빨간 캔을 노란색 박스에 넣어줘
오렌지를 분홍색 박스에 넣어줘
사과를 노란색 박스에 넣어줘
```

Success requires correct target selection, obstacle-aware travel where needed,
the existing precision-control handoff, successful pick and place, no reuse of
stale targets, and no changes to the current base or arm motion tuning.

## Rollout Boundary

Implementation proceeds in three independently verifiable layers:

1. LLM-first parsing and open query validation with mocked unit tests.
2. Query propagation and perception-grounded no-motion failure handling.
3. Isaac Sim acceptance using the restored stable navigation and manipulation
   configuration.

No top-down grasp work, base-speed tuning, controller refactoring, or unrelated
perception optimization is included in this change.
