# Pick/Place Attribute Isolation Design

## Context

The open-vocabulary natural-language parser correctly handles commands such as
`빨간 캔을 노란색 박스에 넣어줘`, but Gemma 3 4B currently translates
`사과를 노란색 박스에 넣어줘` as `yellow apple -> yellow box`. Repeated live
tests show that this is deterministic destination-to-pick attribute leakage,
not retained state from a previous command. The parser then accepts the result
because both output fields are structurally valid English visual queries.

## Goal

Keep an unqualified pick object unqualified. In particular, the command above
must produce `apple -> yellow box`. Attributes explicitly attached to either
target must remain with that target, while previously unseen object names must
continue to work without adding aliases.

## Non-Goals

- Do not change YOLOE, grasp generation, the task manager, Nav2, precision
  control, base/arm switching, or manipulation controllers.
- Do not add object, color, or material aliases as a correction layer.
- Do not change the existing direct JSON task interface.
- Do not add extra model calls or materially increase task-start latency.

## Selected Design

Use one constrained Gemma request with explicit source ownership. The response
schema adds `pick_source` and `place_source` alongside the existing English
`pick`, `place`, `needs_clarification`, and `reason` fields.

The prompt requires the two source fields to be copied from the original user
command before translation. It then instructs Gemma to translate each source
phrase independently and forbids copying an attribute between targets. No
concrete object example is embedded in the prompt, avoiding the example bias
seen with the earlier `orange`/`pink box` schema example.

Example internal response:

```json
{
  "pick_source": "사과",
  "place_source": "노란색 박스",
  "pick": "apple",
  "place": "yellow box",
  "needs_clarification": false,
  "reason": ""
}
```

## Validation and Data Flow

1. The console publishes the original command unchanged.
2. The existing unresolved-reference input gate runs before Gemma.
3. Gemma extracts two original-language source phrases and independently
   translates them in one request.
4. The parser verifies that both non-empty source phrases occur in the original
   command and that their selected spans do not overlap.
5. The existing English visual-query validation checks `pick` and `place`.
6. Only a valid result is published through the unchanged `/pick_place_task`
   JSON interface.
7. YOLOE grounding and all subsequent navigation and manipulation behavior
   continue unchanged.

Korean case particles may remain at the edge of a source phrase as long as the
phrase is a literal substring of the command. They are omitted from the English
visual query by the model.

## Failure Behavior

If either source phrase is missing, absent from the original command, overlaps
the other source phrase, or the English output fails existing validation, the
parser publishes a rejection status and publishes no task. It does not fall
back to aliases, reuse a previous task, or start navigation.

Direct JSON input remains compatible because source-span validation applies
only to Ollama-produced natural-language results.

## Performance

Live comparison on the current RTX 3090 Ti with Isaac Sim running measured the
selected single-request format at approximately 1.0-1.1 seconds per command,
compared with approximately 0.8 seconds for the previous schema. The model
remains fully GPU-resident. Separate translation requests were rejected because
they would add avoidable inference time and GPU work.

## Verification

Automated tests will cover:

- source phrases that are present and non-overlapping;
- missing, invented, and overlapping source phrases;
- unchanged direct JSON compatibility;
- prompt and JSON schema requirements;
- no task publication after source validation failure;
- regression coverage for existing open-vocabulary and safety behavior.

Live Gemma verification will require these exact results:

- `사과를 노란색 박스에 넣어줘` -> `apple`, `yellow box`;
- `사과를 분홍색 박스에 넣어줘` -> `apple`, `pink box`;
- `오렌지를 분홍색 박스에 넣어줘` -> `orange`, `pink box`;
- `빨간 캔을 노란색 박스에 넣어줘` -> `red can`, `yellow box`;
- `잘 익은 바나나를 나무 트레이에 올려줘` -> `ripe banana`,
  `wooden tray`.

The full existing parser, task-manager, YOLOE, and grasp regression suites must
remain green.
