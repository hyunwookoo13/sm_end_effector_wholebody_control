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
- Keep the added latency confined to task interpretation before motion starts.

## Selected Design

Use three isolated Gemma requests. The first request extracts only literal
`pick_source` and `place_source` spans from the original command. After source
validation, one request translates only the pick span and another request
translates only the place span. A translation request never receives the other
target, making cross-target attribute copying structurally impossible.

Example extraction response:

```json
{
  "pick_source": "사과",
  "place_source": "노란색 박스",
  "needs_clarification": false,
  "reason": ""
}
```

The two isolated translation responses are `{"query":"apple"}` and
`{"query":"yellow box"}`. No concrete object catalog or alias correction is
used.

## Validation and Data Flow

1. The console publishes the original command unchanged.
2. The existing unresolved-reference input gate runs before Gemma.
3. Gemma extracts only two original-language source phrases.
4. The parser verifies that both non-empty source phrases occur in the original
   command and that their selected spans do not overlap.
5. The parser sends `pick_source` alone to one translation request and
   `place_source` alone to a second translation request.
6. The existing English visual-query validation checks the two translated
   queries.
7. Only a valid result is published through the unchanged `/pick_place_task`
   JSON interface.
8. YOLOE grounding and all subsequent navigation and manipulation behavior
   continue unchanged.

Korean case particles may remain at the edge of a source phrase as long as the
phrase is a literal substring of the command. They are omitted from the English
visual query by the model.

## Failure Behavior

If either source phrase is missing, absent from the original command, overlaps
the other source phrase, a translation request fails, or the English output
fails existing validation, the parser publishes a rejection status and
publishes no task. It does not fall back to aliases, reuse a previous task, or
start navigation.

Direct JSON input remains compatible because source-span validation applies
only to Ollama-produced natural-language results.

## Performance

Live acceptance on the current RTX 3090 Ti measured the complete three-request
sequence at approximately 1.55-1.60 seconds per command. This delay occurs once,
before navigation starts. The model remains fully GPU-resident, and no model
request runs during Nav2, precision control, or arm motion.

## Verification

Automated tests will cover:

- source phrases that are present and non-overlapping;
- missing, invented, and overlapping source phrases;
- unchanged direct JSON compatibility;
- extraction and translation JSON schema requirements;
- proof that each translator receives only one source phrase;
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
