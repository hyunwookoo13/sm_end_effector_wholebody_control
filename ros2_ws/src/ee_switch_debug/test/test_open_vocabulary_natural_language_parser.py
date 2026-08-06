import json
import threading

from std_msgs.msg import String

from ee_switch_debug.natural_language_task_parser import (
    DEFAULT_OLLAMA_MODEL,
    NaturalLanguageTaskParser,
)


def make_parser_without_ros_node():
    parser = NaturalLanguageTaskParser.__new__(NaturalLanguageTaskParser)
    parser.max_query_length = 80
    parser.alias_map = {}
    parser.allowed_objects = {"apple", "yellow box"}
    return parser


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class RecordingLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(str(message))

    def warn(self, message):
        self.warnings.append(str(message))


def make_message_parser():
    parser = make_parser_without_ros_node()
    parser.use_ollama = True
    parser.use_rule_fallback = True
    parser.dry_run = False
    parser.task_pub = RecordingPublisher()
    parser.status_pub = RecordingPublisher()
    parser._recording_logger = RecordingLogger()
    parser.get_logger = lambda: parser._recording_logger
    return parser


def test_validate_result_accepts_unregistered_visual_queries():
    result = make_parser_without_ros_node().validate_result(
        {"pick": "orange", "place": "pink box", "needs_clarification": False}
    )

    assert result == (True, "ok", "orange", "pink box")


def test_validate_result_accepts_another_unseen_object_without_aliases():
    result = make_parser_without_ros_node().validate_result(
        {"pick": "ripe banana", "place": "wooden tray", "needs_clarification": False}
    )

    assert result == (True, "ok", "ripe banana", "wooden tray")


def test_validate_result_rejects_model_clarification():
    result = make_parser_without_ros_node().validate_result(
        {"needs_clarification": True, "reason": "missing place target"}
    )

    assert result == (False, "missing place target", None, None)


def test_validate_result_rejects_control_or_topic_syntax():
    result = make_parser_without_ros_node().validate_result(
        {"pick": "orange\n/cmd_vel", "place": "pink box"}
    )

    assert result[0] is False


def test_validate_result_rejects_equal_queries():
    result = make_parser_without_ros_node().validate_result(
        {"pick": "orange", "place": "orange"}
    )

    assert result == (False, "pick and place targets are the same", None, None)


def test_validate_result_rejects_unresolved_visual_references():
    result = make_parser_without_ros_node().validate_result(
        {"pick": "it", "place": "there", "needs_clarification": False}
    )

    assert result[0] is False


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


def test_default_model_has_enough_capacity_for_multilingual_grounding():
    assert DEFAULT_OLLAMA_MODEL == "gemma3:4b"


def test_natural_language_uses_ollama_before_rules():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: {
        "pick_source": "오렌지",
        "place_source": "분홍색 박스",
        "pick": "orange",
        "place": "pink box",
        "needs_clarification": False,
    }
    parser.parse_with_rules = lambda text: (_ for _ in ()).throw(
        AssertionError("rules called before Ollama")
    )

    parser.on_natural_language_task(String(data="오렌지를 분홍색 박스에 넣어줘"))

    assert json.loads(parser.task_pub.messages[-1].data) == {
        "pick": "orange",
        "place": "pink box",
    }
    assert json.loads(parser.status_pub.messages[-1].data)["source"] == "ollama"


def test_ollama_timeout_publishes_no_task_and_no_rule_fallback():
    parser = make_message_parser()
    parser.use_rule_fallback = False
    parser.parse_with_ollama = lambda text: (_ for _ in ()).throw(TimeoutError("timeout"))

    parser.on_natural_language_task(String(data="오렌지를 분홍색 박스에 넣어줘"))

    assert parser.task_pub.messages == []
    status = json.loads(parser.status_pub.messages[-1].data)
    assert status["ok"] is False
    assert status["source"] == "ollama"
    assert status["reason"].startswith("nlp_unavailable")


def test_model_clarification_does_not_fall_back_to_rules():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: {
        "pick": "",
        "place": "",
        "needs_clarification": True,
        "reason": "unresolved pronoun",
    }
    parser.parse_with_rules = lambda text: (_ for _ in ()).throw(
        AssertionError("rules called after clarification")
    )

    parser.on_natural_language_task(String(data="옮겨줘"))

    assert parser.task_pub.messages == []
    status = json.loads(parser.status_pub.messages[-1].data)
    assert status["reason"] == "unresolved pronoun"


def test_unresolved_input_reference_is_rejected_before_ollama():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: (_ for _ in ()).throw(
        AssertionError("Ollama called for an unresolved reference")
    )

    parser.on_natural_language_task(String(data="그걸 저기에 놓아줘"))

    assert parser.task_pub.messages == []
    status = json.loads(parser.status_pub.messages[-1].data)
    assert status["ok"] is False
    assert status["reason"] == "unresolved reference in command"


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


def test_direct_json_bypasses_ollama():
    parser = make_message_parser()
    parser.parse_with_ollama = lambda text: (_ for _ in ()).throw(
        AssertionError("Ollama called for direct JSON")
    )

    parser.on_natural_language_task(
        String(data='{"pick":"orange","place":"pink box"}')
    )

    assert json.loads(parser.task_pub.messages[-1].data) == {
        "pick": "orange",
        "place": "pink box",
    }
    assert json.loads(parser.status_pub.messages[-1].data)["source"] == "direct_json"


def test_ollama_prompt_is_open_vocabulary_and_keeps_model_loaded(monkeypatch):
    parser = make_parser_without_ros_node()
    parser.ollama_url = "http://127.0.0.1:11434/api/chat"
    parser.model = DEFAULT_OLLAMA_MODEL
    parser.request_timeout_sec = 5.0
    captured = {}

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return False

        def read(self):
            return json.dumps(
                {
                    "message": {
                        "content": json.dumps(
                            {
                                "pick": "orange",
                                "place": "pink box",
                                "needs_clarification": False,
                                "reason": "",
                            }
                        )
                    }
                }
            ).encode("utf-8")

    def fake_urlopen(request, timeout):
        captured["body"] = json.loads(request.data.decode("utf-8"))
        captured["timeout"] = timeout
        return FakeResponse()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)

    result = parser.parse_with_ollama("오렌지를 분홍색 박스에 넣어줘")

    assert result["pick"] == "orange"
    assert captured["body"]["keep_alive"] == "30m"
    assert captured["body"]["options"]["num_ctx"] == 2048
    assert captured["body"]["options"]["num_predict"] == 64
    assert captured["body"]["format"]["type"] == "object"
    assert set(captured["body"]["format"]["required"]) == {
        "pick_source",
        "place_source",
        "pick",
        "place",
        "needs_clarification",
        "reason",
    }
    system_prompt = captured["body"]["messages"][0]["content"]
    assert "Allowed objects" not in system_prompt
    assert "visual noun phrase" in system_prompt
    assert '"pick":"orange"' not in system_prompt
    assert "exact character substrings" in system_prompt
    assert "Never copy" in system_prompt


def test_warm_ollama_model_loads_without_generating_text(monkeypatch):
    parser = make_parser_without_ros_node()
    parser.ollama_url = "http://127.0.0.1:11434/api/chat"
    parser.model = DEFAULT_OLLAMA_MODEL
    parser.model_warmup_timeout_sec = 30.0
    parser.model_warmup_done = threading.Event()
    parser.model_warmup_error = ""
    parser._recording_logger = RecordingLogger()
    parser.get_logger = lambda: parser._recording_logger
    captured = {}

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return False

        def read(self):
            return b'{"done":true,"response":""}'

    def fake_urlopen(request, timeout):
        captured["url"] = request.full_url
        captured["body"] = json.loads(request.data.decode("utf-8"))
        captured["timeout"] = timeout
        return FakeResponse()

    monkeypatch.setattr("urllib.request.urlopen", fake_urlopen)

    parser.warm_ollama_model()

    assert captured["url"] == "http://127.0.0.1:11434/api/generate"
    assert captured["body"] == {
        "model": DEFAULT_OLLAMA_MODEL,
        "prompt": "",
        "stream": False,
        "keep_alive": "30m",
        "options": {"num_ctx": 2048},
    }
    assert parser.model_warmup_done.is_set()
    assert parser.model_warmup_error == ""
