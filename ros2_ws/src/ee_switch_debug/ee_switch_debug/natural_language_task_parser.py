import json
import re
import threading
import urllib.error
import urllib.request
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


DEFAULT_OLLAMA_MODEL = "gemma3:4b"

UNRESOLVED_VISUAL_REFERENCES = {
    "it",
    "this",
    "that",
    "this object",
    "that object",
    "there",
    "here",
    "this place",
    "that place",
}


DEFAULT_ALIASES = {
    "can": "can",
    "cans": "can",
    "soup can": "can",
    "tomato soup can": "can",
    "blue can": "blue can",
    "blue tin": "blue can",
    "파란 캔": "blue can",
    "파란색 캔": "blue can",
    "파랑 캔": "blue can",
    "파란 통조림": "blue can",
    "파란색 통조림": "blue can",
    "red can": "red can",
    "red tin": "red can",
    "빨간 캔": "red can",
    "빨간색 캔": "red can",
    "빨강 캔": "red can",
    "빨간 통조림": "red can",
    "빨간색 통조림": "red can",
    "blue box": "blue box",
    "blue bin": "blue box",
    "blue container": "blue box",
    "파란 박스": "blue box",
    "파란색 박스": "blue box",
    "파랑 박스": "blue box",
    "파란 상자": "blue box",
    "파란색 상자": "blue box",
    "red box": "red box",
    "red bin": "red box",
    "red container": "red box",
    "빨간 박스": "red box",
    "빨간색 박스": "red box",
    "빨강 박스": "red box",
    "빨간 상자": "red box",
    "빨간색 상자": "red box",
    "yellow box": "yellow box",
    "yellow bin": "yellow box",
    "yellow container": "yellow box",
    "yellow tray": "yellow box",
    "노란 박스": "yellow box",
    "노란색 박스": "yellow box",
    "노랑 박스": "yellow box",
    "노란 상자": "yellow box",
    "노란색 상자": "yellow box",
    "노란 트레이": "yellow box",
    "노란색 트레이": "yellow box",
    "노란 쟁반": "yellow box",
    "노란색 쟁반": "yellow box",
    "pink box": "pink box",
    "pink bin": "pink box",
    "pink container": "pink box",
    "pink tray": "pink box",
    "분홍 박스": "pink box",
    "분홍색 박스": "pink box",
    "핑크 박스": "pink box",
    "분홍 상자": "pink box",
    "분홍색 상자": "pink box",
    "핑크 상자": "pink box",
    "분홍 트레이": "pink box",
    "분홍색 트레이": "pink box",
    "핑크 트레이": "pink box",
    "분홍 쟁반": "pink box",
    "분홍색 쟁반": "pink box",
    "캔": "can",
    "통조림": "can",
    "토마토캔": "can",
    "토마토 캔": "can",
    "box": "box",
    "bin": "box",
    "container": "box",
    "상자": "box",
    "박스": "box",
    "통": "box",
    "dish": "dish",
    "plate": "dish",
    "tray": "dish",
    "접시": "dish",
    "쟁반": "dish",
    "apple": "apple",
    "사과": "apple",
    "bottle": "bottle",
    "병": "bottle",
    "mug": "mug",
    "cup": "mug",
    "머그": "mug",
    "컵": "mug",
    "green cup": "green cup",
    "green mug": "green cup",
    "초록 컵": "green cup",
    "초록색 컵": "green cup",
    "녹색 컵": "green cup",
    "초록 머그": "green cup",
    "초록색 머그": "green cup",
    "녹색 머그": "green cup",
}


class NaturalLanguageTaskParser(Node):
    """Convert natural-language pick/place commands into /pick_place_task JSON."""

    def __init__(self) -> None:
        super().__init__("natural_language_task_parser")

        self.declare_parameter("natural_language_topic", "/natural_language_task")
        self.declare_parameter("task_topic", "/pick_place_task")
        self.declare_parameter("status_topic", "/natural_language_task_status")
        self.declare_parameter("ollama_url", "http://127.0.0.1:11434/api/chat")
        self.declare_parameter("model", DEFAULT_OLLAMA_MODEL)
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter("model_warmup_timeout_sec", 30.0)
        self.declare_parameter("warm_model_on_startup", True)
        self.declare_parameter("max_query_length", 80)
        self.declare_parameter("alias_map_json", "{}")
        self.declare_parameter("use_ollama", True)
        self.declare_parameter("use_rule_fallback", False)
        self.declare_parameter("dry_run", False)

        self.natural_language_topic = str(self.get_parameter("natural_language_topic").value)
        self.task_topic = str(self.get_parameter("task_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.ollama_url = str(self.get_parameter("ollama_url").value)
        self.model = str(self.get_parameter("model").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.model_warmup_timeout_sec = float(
            self.get_parameter("model_warmup_timeout_sec").value
        )
        self.warm_model_on_startup = bool(
            self.get_parameter("warm_model_on_startup").value
        )
        self.max_query_length = int(self.get_parameter("max_query_length").value)
        self.alias_map = self._load_aliases(str(self.get_parameter("alias_map_json").value))
        self.use_ollama = bool(self.get_parameter("use_ollama").value)
        self.use_rule_fallback = bool(self.get_parameter("use_rule_fallback").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.task_pub = self.create_publisher(String, self.task_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(
            String,
            self.natural_language_topic,
            self.on_natural_language_task,
            10,
        )

        self.model_warmup_done = threading.Event()
        self.model_warmup_error = ""
        if self.use_ollama and self.warm_model_on_startup:
            threading.Thread(target=self.warm_ollama_model, daemon=True).start()
        else:
            self.model_warmup_done.set()

        self.get_logger().info(
            "Natural language task parser: "
            f"{self.natural_language_topic} -> {self.task_topic}, "
            f"model={self.model}, open_vocabulary=true, rule_compat={self.use_rule_fallback}"
        )

    def _load_aliases(self, alias_map_json: str) -> dict[str, str]:
        aliases = self._load_default_aliases()
        try:
            extra = json.loads(alias_map_json) if alias_map_json.strip() else {}
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid alias_map_json; using defaults: {exc}")
            extra = {}
        if isinstance(extra, dict):
            for alias, canonical in extra.items():
                aliases[str(alias).strip().lower()] = str(canonical).strip().lower()
        return aliases

    @staticmethod
    def _load_default_aliases() -> dict[str, str]:
        return dict(DEFAULT_ALIASES)

    def on_natural_language_task(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            self.publish_status(False, "empty command")
            return

        parsed: dict[str, Any] | None = self.parse_direct_json(text)
        source = "direct_json" if parsed is not None else ""

        if parsed is None and self.contains_unresolved_reference(text):
            self.publish_status(
                False,
                "unresolved reference in command",
                source="input_safety",
                text=text,
            )
            self.get_logger().warn(
                f"Natural language task rejected: unresolved reference; text={text!r}"
            )
            return

        if parsed is None and self.use_ollama:
            source = "ollama"
            try:
                parsed = self.parse_with_ollama(text)
            except Exception as exc:
                self.get_logger().warn(f"Ollama parse failed: {exc}")
                self.publish_status(
                    False,
                    f"nlp_unavailable: {exc}",
                    source=source,
                    text=text,
                )
                return
        elif parsed is None and self.use_rule_fallback:
            parsed = self.parse_with_rules(text)
            source = "rules"

        valid, reason, pick, place = self.validate_result(parsed)
        if not valid:
            self.publish_status(False, reason, source=source, text=text)
            self.get_logger().warn(f"Natural language task rejected: {reason}; text={text!r}")
            return

        task = {"pick": pick, "place": place}
        if not self.dry_run:
            self.task_pub.publish(String(data=json.dumps(task, ensure_ascii=False)))
        self.publish_status(True, "published", source=source, text=text, **task)
        self.get_logger().warn(
            f"Natural language task -> pick={pick}, place={place}, source={source}"
        )

    def parse_direct_json(self, text: str) -> dict[str, Any] | None:
        raw = text.strip()
        if not raw.startswith("{"):
            return None
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return payload if isinstance(payload, dict) else None

    @staticmethod
    def contains_unresolved_reference(text: str) -> bool:
        normalized = text.strip().lower()
        korean_references = r"(그걸|이걸|저걸|그것|이것|저것|거기|여기|저기|그곳|이곳|저곳)"
        english_references = r"\b(it|there|here|this one|that one|these|those)\b"
        return bool(
            re.search(korean_references, normalized)
            or re.search(english_references, normalized)
        )

    def parse_with_ollama(self, text: str) -> dict[str, Any] | None:
        warmup_done = getattr(self, "model_warmup_done", None)
        if warmup_done is not None and not warmup_done.wait(self.model_warmup_timeout_sec):
            raise TimeoutError("model warmup did not finish")

        system_prompt = (
            "Extract the complete object-to-pick phrase and destination phrase from one "
            "Korean or English robot pick-and-place command. Translate each phrase into "
            "a concise lowercase English visual noun phrase. Preserve every stated visual "
            "attribute, including color, material, state, and object class; never replace "
            "or invent an object or attribute. A missing color or material is not ambiguous. "
            "Set needs_clarification true and leave pick and place empty only when the pick "
            "object or destination itself is absent, a reference such as it/there is "
            "unresolved, the request is not pick-and-place, or it contains multiple tasks. "
            "Set reason to an empty string for a valid command. Return JSON only."
        )
        response_schema = {
            "type": "object",
            "properties": {
                "pick": {"type": "string"},
                "place": {"type": "string"},
                "needs_clarification": {"type": "boolean"},
                "reason": {"type": "string"},
            },
            "required": ["pick", "place", "needs_clarification", "reason"],
        }
        request_body = {
            "model": self.model,
            "stream": False,
            "format": response_schema,
            "keep_alive": "30m",
            "options": {
                "temperature": 0.0,
                "num_predict": 64,
                "num_ctx": 2048,
            },
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": text},
            ],
        }
        request = urllib.request.Request(
            self.ollama_url,
            data=json.dumps(request_body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.request_timeout_sec) as response:
                payload = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            body = exc.read().decode("utf-8", errors="replace")
            raise RuntimeError(f"HTTP {exc.code}: {body}") from exc

        content = ""
        if isinstance(payload, dict):
            message = payload.get("message")
            if isinstance(message, dict):
                content = str(message.get("content", ""))
            else:
                content = str(payload.get("response", ""))
        return self.extract_json_object(content)

    def warm_ollama_model(self) -> None:
        generate_url = self.ollama_url.rsplit("/", 1)[0] + "/generate"
        request_body = {
            "model": self.model,
            "prompt": "",
            "stream": False,
            "keep_alive": "30m",
            "options": {"num_ctx": 2048},
        }
        request = urllib.request.Request(
            generate_url,
            data=json.dumps(request_body).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request,
                timeout=self.model_warmup_timeout_sec,
            ) as response:
                response.read()
            self.get_logger().info(f"Ollama model warmed: {self.model}")
        except Exception as exc:
            self.model_warmup_error = str(exc)
            self.get_logger().warn(f"Ollama model warmup failed: {exc}")
        finally:
            self.model_warmup_done.set()

    def extract_json_object(self, text: str) -> dict[str, Any] | None:
        raw = text.strip()
        try:
            payload = json.loads(raw)
            return payload if isinstance(payload, dict) else None
        except json.JSONDecodeError:
            pass
        start = raw.find("{")
        end = raw.rfind("}")
        if start < 0 or end <= start:
            return None
        try:
            payload = json.loads(raw[start:end + 1])
        except json.JSONDecodeError:
            return None
        return payload if isinstance(payload, dict) else None

    def parse_with_rules(self, text: str) -> dict[str, Any] | None:
        normalized = text.lower()
        mentions = self.find_object_mentions(normalized)
        if len(mentions) < 2:
            return {"needs_clarification": True, "reason": "found fewer than two objects"}

        place = self.find_korean_place_target(normalized, mentions)
        if place is not None:
            pick = next((obj for _, obj in mentions if obj != place), None)
            return {"pick": pick, "place": place, "needs_clarification": pick is None}

        ordered_objects = []
        for _, obj in mentions:
            if obj not in ordered_objects:
                ordered_objects.append(obj)
        if len(ordered_objects) < 2:
            return {"needs_clarification": True, "reason": "found fewer than two distinct objects"}
        return {
            "pick": ordered_objects[0],
            "place": ordered_objects[1],
            "needs_clarification": False,
        }

    def find_object_mentions(self, text: str) -> list[tuple[int, str]]:
        matches: list[tuple[int, int, str]] = []
        aliases = sorted(self.alias_map.items(), key=lambda item: len(item[0]), reverse=True)
        for alias, canonical in aliases:
            for match in re.finditer(re.escape(alias), text):
                matches.append((match.start(), match.end(), canonical))

        matches.sort(key=lambda item: (item[0], -(item[1] - item[0])))
        non_overlapping: list[tuple[int, int, str]] = []
        for candidate in matches:
            start, end, _ = candidate
            overlaps = any(
                start < kept_end and end > kept_start
                for kept_start, kept_end, _ in non_overlapping
            )
            if overlaps:
                continue
            non_overlapping.append(candidate)

        return [(start, canonical) for start, _, canonical in sorted(non_overlapping)]

    def find_korean_place_target(
        self,
        text: str,
        mentions: list[tuple[int, str]],
    ) -> str | None:
        if not any(token in text for token in ("넣", "담", "놓", "올려")):
            return None
        aliases = sorted(self.alias_map.items(), key=lambda item: len(item[0]), reverse=True)
        for alias, canonical in aliases:
            pattern = re.escape(alias) + r"\s*(안에|속에|위에|에|으로|로)"
            if re.search(pattern, text):
                return canonical
        return None

    def validate_result(
        self,
        payload: dict[str, Any] | None,
    ) -> tuple[bool, str, str | None, str | None]:
        if not isinstance(payload, dict):
            return False, "parser returned no JSON object", None, None
        if bool(payload.get("needs_clarification", False)):
            return False, str(payload.get("reason", "needs clarification")), None, None

        pick_raw = payload.get("pick", payload.get("pick_target", ""))
        place_raw = payload.get("place", payload.get("place_target", ""))
        pick = self.normalize_visual_query(pick_raw)
        place = self.normalize_visual_query(place_raw)
        if not pick:
            return False, f"invalid pick query: {pick_raw}", None, None
        if not place:
            return False, f"invalid place query: {place_raw}", None, None
        if pick == place:
            return False, "pick and place targets are the same", None, None
        return True, "ok", pick, place

    def normalize_visual_query(self, value: Any) -> str:
        query = re.sub(r"\s+", " ", str(value).strip().lower())
        if not query or len(query) > self.max_query_length:
            return ""
        if not re.fullmatch(r"[a-z0-9][a-z0-9 ._'\-]*", query):
            return ""
        if query in UNRESOLVED_VISUAL_REFERENCES:
            return ""
        return query if re.search(r"[a-z]", query) else ""

    def normalize_object(self, value: Any) -> str:
        raw = str(value).strip().lower()
        raw = re.sub(r"^[\"'`]+|[\"'`.,!?]+$", "", raw)
        return self.alias_map.get(raw, raw)

    def publish_status(self, ok: bool, reason: str, **kwargs: Any) -> None:
        payload = {"ok": ok, "reason": reason}
        payload.update(kwargs)
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NaturalLanguageTaskParser()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
