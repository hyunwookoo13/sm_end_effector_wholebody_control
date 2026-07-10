import json
import re
import urllib.error
import urllib.request
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        self.declare_parameter("model", "gemma3:1b")
        self.declare_parameter("request_timeout_sec", 5.0)
        self.declare_parameter(
            "allowed_objects",
            [
                "can",
                "blue can",
                "red can",
                "box",
                "blue box",
                "red box",
                "yellow box",
                "pink box",
                "dish",
                "apple",
                "bottle",
                "mug",
                "green cup",
            ],
        )
        self.declare_parameter("alias_map_json", "{}")
        self.declare_parameter("use_ollama", True)
        self.declare_parameter("use_rule_fallback", True)
        self.declare_parameter("dry_run", False)

        self.natural_language_topic = str(self.get_parameter("natural_language_topic").value)
        self.task_topic = str(self.get_parameter("task_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.ollama_url = str(self.get_parameter("ollama_url").value)
        self.model = str(self.get_parameter("model").value)
        self.request_timeout_sec = float(self.get_parameter("request_timeout_sec").value)
        self.allowed_objects = {
            str(name).strip().lower()
            for name in self.get_parameter("allowed_objects").value
            if str(name).strip()
        }
        self.alias_map = self._load_aliases(str(self.get_parameter("alias_map_json").value))
        self.use_ollama = bool(self.get_parameter("use_ollama").value)
        self.use_rule_fallback = bool(self.get_parameter("use_rule_fallback").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.task_pub = self.create_publisher(String, self.task_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(String, self.natural_language_topic, self.on_natural_language_task, 10)

        self.get_logger().info(
            "Natural language task parser: "
            f"{self.natural_language_topic} -> {self.task_topic}, "
            f"model={self.model}, allowed={sorted(self.allowed_objects)}"
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

        if parsed is None and self.use_rule_fallback:
            rule_parsed = self.parse_with_rules(text)
            rule_valid, _, _, _ = self.validate_result(rule_parsed)
            if rule_valid:
                parsed = rule_parsed
                source = "rules"

        if parsed is None and self.use_ollama:
            try:
                parsed = self.parse_with_ollama(text)
                source = "ollama"
            except Exception as exc:
                self.get_logger().warn(f"Ollama parse failed: {exc}")

        valid, reason, pick, place = self.validate_result(parsed)
        if not valid and self.use_rule_fallback and source != "rules":
            fallback = self.parse_with_rules(text)
            fallback_valid, fallback_reason, fallback_pick, fallback_place = self.validate_result(fallback)
            if fallback_valid:
                parsed = fallback
                source = f"{source}+rules" if source else "rules"
                valid = fallback_valid
                reason = fallback_reason
                pick = fallback_pick
                place = fallback_place
            elif parsed is None:
                reason = fallback_reason
        if not valid:
            self.publish_status(False, reason, source=source, text=text)
            self.get_logger().warn(f"Natural language task rejected: {reason}; text={text!r}")
            return

        task = {"pick": pick, "place": place}
        if not self.dry_run:
            self.task_pub.publish(String(data=json.dumps(task, ensure_ascii=False)))
        self.publish_status(True, "published", source=source, text=text, **task)
        self.get_logger().warn(f"Natural language task -> pick={pick}, place={place}, source={source}")

    def parse_direct_json(self, text: str) -> dict[str, Any] | None:
        raw = text.strip()
        if not raw.startswith("{"):
            return None
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError:
            return None
        return payload if isinstance(payload, dict) else None

    def parse_with_ollama(self, text: str) -> dict[str, Any] | None:
        system_prompt = (
            "You convert Korean or English robot commands into JSON. "
            "Return only JSON. "
            f"Allowed objects: {', '.join(sorted(self.allowed_objects))}. "
            "Use canonical English object names from the allowed list. "
            "If the command is ambiguous, set needs_clarification true. "
            "Schema: {\"pick\":\"can\",\"place\":\"box\",\"needs_clarification\":false}"
        )
        request_body = {
            "model": self.model,
            "stream": False,
            "format": "json",
            "options": {
                "temperature": 0.0,
                "num_predict": 128,
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
            payload = json.loads(raw[start : end + 1])
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
            if canonical not in self.allowed_objects:
                continue
            for match in re.finditer(re.escape(alias), text):
                matches.append((match.start(), match.end(), canonical))

        matches.sort(key=lambda item: (item[0], -(item[1] - item[0])))
        non_overlapping: list[tuple[int, int, str]] = []
        for candidate in matches:
            start, end, _ = candidate
            if any(start < kept_end and end > kept_start for kept_start, kept_end, _ in non_overlapping):
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
            if canonical not in self.allowed_objects:
                continue
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

        pick = self.normalize_object(payload.get("pick", payload.get("pick_target", "")))
        place = self.normalize_object(payload.get("place", payload.get("place_target", "")))
        if not pick or pick not in self.allowed_objects:
            return False, f"invalid pick target: {payload.get('pick', payload.get('pick_target', ''))}", None, None
        if not place or place not in self.allowed_objects:
            return False, f"invalid place target: {payload.get('place', payload.get('place_target', ''))}", None, None
        if pick == place:
            return False, "pick and place targets are the same", None, None
        return True, "ok", pick, place

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
