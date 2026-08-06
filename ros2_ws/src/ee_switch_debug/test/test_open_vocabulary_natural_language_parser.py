from ee_switch_debug.natural_language_task_parser import NaturalLanguageTaskParser


def make_parser_without_ros_node():
    parser = NaturalLanguageTaskParser.__new__(NaturalLanguageTaskParser)
    parser.max_query_length = 80
    parser.alias_map = {}
    parser.allowed_objects = {"apple", "yellow box"}
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
