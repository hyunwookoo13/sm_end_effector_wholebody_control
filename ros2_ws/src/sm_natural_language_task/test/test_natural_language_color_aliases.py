from sm_natural_language_task.natural_language_task_parser import NaturalLanguageTaskParser


def test_default_aliases_support_green_cup_and_pink_box():
    aliases = NaturalLanguageTaskParser._load_default_aliases()

    assert aliases["green cup"] == "green cup"
    assert aliases["초록 컵"] == "green cup"
    assert aliases["pink box"] == "pink box"
    assert aliases["분홍 박스"] == "pink box"
