from sm_natural_language_task.natural_language_task_parser import DEFAULT_OLLAMA_MODEL
from sm_natural_language_task.natural_language_task_console import NaturalLanguageTaskConsole


def test_language_package_exports_current_nodes():
    assert DEFAULT_OLLAMA_MODEL == "gemma3:4b"
    assert NaturalLanguageTaskConsole.__name__ == "NaturalLanguageTaskConsole"
