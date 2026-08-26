.PHONY: setup test validate week meeting notion snapshot

setup:
	python3 -m pip install -r requirements.txt

test:
	python3 -m unittest discover -s tests -v

validate:
	python3 scripts/validate_research_os.py

week:
	python3 scripts/new_week.py

meeting:
	python3 scripts/build_meeting_packet.py

notion:
	python3 scripts/build_notion_report.py

snapshot:
	python3 scripts/snapshot_project.py
