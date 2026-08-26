from pathlib import Path

from sm_semantic_map.semantic_db import SemanticMapDB


SEED = Path(__file__).resolve().parents[1] / "config" / "warehouse_objects.yaml"


def test_load_and_find_by_english_and_korean_alias(tmp_path):
    database = SemanticMapDB(tmp_path / "objects.sqlite3")
    try:
        assert database.load_seed(SEED) == 6
        assert database.find("blue_can")["object_id"] == "blue_can"
        assert database.find("파란 캔")["object_id"] == "blue_can"
        assert database.find("노란 상자")["object_id"] == "yellow_box"
    finally:
        database.close()


def test_seed_reload_is_idempotent(tmp_path):
    database = SemanticMapDB(tmp_path / "objects.sqlite3")
    try:
        database.load_seed(SEED)
        database.load_seed(SEED)
        assert len(database.list_objects()) == 6
    finally:
        database.close()


def test_existing_database_schema_is_migrated(tmp_path):
    path = tmp_path / "legacy.sqlite3"
    import sqlite3

    connection = sqlite3.connect(path)
    connection.execute(
        """
        CREATE TABLE objects (
          object_id TEXT PRIMARY KEY,
          canonical_name TEXT NOT NULL,
          semantic_class TEXT NOT NULL,
          capabilities_json TEXT NOT NULL,
          zone TEXT NOT NULL,
          source_sensor TEXT NOT NULL,
          status TEXT NOT NULL,
          frame_id TEXT NOT NULL,
          object_x REAL NOT NULL,
          object_y REAL NOT NULL,
          object_z REAL NOT NULL,
          approach_x REAL NOT NULL,
          approach_y REAL NOT NULL,
          approach_yaw REAL NOT NULL,
          approach_clearance_m REAL NOT NULL,
          observation_json TEXT NOT NULL
        )
        """
    )
    connection.commit()
    connection.close()

    migrated = SemanticMapDB(path)
    try:
        columns = {
            row["name"]
            for row in migrated.connection.execute("PRAGMA table_info(objects)")
        }
        assert {"location_reference", "updated_at"} <= columns
    finally:
        migrated.close()
