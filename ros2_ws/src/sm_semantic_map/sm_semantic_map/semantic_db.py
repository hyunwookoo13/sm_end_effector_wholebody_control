from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

import yaml


SCHEMA = """
CREATE TABLE IF NOT EXISTS objects (
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
    observation_json TEXT NOT NULL,
    location_reference TEXT NOT NULL DEFAULT '',
    updated_at TEXT NOT NULL DEFAULT ''
);
CREATE TABLE IF NOT EXISTS aliases (
    alias TEXT PRIMARY KEY COLLATE NOCASE,
    object_id TEXT NOT NULL REFERENCES objects(object_id) ON DELETE CASCADE
);
"""


def normalize_name(value: str) -> str:
    return " ".join(str(value).strip().lower().replace("_", " ").split())


class SemanticMapDB:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.connection = sqlite3.connect(self.path)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA foreign_keys = ON")
        self.connection.executescript(SCHEMA)
        self._migrate_schema()

    def _migrate_schema(self) -> None:
        columns = {
            str(row["name"])
            for row in self.connection.execute("PRAGMA table_info(objects)")
        }
        with self.connection:
            if "location_reference" not in columns:
                self.connection.execute(
                    "ALTER TABLE objects ADD COLUMN "
                    "location_reference TEXT NOT NULL DEFAULT ''"
                )
            if "updated_at" not in columns:
                self.connection.execute(
                    "ALTER TABLE objects ADD COLUMN updated_at TEXT NOT NULL DEFAULT ''"
                )

    def close(self) -> None:
        self.connection.close()

    def load_seed(self, seed_path: str | Path) -> int:
        payload = yaml.safe_load(Path(seed_path).read_text())
        frame_id = str(payload["frame_id"])
        objects = payload.get("objects", [])
        with self.connection:
            for item in objects:
                object_id = str(item["object_id"])
                object_pose = item["object_pose"]
                approach_pose = item["approach_pose"]
                self.connection.execute(
                    """
                    INSERT INTO objects (
                      object_id, canonical_name, semantic_class, capabilities_json,
                      zone, source_sensor, status, frame_id,
                      object_x, object_y, object_z,
                      approach_x, approach_y, approach_yaw,
                      approach_clearance_m, observation_json,
                      location_reference, updated_at
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    ON CONFLICT(object_id) DO UPDATE SET
                      canonical_name=excluded.canonical_name,
                      semantic_class=excluded.semantic_class,
                      capabilities_json=excluded.capabilities_json,
                      zone=excluded.zone,
                      source_sensor=excluded.source_sensor,
                      status=excluded.status,
                      frame_id=excluded.frame_id,
                      object_x=excluded.object_x,
                      object_y=excluded.object_y,
                      object_z=excluded.object_z,
                      approach_x=excluded.approach_x,
                      approach_y=excluded.approach_y,
                      approach_yaw=excluded.approach_yaw,
                      approach_clearance_m=excluded.approach_clearance_m,
                      observation_json=excluded.observation_json,
                      location_reference=excluded.location_reference,
                      updated_at=excluded.updated_at
                    """,
                    (
                        object_id,
                        item["canonical_name"],
                        item["semantic_class"],
                        json.dumps(item["capabilities"], ensure_ascii=False),
                        item["zone"],
                        item["source_sensor"],
                        item["status"],
                        frame_id,
                        object_pose["x"], object_pose["y"], object_pose["z"],
                        approach_pose["x"], approach_pose["y"], approach_pose["yaw"],
                        item["approach_clearance_m"],
                        json.dumps(item.get("observation", {}), ensure_ascii=False),
                        str(item.get("location_reference", "")),
                        str(item.get("updated_at", "")),
                    ),
                )
                self.connection.execute("DELETE FROM aliases WHERE object_id = ?", (object_id,))
                aliases = set(item.get("aliases", [])) | {
                    object_id,
                    item["canonical_name"],
                }
                for alias in aliases:
                    self.connection.execute(
                        "INSERT OR REPLACE INTO aliases(alias, object_id) VALUES (?, ?)",
                        (normalize_name(alias), object_id),
                    )
        return len(objects)

    def find(self, name: str) -> dict[str, object] | None:
        row = self.connection.execute(
            """
            SELECT objects.* FROM objects
            JOIN aliases ON aliases.object_id = objects.object_id
            WHERE aliases.alias = ? COLLATE NOCASE
            """,
            (normalize_name(name),),
        ).fetchone()
        if row is None:
            return None
        result = dict(row)
        result["capabilities"] = json.loads(result.pop("capabilities_json"))
        result["observation"] = json.loads(result.pop("observation_json"))
        return result

    def list_objects(self) -> list[dict[str, object]]:
        return [self.find(row["object_id"]) for row in self.connection.execute(
            "SELECT object_id FROM objects ORDER BY object_id"
        )]

    def count_objects(self) -> int:
        row = self.connection.execute("SELECT COUNT(*) AS count FROM objects").fetchone()
        return int(row["count"])


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", required=True)
    subparsers = parser.add_subparsers(dest="command", required=True)
    initialize = subparsers.add_parser("init")
    initialize.add_argument("--seed", required=True)
    find = subparsers.add_parser("find")
    find.add_argument("name")
    subparsers.add_parser("list")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> None:
    args = parse_args(argv)
    database = SemanticMapDB(args.db)
    try:
        if args.command == "init":
            print(json.dumps({"loaded": database.load_seed(args.seed), "db": args.db}))
        elif args.command == "find":
            print(json.dumps(database.find(args.name), ensure_ascii=False, indent=2))
        else:
            print(json.dumps(database.list_objects(), ensure_ascii=False, indent=2))
    finally:
        database.close()
