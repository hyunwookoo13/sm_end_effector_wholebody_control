from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_find_object_service_exposes_query_metadata_and_poses():
    service = (PACKAGE_ROOT / "srv" / "FindObject.srv").read_text()
    for field in (
        "string query",
        "string required_capability",
        "bool found",
        "string object_id",
        "string[] capabilities",
        "string status",
        "geometry_msgs/PoseStamped object_pose",
        "geometry_msgs/PoseStamped approach_pose",
    ):
        assert field in service
