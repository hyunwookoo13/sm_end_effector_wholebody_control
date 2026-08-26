from math import cos, sin

from pytest import approx

from sm_semantic_map.geometry import Point3, summarize_points, world_to_map


def test_world_to_map_inverts_map_pose_in_world():
    map_x = 2.0
    map_y = -3.0
    yaw = 0.2
    expected = Point3(4.0, -1.5, 0.8)
    world = Point3(
        map_x + cos(yaw) * expected.x - sin(yaw) * expected.y,
        map_y + sin(yaw) * expected.x + cos(yaw) * expected.y,
        expected.z,
    )
    actual = world_to_map(world, map_x, map_y, yaw)
    assert actual.x == approx(expected.x)
    assert actual.y == approx(expected.y)
    assert actual.z == approx(expected.z)


def test_summarize_points_uses_median_and_reports_spread():
    summary = summarize_points(
        [Point3(1.0, 2.0, 3.0), Point3(1.2, 2.2, 3.2), Point3(9.0, 9.0, 9.0)]
    )
    assert summary["samples"] == 3
    assert summary["median"] == approx([1.2, 2.2, 3.2])
    assert all(value > 0.0 for value in summary["stddev"])
