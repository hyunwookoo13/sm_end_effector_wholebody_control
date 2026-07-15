import numpy as np

from sm_grasping_ros2.gpd_wrapper import GpdWrapper
import sm_grasping_ros2.grasping_inference_node as inference_node


class LoggerStub:
    def warn(self, _message):
        pass


def test_default_heuristic_grasp_keeps_xy_at_roi_centroid():
    centroid = np.array([0.50, 0.10, 0.20], dtype=float)
    points = np.array(
        [
            centroid + [-0.06, -0.02, 0.0],
            centroid + [-0.06, 0.02, 0.0],
            centroid + [0.00, -0.02, 0.0],
            centroid + [0.00, 0.02, 0.0],
            centroid + [0.06, -0.02, 0.0],
            centroid + [0.06, 0.02, 0.0],
        ],
        dtype=float,
    )

    candidate = GpdWrapper(LoggerStub()).infer(points, max_candidates=1, lims=[])[0]

    assert np.allclose(candidate.translation[:2], centroid[:2], atol=1e-9)


def test_radial_offset_moves_a_front_target_further_toward_object():
    target = inference_node.apply_radial_xy_offset(
        np.array([1.0, 0.0, 0.3], dtype=float),
        0.04,
    )

    assert np.allclose(target, [1.04, 0.0, 0.3], atol=1e-9)


def test_radial_offset_preserves_diagonal_object_direction():
    target = inference_node.apply_radial_xy_offset(
        np.array([3.0, 4.0, 0.3], dtype=float),
        0.04,
    )

    assert np.allclose(target, [3.024, 4.032, 0.3], atol=1e-9)
