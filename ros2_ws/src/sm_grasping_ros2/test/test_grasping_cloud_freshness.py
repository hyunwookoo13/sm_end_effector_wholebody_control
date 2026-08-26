import threading

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String

from sm_grasping_ros2.grasping_inference_node import (
    AnyGraspInferenceNode,
    EmaFilterState,
)


def test_take_latest_cloud_consumes_cloud_once():
    node = AnyGraspInferenceNode.__new__(AnyGraspInferenceNode)
    node._lock = threading.Lock()
    cloud = PointCloud2()
    node._latest_cloud = cloud

    assert node._take_latest_cloud() is cloud
    assert node._latest_cloud is None
    assert node._take_latest_cloud() is None


class _Clock:
    class _Now:
        nanoseconds = 123

    def now(self):
        return self._Now()


class _Wrapper:
    def __init__(self):
        self.reset_count = 0

    def reset_tracking_state(self):
        self.reset_count += 1


class _Logger:
    def warn(self, _message):
        pass


def test_empty_target_clears_previous_cloud_and_tracking_state():
    node = AnyGraspInferenceNode.__new__(AnyGraspInferenceNode)
    node._lock = threading.Lock()
    node._latest_cloud = PointCloud2()
    node._target_key = ("red can",)
    node._ema_state = EmaFilterState()
    node._wrapper = _Wrapper()
    node.p_target_change_settle_sec = 0.4
    node.get_clock = lambda: _Clock()
    node.get_logger = lambda: _Logger()

    node._target_objects_callback(
        String(data='{"target_objects": [], "roi_target_objects": []}')
    )

    assert node._target_key == ()
    assert node._latest_cloud is None
    assert node._accept_cloud_after_ns == 400_000_123
    assert node._wrapper.reset_count == 1
