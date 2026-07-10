import threading

from sensor_msgs.msg import PointCloud2

from sm_grasping_ros2.grasping_inference_node import AnyGraspInferenceNode


def test_take_latest_cloud_consumes_cloud_once():
    node = AnyGraspInferenceNode.__new__(AnyGraspInferenceNode)
    node._lock = threading.Lock()
    cloud = PointCloud2()
    node._latest_cloud = cloud

    assert node._take_latest_cloud() is cloud
    assert node._latest_cloud is None
    assert node._take_latest_cloud() is None
