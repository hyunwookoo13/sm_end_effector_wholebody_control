# sm_grasping_ros2

`/sm_florence_2_vlm/roi_pointcloud`를 입력으로 받아 GPD 기반 6DoF grasp를 추론하고, RViz에 그리퍼 마커를 표시하는 ROS2 패키지입니다.

## 구성 노드

- `sm_grasping_inference`:
  - 입력: `/sm_florence_2_vlm/roi_pointcloud` (`sensor_msgs/msg/PointCloud2`)
  - 출력:
    - `/sm_grasping/grasp_candidates` (`geometry_msgs/msg/PoseArray`)
    - `/sm_grasping/grasp_scores` (`std_msgs/msg/Float32MultiArray`)
    - `/sm_grasping/grasp_best` (`geometry_msgs/msg/PoseStamped`)
    - `/sm_grasping/grasp_debug` (`std_msgs/msg/String`, JSON)
- `sm_gripper_marker`:
  - 입력: `/sm_grasping/grasp_candidates`, `/sm_grasping/grasp_best`
  - 출력: `/sm_grasping/gripper_markers` (`visualization_msgs/msg/MarkerArray`)

## GPD 연동

`./setup.sh`에서 GPD 저장소를 `third_party/gpd`로 다운로드할 수 있습니다.

```bash
cd third_party/gpd
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

현재 노드는 GPD 실연동 전 단계로, 통합 검증을 위해 중심점 기반 임시 grasp를 생성합니다.
향후 GPD 출력(검출 결과) 브리지를 붙이면 실제 GPD 추론 결과를 같은 토픽 형식으로 내보낼 수 있습니다.

## 실행

```bash
cd ~/colcon_ws
colcon build --packages-select sm_grasping_ros2
source install/setup.bash
ros2 launch sm_grasping_ros2 sm_grasping_ros2.launch.py
```

## RViz

- `MarkerArray` 디스플레이 추가
- Topic: `/sm_grasping/gripper_markers`

## 참고

- GPD: https://github.com/atenpas/gpd
