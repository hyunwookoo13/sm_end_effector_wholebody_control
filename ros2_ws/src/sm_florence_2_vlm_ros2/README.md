# sm_florence_2_vlm_ros2 (ROS2 Jazzy)

## 1. 패키지 개요
`sm_florence_2_vlm_ros2`는 Intel RealSense2 RGB-D 토픽을 구독하고 Florence-2 Vision-Language Model로 지정 객체를 검출한 뒤, 2D bbox(또는 depth 기반 segmentation ROI), camera optical frame 3D 좌표, `base_link` 3D 좌표, ROI PointCloud2, RViz2 Marker, Debug Image를 publish하는 ROS2 Python 패키지입니다.

## 2. RealSense2 실행 전제
이 패키지는 RealSense2 드라이버를 실행하지 않습니다. 별도 런처에서 RGB, aligned depth, CameraInfo, PointCloud2, `base_link -> camera_link` static TF, RealSense 내부 camera TF가 이미 publish되고 있어야 합니다.

## 3. RealSense2 토픽 확인
```bash
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/camera/depth/color/points
```

## 4. TF Tree 확인
```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

## 5. `ordered_pc=true` 권장 이유
ROI PointCloud2 추출은 `roi_geometry_mode`에 따라 bbox 또는 depth segmentation mask를 사용합니다. RealSense2 런처에서 `pointcloud.ordered_pc:=true`를 사용하면 organized cloud가 유지되어 ROI 픽셀-포인트 인덱스 매칭이 정확합니다. unordered cloud에서는 픽셀 ROI와 포인트 인덱스 대응이 보장되지 않습니다.

## 6. Python 의존성 설치
```bash
python3 -m pip install -r requirements.txt --break-system-packages
```

Jetson Thor에서는 PyTorch/Transformers 버전이 JetPack CUDA 버전과 맞아야 합니다. NVIDIA에서 제공하는 PyTorch wheel 사용을 권장합니다.

`ModuleNotFoundError: No module named 'torch'`가 발생하면 현재 ROS2 노드를 실행하는 Python 환경에 PyTorch가 설치되지 않은 상태입니다. Jetson Thor에서는 먼저 JetPack 버전에 맞는 NVIDIA PyTorch wheel을 설치한 뒤 나머지 Python 패키지를 설치하세요.

```bash
python3 -m pip install --upgrade pip wheel setuptools --break-system-packages
python3 -m pip install --break-system-packages \
  torch==2.11.0 \
  --index-url https://pypi.jetson-ai-lab.io/sbsa/cu130
python3 -m pip install --break-system-packages \
  transformers==4.49.0 numpy==1.26.4 opencv-python<4.12 Pillow einops timm torchvision
```

`transformers 5.x`에서는 Florence-2 remote code와 호환성 문제가 발생할 수 있으므로 `transformers==4.49.0` 고정을 권장합니다.

공식 설치 문서: https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html

Florence-2 모델은 기본적으로 아래 폴더에 다운로드/캐시됩니다.

```bash
/home/kiro/colcon_ws/src/sm_florence_2_vlm_ros2/models
```

## 7. ROS 의존성 설치
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 8. 빌드 방법
```bash
cd ~/colcon_ws
colcon build --packages-select sm_florence_2_vlm_ros2
source install/setup.bash
```

## 9. 실행 방법
```bash
ros2 launch sm_florence_2_vlm_ros2 sm_florence_2_vlm_ros2.launch.py
```

커스텀 YAML:
```bash
ros2 launch sm_florence_2_vlm_ros2 sm_florence_2_vlm_ros2.launch.py \
  config_file:=/home/kiro/my_config/sm_florence_2_vlm_ros2.yaml
```

## 10. 파라미터 설명
| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `rgb_topic` | `/camera/camera/color/image_raw` | RGB 입력 |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | RGB 정렬 depth 입력 |
| `camera_info_topic` | `/camera/camera/color/camera_info` | color intrinsic |
| `pointcloud_topic` | `/camera/camera/depth/color/points` | RealSense PointCloud2 |
| `target_objects` | `person, chair, box` | Florence-2 검출 대상 |
| `target_objects_topic` | `/sm_florence_2_vlm/target_objects` | 런타임 객체 목록 입력 토픽 (`std_msgs/String`) |
| `use_target_objects_topic` | `true` | 토픽 기반 객체 목록 갱신 사용 여부 |
| `model_id` | `microsoft/Florence-2-base` | Hugging Face 모델 ID |
| `model_cache_dir` | `/home/kiro/colcon_ws/src/sm_florence_2_vlm_ros2/models` | 모델 다운로드/캐시 폴더 |
| `device` | `auto` | `auto`, `cuda`, `cpu` |
| `confidence_threshold` | `0.3` | bbox confidence 하한 |
| `inference_rate_hz` | `3.0` | timer 기반 추론 주기 |
| `inference_input_width` | `640` | 추론용 입력 가로 해상도(0 또는 원본 이상이면 원본 사용) |
| `num_beams` | `1` | 생성 빔 수(1이 가장 빠름) |
| `max_new_tokens` | `64` | 생성 최대 토큰 수(작을수록 빠름) |
| `enable_perf_log` | `true` | 단계별 성능 로그 출력 여부 |
| `perf_log_interval_sec` | `2.0` | 성능 로그 출력 간격(초) |
| `roi_pointcloud_source` | `depth` | ROI 포인트클라우드 생성 소스 (`depth` 또는 `pointcloud`) |
| `roi_pointcloud_require_organized` | `true` | `true`이면 `ordered_pc=false`(unorganized cloud)에서 ROI 포인트클라우드 추출을 생략해 지연을 줄임 |
| `roi_geometry_mode` | `bbox` | ROI 형상 추출 모드 (`bbox` 또는 `depth_segmentation`) |
| `roi_geometry_fallback_to_bbox` | `true` | segmentation mask 생성 실패 시 bbox ROI로 폴백 |
| `roi_depth_segmentation_min_area_px` | `10` | depth segmentation 최소 픽셀 면적 |
| `roi_depth_segmentation_kernel_px` | `1` | depth segmentation morphology kernel 크기(홀수 권장) |
| `depth_scale` | `0.001` | `uint16` depth meter 변환 스케일 |
| `camera_frame` | `camera_color_optical_frame` | camera 좌표계 |
| `target_frame` | `base_link` | TF 변환 목표 좌표계 |
| `tf_timeout_sec` | `0.2` | TF 대기 시간 |

## 11. 출력 토픽 설명
| 토픽 | 타입 | 설명 |
|---|---|---|
| `/sm_florence_2_vlm/detections` | `std_msgs/msg/String` | Detection JSON |
| `/sm_florence_2_vlm/debug/image` | `sensor_msgs/msg/Image` | bbox/좌표 표시 이미지 |
| `/sm_florence_2_vlm/markers` | `visualization_msgs/msg/MarkerArray` | RViz2 위치 마커 |
| `/sm_florence_2_vlm/roi_pointcloud` | `sensor_msgs/msg/PointCloud2` | bbox 또는 segmentation ROI 포인트 |

## 11-1. 객체 목록 입력 토픽
런타임 중 검출 대상 객체를 바꾸려면 `/sm_florence_2_vlm/target_objects` 토픽에 `std_msgs/String`을 publish하세요.

JSON 배열 형식:
```bash
ros2 topic pub --once /sm_florence_2_vlm/target_objects std_msgs/msg/String \
  "{data: '[\"mug\", \"bottle\", \"cup\"]'}"
```

콤마 문자열 형식:
```bash
ros2 topic pub --once /sm_florence_2_vlm/target_objects std_msgs/msg/String \
  "{data: 'mug,bottle,cup'}"
```

## 12. Detection JSON 구조
```json
{
  "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": "camera_color_optical_frame"},
  "target_frame": "base_link",
  "objects": [
    {
      "object_id": "person_0",
      "object_name": "person",
      "confidence": 0.87,
      "bbox": {"xmin": 120, "ymin": 80, "xmax": 300, "ymax": 420},
      "center_pixel": {"u": 210, "v": 250},
      "position_camera_frame": {"frame_id": "camera_color_optical_frame", "x": 0.35, "y": -0.12, "z": 2.15},
      "position_target_frame": {"frame_id": "base_link", "x": 1.05, "y": 0.18, "z": 0.74},
      "transform_available": true,
      "roi_point_count": 15234
    }
  ]
}
```

## 13. RViz2 시각화
RViz2에서 `MarkerArray` 디스플레이를 추가하고 topic을 `/sm_florence_2_vlm/markers`로 설정합니다. Debug image는 Image 디스플레이에서 `/sm_florence_2_vlm/debug/image`를 선택합니다.

## 14. Jetson Thor CUDA 확인
```bash
python3 - <<'PY'
import torch
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else "CPU")
PY
```

## 15. 실시간 갱신 구조
RGB/Depth/CameraInfo/PointCloud2는 `message_filters.ApproximateTimeSynchronizer`로 동기화됩니다. subscriber callback은 최신 프레임만 저장하고, Florence-2 추론은 `inference_rate_hz` 주기의 timer callback에서만 수행합니다. 추론 중에는 `is_inferencing` 플래그로 중복 실행을 막습니다.

## 16. 문제 해결
- `No module named 'torch'`: PyTorch가 설치되지 않았습니다. Jetson에서는 NVIDIA Jetson PyTorch 공식 문서의 wheel을 사용하세요.
- `No module named 'einops'`: Florence-2 remote code 의존성이 누락된 상태입니다. `python3 -m pip install einops --break-system-packages`를 실행하세요.
- Detection이 느리면 `inference_rate_hz`를 낮추거나 Florence-2 small/base 모델을 검토하세요.
- Jetson 속도 최적화는 `publish_roi_pointcloud=false`, `publish_markers=false`, `publish_debug_image=false`, `max_new_tokens=64`, `inference_input_width=640` 조합을 권장합니다.
- ROI PointCloud가 비거나 느리면 `roi_pointcloud_source=depth`를 우선 사용하세요. `pointcloud` 소스는 `ordered_pc=true` 환경에서 가장 효율적입니다.
- 작은 물체(예: 마우스)에서 배경 혼입이 크면 `roi_geometry_mode=depth_segmentation`을 사용하고, 누락이 생기면 `roi_geometry_fallback_to_bbox=true`를 유지하세요.
- ROI PointCloud가 비어 있으면 RealSense2가 `pointcloud.ordered_pc:=true`로 실행 중인지 확인하세요.
- `transform_available=false`가 계속 나오면 `tf2_echo base_link camera_color_optical_frame`로 TF tree를 확인하세요.
- CUDA가 잡히지 않으면 Jetson용 PyTorch wheel과 CUDA 버전을 확인하세요.

## 17. Custom Message 확장
현재는 `std_msgs/String` JSON으로 publish합니다. 이후 `sm_florence_2_vlm_msgs/msg/DetectionArray.msg`를 추가하면 타입 안정성과 downstream 처리 성능을 개선할 수 있습니다.

## 18. Tracker 확장 방향
향후 bbox/object_id를 ByteTrack, SORT, 또는 3D nearest-neighbor tracker와 연결하면 frame 간 object_id를 유지할 수 있습니다. 현재 `object_id_mode=per_frame`은 매 추론 프레임마다 `{object_name}_{index}`를 새로 부여합니다.
