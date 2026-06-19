# CHANGELOG

## [v0.1.12] - 2026-04-30

### 버그 수정
- `target_objects: []` 설정 시 `ParameterUninitializedException`으로 노드가 종료되던 문제 수정
- `target_objects` 파라미터가 `NOT_SET`/`None`인 경우를 안전하게 `[]`로 처리하도록 보강

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.12`로 동기화

## [v0.1.11] - 2026-04-30

### 기능 추가
- `target_objects`를 런타임에 토픽으로 갱신하는 기능 추가
- 신규 파라미터:
  - `target_objects_topic` (기본 `/sm_florence_2_vlm/target_objects`)
  - `use_target_objects_topic` (기본 `true`)
- 입력 포맷 지원:
  - JSON 배열 문자열 예: `["mug", "bottle"]`
  - 콤마 구분 문자열 예: `mug,bottle`

### 안정성 개선
- 추론 스레드와 토픽 콜백 간 `target_objects` 접근을 lock으로 보호하여 thread-safe하게 갱신

### 문서화
- YAML/README에 객체 목록 입력 토픽 사용법 및 예시 명령 추가

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.11`로 동기화

## [v0.1.10] - 2026-04-30

### 변경된 내용
- ROS2 패키지명을 `sm_florence_2_vlm`에서 `sm_florence_2_vlm_ros2`로 변경
- 디렉토리명을 `/home/kiro/colcon_ws/src/sm_florence_2_vlm_ros2`로 변경
- launch 파일명을 `sm_florence_2_vlm_ros2.launch.py`로 변경
- 기본 파라미터 파일명을 `sm_florence_2_vlm_ros2.yaml`로 변경
- `resource` 인덱스 파일명을 패키지명에 맞게 `sm_florence_2_vlm_ros2`로 변경
- 모델 캐시 기본 경로를 새 패키지 경로로 변경

### 문서화
- README의 빌드/실행 명령과 경로를 새 패키지명 기준으로 갱신

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.10`으로 동기화

## [v0.1.9] - 2026-04-29

### 성능 개선
- ROI 포인트클라우드 생성 소스 선택 파라미터 추가: `roi_pointcloud_source` (`depth` / `pointcloud`)
- 기본값을 `depth`로 설정하여 unorganized cloud 전체 순회 없이 ROI 픽셀만 3D 역투영해 고속 생성
- `pointcloud` 소스를 선택한 경우에만 PointCloud2 구독을 활성화하여 불필요한 구독 부하 감소

### 기능 추가
- `DepthToPointConverter.roi_to_camera_points` 추가: depth ROI를 camera frame XYZ 포인트 집합으로 직접 변환
- `PointCloudROIFilter.make_xyz_cloud` 추가: XYZ 필드 기반 ROI PointCloud2 생성 지원

### 문서화
- YAML/README에 `roi_pointcloud_source` 파라미터 설명 및 권장 사용법 추가

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.9`로 동기화

## [v0.1.8] - 2026-04-29

### 성능 개선
- ROI 포인트클라우드 고속 보호 옵션 추가: `roi_pointcloud_require_organized`(기본 `true`)
- `ordered_pc=false`로 들어오는 unorganized cloud(`height=1`)에서 전체 포인트 순회를 기본 생략하여 `post` 병목(수백 ms~1s) 완화

### 문서화
- README 파라미터 표에 `roi_pointcloud_require_organized` 설명 추가
- YAML 기본 설정에 `roi_pointcloud_require_organized: true` 반영

### 백업
- 수정 전 패키지 백업 생성: `backups/v0.1.7_20260429_162337.tar.gz`

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.8`로 동기화

## [v0.1.7] - 2026-04-29

### 성능 개선
- 추론 입력 다운스케일 파라미터 추가: `inference_input_width`(기본 `640`) 기반으로 Florence-2 입력 해상도를 축소해 추론 지연 감소
- bbox 좌표 원복 로직 추가: 다운스케일 추론 결과를 원본 해상도로 정확히 복원해 depth/ROI 계산 일관성 유지
- 단계별 성능 로그 추가: `enable_perf_log`, `perf_log_interval_sec` 파라미터로 `detect/post/pub/cv/total` 구간 ms 출력
- 성능 우선 기본 프로파일로 YAML 조정:
  - `publish_debug_image=false`
  - `publish_markers=false`
  - `publish_roi_pointcloud=false`
  - `inference_rate_hz=5.0`
  - `max_new_tokens=64`

### 문서화
- README 파라미터 표에 성능 관련 신규 항목(`inference_input_width`, `enable_perf_log`, `perf_log_interval_sec`) 추가
- Jetson 속도 최적화 권장 조합 문구 추가

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.7`로 동기화

## [v0.1.6] - 2026-04-29

### 성능 개선
- 추론 타이머 → 전용 루프 스레드 교체: `create_timer` + `is_inferencing` 플래그 방식 제거. `_inference_loop`이 `_stop_event`로 종료 제어되는 데몬 스레드로 실행되며 GPU 속도에 맞춰 최신 프레임을 즉시 소비 — 타이머 주기와 추론 완료 시점이 어긋날 때 발생하던 틱 스킵 문제 해결
- `inference_rate_hz`는 상한 속도 캡으로만 동작: 추론이 빠를수록 해당 Hz까지 자동으로 속도 향상
- `destroy_node` 오버라이드 추가: `_stop_event.set()`으로 추론 루프를 안전하게 종료

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.6`으로 동기화

## [v0.1.5] - 2026-04-29

### 성능 개선
- 디버그 이미지 발행을 추론 주기에서 분리: `_publish_results`에서 제거하고 `synced_callback`에서 카메라 rate(~30Hz)로 즉시 발행 — 이전 추론 결과(`latest_objects`)를 overlay하여 최신 프레임에 bbox 표시
- `latest_objects` / `objects_lock` 추가: 추론 스레드(쓰기)와 synced_callback(읽기) 간 thread-safe 공유

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.5`로 동기화

## [v0.1.4] - 2026-04-29

### 성능 개선
- `ApproximateTimeSynchronizer`에서 PointCloud2 토픽 분리: 4-topic sync → 3-topic sync(RGB+depth+CameraInfo)로 동기화 매칭 지연 감소. 포인트클라우드는 별도 구독(`_cloud_callback`)으로 최신 값 유지
- `sync_slop` 기본값 `0.1` → `0.05`s 축소: 동기화 대기 창 절반으로 감소
- `inference_rate_hz` 기본값 `3.0` → `10.0`Hz: 타이머 주기가 실제 추론 속도 병목이었던 문제 해결

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.4`로 동기화

## [v0.1.3] - 2026-04-29

### 버그 수정
- `_normalize_detections` 레이블 키 불일치 수정: `<OPEN_VOCABULARY_DETECTION>` 태스크 post-process 결과는 `'labels'`가 아닌 `'bboxes_labels'` 키 사용 (`processing_florence2.py` 확인) — 모든 검출이 필터링되던 근본 원인 해결
- `inference_timer_callback` 실행자 블로킹 수정: Florence-2 추론을 백그라운드 데몬 스레드로 분리 — 단일 스레드 executor에서 추론 중 `synced_callback` 실행 불가 문제 해결
- `PointCloudROIFilter` unorganized cloud 지원 추가: `ordered_pc=false`(기본값) 환경에서 ROI 포인트 클라우드가 항상 비어 있던 문제 해결 — unorganized cloud(height=1)에 대해 카메라 핀홀 투영 기반 frustum 필터링 적용

### 기능 개선
- `num_beams` 파라미터 추가 (기본값 `1`, greedy decoding): `num_beams=3` 대비 2~3배 추론 속도 향상
- `max_new_tokens` 파라미터 추가 (기본값 `256`): detection 출력 길이에 맞게 조정, 기존 1024 대비 불필요한 생성 제거

### 버전 관리
- `VERSION`, `package.xml`, `setup.py` 버전 표기를 `0.1.3`으로 동기화

## [v0.1.2] - 2026-04-29
### 변경된 내용
- Florence-2 모델 로딩 의존성에 `einops`를 추가했습니다.
- README 설치 가이드의 Python 의존성 명령에 `einops`를 반영했습니다.

### 버그 수정
- `No module named 'einops'`로 노드가 시작 직후 종료되던 문제를 해결했습니다.

## [v0.1.1] - 2026-04-29
### 추가된 기능
- Florence-2 모델 캐시 경로(`model_cache_dir`)를 명시적으로 지원하도록 개선했습니다.
- `transformers` 메이저 버전 호환성 가드를 추가해 Florence-2 비호환 버전 사용 시 명확한 오류 메시지를 제공하도록 했습니다.

### 변경된 내용
- Python 의존성 버전을 Jetson Thor + ROS2 Jazzy 환경 기준으로 고정했습니다.
- README의 설치/실행 가이드를 실제 운용 환경 기준으로 최신화했습니다.

### 버그 수정
- `transformers 5.x` 사용 시 Florence-2 로딩 과정에서 발생하던 `forced_bos_token_id` 예외 문제를 해결했습니다.
- `numpy`, `opencv-python`, `torch`, `transformers` 간 버전 충돌 가능성을 줄이도록 의존성 조합을 안정화했습니다.

## [v0.1.0] - 2026-04-29
### 추가된 기능
- 초기 ROS2 Jazzy 패키지 구조를 구성했습니다.
- RealSense2 토픽 동기화 기반 Florence-2 객체 검출 파이프라인을 구현했습니다.
- Detection JSON, Debug Image, MarkerArray, ROI PointCloud2 출력 기능을 추가했습니다.
