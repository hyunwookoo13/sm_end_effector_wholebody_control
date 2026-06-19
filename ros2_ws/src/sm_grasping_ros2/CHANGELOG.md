## [v0.2.0] - 2026-04-30
### 추가된 기능
- GPD 백엔드 래퍼(`gpd_wrapper.py`) 추가
- GPD 소스 설치용 `setup.sh` 루틴 추가

### 변경된 내용
- 추론 노드를 AnyGrasp 의존성에서 GPD 기준 파라미터 구조로 전환
- 설정 파일에서 `grasping_*` 항목을 `gpd_*` 항목으로 교체
- README 및 패키지 설명을 GPD 기준으로 갱신

### 버그 수정
- 없음

## [v0.1.0] - 2026-04-30
### 추가된 기능
- `sm_grasping_ros2` 신규 패키지 스캐폴딩 추가
- `grasping_inference_node` 기본 구조 추가 (ROI 포인트클라우드 입력, grasp 토픽 출력)
- `gripper_marker_node` 기본 구조 추가 (그리퍼 MarkerArray 시각화)
- 모델 경로를 패키지 내부 `models/`로 관리하는 정책 반영

### 변경된 내용
- 없음

### 버그 수정
- 없음
