# 휴리스틱 Top-Down Grasp Pose의 단계적 정렬 및 수직 접근 제어 개선

## 1. 미션 한 문장

이미 생성되는 휴리스틱 top-down grasp pose를 이용하되, 위치 정렬과 손목 자세 정렬을 단계적으로 수행하고 손목 회전에 따른 EE 위치 변화를 보상한 뒤 수직으로 접근하여 안정적으로 물체를 잡는다.

## 2. 배경

현재 `sm_grasping_ros2`는 실제 GPD 추론 대신 ROI PointCloud를 이용해 휴리스틱 grasp pose를 생성한다.

- Position: ROI PointCloud 평균점
- Roll/Pitch: top-down grasp가 되도록 고정 보정
- Yaw: ROI PointCloud의 XY PCA 결과
- 접근 방식: `APPROACH -> DESCEND -> GRASP -> LIFT -> HOLD`

따라서 top-down grasp pose 자체는 이미 존재한다. 현재 문제는 pose 생성이 아니라, 베이스와 여러 관절이 위치 및 자세 오차를 동시에 줄이면서 EE가 물체에 대각선 또는 곡선으로 접근할 수 있다는 점이다.

## 3. 이번 단계의 목표

1. 베이스와 link1으로 물체 방향을 먼저 정렬한다.
2. link2와 link3으로 EE를 목표 상공의 안전 높이까지 이동한다.
3. 베이스 이동을 멈추고 상공의 XY 영역을 유지한다.
4. link4, link5, link6으로 top-down 손목 자세와 물체 yaw를 정렬한다.
5. 손목 회전으로 발생하는 TCP/EE 위치 변화를 link1~3이 보상한다.
6. XY와 손목 자세가 모두 안정된 뒤에만 수직 하강한다.
7. 하강 중에는 베이스를 정지하고 EE의 XY 및 orientation을 유지하면서 Z만 감소시킨다.
8. grasp 후 동일한 원칙으로 수직 상승한다.

## 4. 제안 상태 머신

```text
COARSE_ALIGN
    베이스 + link1로 목표 방향 정렬
        |
        v
OVERHEAD_POSITION
    link2/3으로 목표 상공 위치 확보
        |
        v
WRIST_ORIENTATION_ALIGN
    link4/5/6으로 top-down + object yaw 정렬
    link1/2/3으로 EE 위치 변화 보상
        |
        v
READY_TO_DESCEND
    XY, Z 접근 높이, orientation 안정성 확인
        |
        v
DESCEND
    베이스 정지, XY/orientation 유지, Z만 감소
        |
        v
GRASP
    위치 정지, 그리퍼 닫기
        |
        v
LIFT
    XY/orientation 유지, Z만 증가
        |
        v
HOLD
```

## 5. 각 상태의 제어 책임

### COARSE_ALIGN

- 베이스와 link1 사용 가능
- 물체 방향의 큰 yaw 오차 제거
- 팔은 안전한 접근 자세 유지
- 목표 TF가 유효하고 최신일 때만 진행

### OVERHEAD_POSITION

- link2와 link3을 함께 사용해 목표 상공으로 이동
- 아직 물체 높이까지 내려가지 않음
- EE 위치 목표는 grasp pose에 approach clearance를 더한 위치
- 위치 오차가 연속된 여러 제어 주기 동안 허용 범위 안에 있어야 다음 상태로 진행

### WRIST_ORIENTATION_ALIGN

- 현재 휴리스틱 모드에서는 top-down roll/pitch와 PCA yaw를 목표로 사용
- link4, link5, link6을 이용해 손목 orientation 정렬
- 손목 회전으로 EE/TCP의 XY 및 Z가 변하면 link1~3이 이를 보상
- 특정 손목 관절 각도를 단순 고정하지 않고, 세계/목표 좌표계 기준 EE orientation을 유지

### READY_TO_DESCEND

아래 조건을 모두 만족할 때만 하강한다.

- 베이스 속도가 사실상 0
- XY 오차가 허용 범위 안
- 접근 높이 오차가 허용 범위 안
- orientation 오차가 허용 범위 안
- target TF가 최신 상태
- 관절 한계 및 작업 공간 조건 만족
- 위 조건이 일정 횟수 연속으로 유지됨

### DESCEND

- 베이스 command는 0으로 고정
- 목표 X/Y 및 orientation 유지
- 목표 Z만 제한된 속도로 감소
- link2/3의 결합 운동으로 생기는 rho 변화를 함께 보상
- XY 또는 orientation 오차가 이탈 임계값을 넘으면 즉시 하강 중지 후 재정렬

### GRASP

- 베이스와 팔 위치 이동 정지
- 그리퍼만 닫기
- 설정된 유지 시간 또는 grasp 확인 조건 이후 LIFT로 전환

### LIFT

- DESCEND의 역방향 경로 사용
- XY 및 orientation 유지
- Z만 안전 높이까지 증가
- 완료 후 HOLD 보고

## 6. 안전 조건

- 상태 진입과 이탈 허용 오차를 다르게 두어 chattering을 방지한다.
- target TF가 오래되면 DESCEND를 시작하거나 계속하지 않는다.
- DESCEND 중 베이스가 움직이면 즉시 정지한다.
- XY 또는 orientation 오차가 커지면 Z 하강을 멈춘다.
- 관절 한계에 가까워지면 안전 자세로 복귀한다.
- 속도, 가속도 및 한 제어 주기의 position command 변화량을 제한한다.
- 새로운 grasp target이 들어와도 PICK 동작 중에는 현재 선택된 target을 고정한다.

## 7. 이번 단계에서 하지 않는 것

- 실제 GPD 백엔드 연결
- 여러 6DoF grasp 후보의 점수 비교
- 측면 및 대각선 grasp 실행
- MoveIt 기반 충돌 회피 궤적 생성
- 힘/토크 센서 기반 접촉 제어

이번 단계는 기존 휴리스틱 top-down pose를 안정적으로 실행하는 데 집중한다.

## 8. GPD 확장 시 유지할 구조

현재 입력은 다음과 같다.

```text
ROI centroid position + fixed top-down roll/pitch + PCA yaw
```

향후 입력은 다음으로 교체한다.

```text
GPD grasp position + GPD grasp quaternion + grasp score/opening
```

상태 머신은 `WRIST_ORIENTATION_ALIGN`에서 top-down orientation 대신 GPD quaternion을 추종하고, `DESCEND`에서는 세계 Z축이 아니라 GPD가 제시한 approach axis를 따라 이동하도록 확장한다. 위치 정렬, 자세 정렬, 접근, grasp, 이탈을 분리하는 상위 구조는 유지한다.

## 9. 구현 순서

1. 현재 제어 로그에 EE 목표/현재 위치, orientation 오차, grasp phase를 기록한다.
2. 기존 `APPROACH`를 `COARSE_ALIGN`, `OVERHEAD_POSITION`, `WRIST_ORIENTATION_ALIGN`으로 분리한다.
3. 손목 orientation 오차 계산과 정렬 완료 조건을 명확히 한다.
4. 손목 회전 중 EE 위치를 link1~3이 보상하도록 제어한다.
5. 베이스가 정지한 상태에서만 `DESCEND`가 시작되도록 한다.
6. `DESCEND` 중 XY/orientation hold와 Z-only 목표 갱신을 구현한다.
7. 동일한 조건으로 수직 `LIFT`를 구현한다.
8. 시뮬레이션에서 캔으로 검증한 뒤 사과, 컵, 바나나 순으로 확인한다.

## 10. 완료 기준

- EE가 목표 상공에 도착한 뒤 손목 orientation을 정렬한다.
- 손목 정렬 중 EE가 목표 상공 영역을 벗어나지 않는다.
- 하강 시작 전에 베이스가 정지한다.
- DESCEND 구간에서 EE의 XY 이동량이 정한 허용 범위 안에 있다.
- DESCEND 구간에서 그리퍼 orientation이 허용 범위 안에 유지된다.
- 캔을 향한 접근 궤적이 측면 관찰 시 명확한 수직 하강으로 보인다.
- grasp 후 캔이 안정적으로 들어 올려지고 `PICK:HOLD`가 보고된다.
- 기존 Pick/Place task manager 및 Nav2 흐름을 깨뜨리지 않는다.

## 11. 권장 초기 검증 지표

정확한 값은 시뮬레이션 결과를 보고 조정한다.

- 상공 XY 정렬 오차: 2~3cm 이내
- 손목 orientation 오차: 5도 이내
- 하강 중 XY drift: 1~2cm 이내
- target TF 최대 age: 0.2초 이내
- 정렬 완료 연속 확인: 5~10 control cycles
- 하강 속도: 시연 안전성을 우선한 저속부터 시작

