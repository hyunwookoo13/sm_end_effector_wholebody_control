# Nav2–정밀 제어 하이브리드 전환

## 목표

Nav2의 장거리 경로 추종과 물체 기반 정밀 제어 사이에 전환 구역과 속도 블렌딩을 적용해 정지와 재출발 없이 연속적으로 접근한다.

## 상태 흐름

```text
NAVIGATE_PICK / NAVIGATE_PLACE
    Nav2 100%
        |
        | target distance <= hybrid_outer_distance_m
        v
HYBRID_PICK / HYBRID_PLACE
    Nav2 + precision smoothstep blending
        |
        | target distance <= hybrid_inner_distance_m
        v
PICK / PLACE
    precision 100%, Nav2 goal cancel
```

## 블렌딩

```text
cmd_vel = (1 - weight) * cmd_vel_navigation
        + weight * cmd_vel_manipulation
```

`weight`는 전환 구역에서 smoothstep으로 `0.0 -> 1.0` 증가한다.

- outer boundary: Nav2 100%
- middle: Nav2 50%, precision 50%
- inner boundary: precision 100%

정밀 명령이 아직 도착하지 않았으면 mux는 Nav2 명령을 유지한다. 두 명령이 모두 없거나 오래되면 정지한다.

## 기본 파라미터

```text
enable_hybrid_handoff:=true
hybrid_outer_distance_m:=1.40
hybrid_inner_distance_m:=0.85
mux_max_linear_velocity_mps:=1.50
mux_max_angular_velocity_rps:=1.40
mux_max_linear_acceleration_mps2:=2.00
mux_max_angular_acceleration_rps2:=2.00
```

## 관련 토픽

```text
/cmd_vel_navigation     Nav2 출력
/cmd_vel_manipulation   정밀 제어 출력
/base_control_mode      NAVIGATION, HYBRID, MANIPULATION, RETREAT, STOP
/base_control_blend     정밀 제어 가중치 0.0~1.0
/cmd_vel                mux 최종 출력
/pick_place_task_state  task phase, navigation state, blend weight
```

## 안전 동작

- `STOP` 모드는 가속도 ramp 없이 즉시 0을 출력한다.
- 그 외 모드 전환에는 공통 속도 및 가속도 제한을 적용한다.
- command가 timeout을 넘으면 해당 입력을 사용하지 않는다.
- inner boundary 진입 시 Nav2 action을 취소한다.
- 취소 결과는 실패가 아닌 `HANDOFF`로 기록한다.
- Place target이 직접 접근 범위 안이면 기존 정책대로 Nav2와 hybrid 구간을 모두 생략한다.

## 실행 예시

안전을 위해 자동 시작을 끈다.

```bash
ros2 launch ee_switch_debug florence_long_range_pick_place_control.launch.py \
  enable_nav2:=true \
  autostart:=false \
  enable_hybrid_handoff:=true \
  hybrid_outer_distance_m:=1.40 \
  hybrid_inner_distance_m:=0.85
```

## 주행 확인

```bash
ros2 topic echo /base_control_mode
ros2 topic echo /base_control_blend
ros2 topic echo /cmd_vel_navigation
ros2 topic echo /cmd_vel_manipulation
ros2 topic echo /cmd_vel
```

정상적인 로그 흐름:

```text
Task phase: NAVIGATE_PICK -> HYBRID_PICK
hybrid pick: distance=..., precision_weight=...
Hybrid handoff complete: precision control owns pick approach
```

## 튜닝 지침

- 전환이 여전히 늦으면 `hybrid_outer_distance_m`를 늘린다.
- 정밀 제어 구간이 너무 길면 `hybrid_inner_distance_m`를 줄이지 말고 outer를 줄인다.
- 전환 순간 가속이 약하면 mux acceleration을 조금 높인다.
- 진동하거나 급하면 mux acceleration을 낮춘다.
- 좁은 장애물 환경에서 Nav2가 지나치게 보수적이면 inflation radius와 DWB critic을 재조정한다.
