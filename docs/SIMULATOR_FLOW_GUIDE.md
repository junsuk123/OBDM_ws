# 시뮬레이터 흐름 구성 가이드

이 문서는 현재 코드 공간을 기준으로, OBDM 시뮬레이터가 **어떻게 시작되고, 상태를 읽고, 이륙하고, 원형 경로를 추종하는지**를 단계별로 설명한다.

## 1. 전체 목적

현재 시뮬레이터의 목표는 다음 5단계를 안정적으로 연결하는 것이다.

1. Gazebo + ArduPilot SITL를 실행한다.
2. ROS 2 토픽과 서비스로 항공기 상태를 수집한다.
3. MATLAB에서 상태를 해석한다.
4. Arming / Mode / Takeoff를 수행한다.
5. 목표 웨이포인트를 발행하고 도달 여부를 검증한다.

핵심 오케스트레이션은 [matlab/main/integrated_main.m](../matlab/main/integrated_main.m#L1), [matlab/modules/SimulationManager.m](../matlab/modules/SimulationManager.m#L173), [matlab/aircraft_state_receiver/AircraftStateReceiver.m](../matlab/aircraft_state_receiver/AircraftStateReceiver.m#L46), [matlab/modules/CircleControlModule.m](../matlab/modules/CircleControlModule.m#L133) 에 분산되어 있다.

## 2. 권장 흐름

```mermaid
flowchart TD
    A[MATLAB entrypoint\nintegrated_main.m] --> B[SimulationManager.initialize()]
    B --> C[SimulationManager.start_simulation()]
    C --> D[DataCollectionModule.subscribe_to_topics()]
    D --> E[AircraftStateReceiver.subscribe_to_topics()]
    E --> F[CircleControlModule.initialize()]
    F --> G[wait_for_valid_state()]
    G --> H[arm_mode_takeoff_sequence()]
    H --> I[hover verification]
    I --> J[build_circle_waypoints()]
    J --> K[publish_goal() / publish_goal_local()]
    K --> L[wait_until_close()]
    L --> M[cleanup()]
```

## 3. 시작 단계

### 3.1 MATLAB 진입점

통합 실행 파일은 [matlab/main/integrated_main.m](../matlab/main/integrated_main.m#L1) 이다. 이 파일은 다음 순서로 동작한다.

- `SimulationManager` 생성 및 초기화
- 시뮬레이터 자동 실행
- 토픽 구독 및 데이터 수집
- 원형 비행 제어 실행

현재 구성에서는 `launch_mode` 에 따라 두 경로를 지원한다.

- `gazebo_bridge`: 공식 `iris_runway.launch.py` 기반 Gazebo + ArduPilot SITL
- `ap_dds_plane`: AP DDS 전용 plane 시뮬레이션

### 3.2 시뮬레이션 시작

[matlab/modules/SimulationManager.m](../matlab/modules/SimulationManager.m#L173) 의 `start_simulation()` 이 런치 명령을 만든다.

실행 시 중요한 환경 조건은 다음과 같다.

- `ROS_DOMAIN_ID=0`
- `GZ_VERSION=harmonic`
- `DISPLAY` / `XAUTHORITY` / `DBUS_SESSION_BUS_ADDRESS`
- 공식 런치: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=true rviz:=false`

## 4. 상태 수집 단계

### 4.1 토픽 구독

[matlab/aircraft_state_receiver/AircraftStateReceiver.m](../matlab/aircraft_state_receiver/AircraftStateReceiver.m#L46) 의 `subscribe_to_topics()` 는 다음 데이터를 구독한다.

- `tf`
- `imu`
- `odometry`
- `navsat` / `gpsfix`
- `geopose`
- `baro`
- `battery`
- `airspeed`

### 4.2 상태 구조

현재 상태는 `create_empty_state()` 에서 아래 필드로 시작한다.

- `position.latitude`, `position.longitude`
- `position.altitude` : 제어용 상대 고도
- `position.absolute_altitude` : GPS / GeoPose 절대 고도
- `position.relative_altitude` : 기준점 대비 상대 고도
- `position.x`, `position.y`, `position.z` : 로컬 오도메트리

중요한 설계 포인트는 **절대 고도와 상대 고도를 분리**하는 것이다. 이 분리가 없으면 이륙 전에도 수 미터의 고도가 있는 것처럼 보이는 오류가 생긴다.

## 5. 제어 단계

### 5.1 제어 모듈 초기화

[matlab/modules/CircleControlModule.m](../matlab/modules/CircleControlModule.m#L1) 의 `initialize()` 는 다음을 만든다.

- ROS 2 노드
- 상태 수신기
- 목표 발행 퍼블리셔 `(/ap/cmd_gps_pose)`
- `arm_motors`, `mode_switch`, `takeoff` 서비스 클라이언트

### 5.2 상태 준비 확인

`execute_circle_flight()` 는 먼저 `wait_for_valid_state()` 로 유효한 상태를 기다린다. 이 함수는 다음 조건을 만족해야 통과한다.

- 상태가 갱신되었을 것
- 절대 위치가 유효할 것
- 위도 / 경도가 NaN 이 아닐 것

이 단계가 중요한 이유는, 이후의 웨이포인트 계산이 **현재 기준점**을 필요로 하기 때문이다.

## 6. 이륙 단계

### 6.1 권장 명령 순서

제어 순서는 다음과 같다.

1. Arm
2. GUIDED 모드 전환
3. `MAV_CMD_NAV_TAKEOFF`
4. 상승 확인

이 순서는 [matlab/modules/CircleControlModule.m](../matlab/modules/CircleControlModule.m#L1315) 의 `arm_mode_takeoff_sequence()` 에 구현되어 있다.

### 6.2 통신 경로

현재 코드에서는 다음 순서로 연결을 시도한다.

1. `udp:127.0.0.1:14550`
2. `tcp:127.0.0.1:5760`

즉, **MAVProxy가 내보내는 UDP 14550을 우선 사용**하고, 실패하면 TCP 5760으로 폴백한다. 이 방식은 로컬 SITL / Gazebo 환경에서 가장 안정적이다.

## 7. 경로 추종 단계

### 7.1 원형 궤적 생성

원형 경로는 다음 식으로 생성된다.

$$
e(t) = R \cos t, \qquad n(t) = R \sin t
$$

여기서 `R` 은 반지름, `e` 는 East, `n` 은 North 방향 거리이다.

위도 / 경도 변환은 근사식으로 처리한다.

$$
\Delta \text{lat} = \frac{n}{111319.9}, \qquad
\Delta \text{lon} = \frac{e}{111319.9 \cos \phi}
$$

이 식은 [matlab/modules/CircleControlModule.m](../matlab/modules/CircleControlModule.m#L416) 의 `build_circle_waypoints()` 와 [matlab/control/circle_flight.m](../matlab/control/circle_flight.m#L1) / [matlab/control/track_follow_test.m](../matlab/control/track_follow_test.m#L1) 의 트랙 생성 코드와 같은 원리를 따른다.

### 7.2 목표 발행

목표는 두 경로 중 하나로 발행된다.

- ROS 2 퍼블리셔: `GeoPoseStamped` 또는 `GlobalPosition`
- PyMAVLink fallback: `set_position_target_global_int_send()` / `set_position_target_local_ned_send()`

[matlab/modules/CircleControlModule.m](../matlab/modules/CircleControlModule.m#L520) 의 `publish_goal()` 은 ROS 2 발행을 우선 시도하고, 실패하면 PyMAVLink 헬퍼로 전환한다.

### 7.3 도달 판정

`wait_until_close()` 는 목표와 현재 상태의 차이를 계산해 허용 오차 안에 들어오면 다음 웨이포인트로 넘어간다.

일반적인 판정은 다음과 같이 표현할 수 있다.

$$
d = \sqrt{(\Delta x)^2 + (\Delta y)^2 + (\Delta z)^2}
$$

`d < tolerance_m` 이면 해당 웨이포인트 도달로 본다.

## 8. 종료 단계

제어가 끝나면 다음 순서로 정리한다.

1. 마지막 호버 / 복귀 목표 발행
2. Disarm
3. 구독 해제
4. 프로세스 정리

이 정리는 `cleanup()` 과 `cleanup_on_exit()` 패턴으로 일관되게 처리한다.

## 9. 현재 코드에서 기억해야 할 설계 원칙

- **상태 분리**: 절대 고도와 상대 고도를 섞지 않는다.
- **통신 우선순위**: `udp:14550` 을 1순위로 둔다.
- **제어 경로 분기**: ROS 2 서비스가 없으면 MAVLink fallback 을 사용한다.
- **검증 우선**: 이륙 확인 없이 경로 추종을 시작하지 않는다.
- **문서와 코드 동기화**: `integrated_main.m` 에서 흐름을 바꾸면 이 가이드도 함께 갱신한다.

## 10. 실행 예시

```bash
matlab -batch "run('matlab/main/integrated_main.m');"
```

원형 비행만 따로 시험하려면 다음 파일을 참고한다.

- [matlab/modules/CircleControlModule.m](../matlab/modules/CircleControlModule.m)
- [matlab/control/circle_flight.m](../matlab/control/circle_flight.m)
- [matlab/control/track_follow_test.m](../matlab/control/track_follow_test.m)

## 11. 한 줄 요약

이 시뮬레이터는 **SimulationManager가 환경을 띄우고, AircraftStateReceiver가 상태를 읽고, CircleControlModule이 Arm → GUIDED → Takeoff → Waypoint Pursuit를 수행하는 3단 분리 구조**로 설계되어 있다.