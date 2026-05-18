# OBDM (On-Demand Battery Management) 프로젝트

Iris 쿼드콥터의 상태 수집, 시뮬레이션, 제어, 의사결정 통합 플랫폼

## 프로젝트 개요

### 목표
드론이 목적지까지 도달하는 과정에서 다음 요소들을 고려하여 방문지(충전 스테이션) 선정:
- **충전 스테이션(착륙 패드)** 위치
- **항공기 상태** (배터리, 연료량, 속도 등)
- **지역별 날씨 상태** (온도, 강수, 바람, 습도, 기압 등)

### 기술 스택
| 영역 | 기술 |
|-----|-----|
| **시뮬레이션** | Gazebo (Harmonic), ROS2 (Humble), ArduPilot SITL |
| **항공기** | Iris 쿼드콥터 (ArduCopter) |
| **제어/의사결정** | MATLAB |
| **환경 데이터** | 기상청 ASOS API |

## 프로젝트 구조

```
OBDM_ws/
├── simulation/          # Gazebo + ROS2 + ArduPilot 시뮬레이션
│   ├── config/         # 설정 파일
│   ├── launch/         # ROS2 launch 파일
│   └── models/         # 항공기 URDF/SDF 모델
├── matlab/             # MATLAB 제어 로직
│   ├── aircraft_state_receiver/     # 항공기 상태 수신
│   ├── decision_logic/               # 의사결정 알고리즘
│   └── utils/                        # 유틸리티 함수
├── docs/               # 문서 및 참고 자료
└── README.md          # 이 파일
```

## 아키텍처

### 데이터 흐름
```
┌─────────────────────────────────────────────────┐
│  Gazebo Simulation (Iris Quadcopter + ArduPilot SITL) │
│  - 쿼드콥터 시뮬레이션                           │
│  - 센서 데이터 생성                             │
└──────────────────┬──────────────────────────────┘
                   │ ROS2 Topics
                   ↓
┌─────────────────────────────────────────────────┐
│  ROS2 Node (Bridge)                             │
│  - 시뮬레이션 데이터 수집                        │
│  - 토픽 발행                                    │
└──────────────────┬──────────────────────────────┘
                   │ 네트워크/파일
                   ↓
┌─────────────────────────────────────────────────┐
│  MATLAB (Flight Manager)                        │
│  1. 항공기 상태 수신                             │
│  2. 날씨 데이터 조회 (기상청 API)               │
│  3. 충전소 위치 확인                            │
│  4. 최적 경로 계산                              │
│  5. 명령 전송 (필요시)                          │
└─────────────────────────────────────────────────┘
```

## 시뮬레이터 흐름 가이드

현재 코드 기준의 실행 순서는 아래처럼 구성된다.

1. [matlab/main/integrated_main.m](matlab/main/integrated_main.m) 이 `SimulationManager` 를 통해 시뮬레이터를 시작한다.
2. [matlab/aircraft_state_receiver/AircraftStateReceiver.m](matlab/aircraft_state_receiver/AircraftStateReceiver.m) 이 ROS 2 토픽에서 상태를 수집한다.
3. [matlab/modules/CircleControlModule.m](matlab/modules/CircleControlModule.m) 이 Arm → GUIDED → Takeoff → Waypoint Pursuit 를 수행한다.
4. [docs/SIMULATOR_FLOW_GUIDE.md](docs/SIMULATOR_FLOW_GUIDE.md) 에 전체 흐름과 수식, 런치 구조를 상세히 정리했다.

핵심 수식은 다음과 같다.

$$
e(t) = R \cos t, \qquad n(t) = R \sin t
$$

$$
\Delta \text{lat} = \frac{n}{111319.9}, \qquad
\Delta \text{lon} = \frac{e}{111319.9 \cos \phi}
$$

$$
h_{\text{rel}} = h_{\text{abs}} - h_{\text{home}}
$$

이 구조는 시뮬레이터를 **환경 초기화 → 상태 수집 → 비행 제어 → 궤적 추종** 으로 분리해 디버깅과 확장을 쉽게 만드는 것을 목표로 한다.

## 시뮬레이션 환경 설정

### 시뮬레이션에서 제공하는 데이터
- **항공기 상태**: 위치(LAT/LON/ALT), 속도, 자세각(Roll/Pitch/Yaw)
- **센서 데이터**: GPS, IMU, 기압계, 배터리 정보
- **환경 데이터**: 바람, 중력 시뮬레이션

### 항공기 모델
- **유형**: Iris 쿼드콥터
- **ArduPilot 비행 모드**: Stabilize, AltHold, Loiter, Guided, Auto
- **통신**: MAVLink 프로토콜 (ROS2를 통한 변환)

## 시작하기

### 1단계: 시뮬레이션 환경 설정

[simulation/README.md](simulation/README.md) 참고

### 2단계: MATLAB 통합

[matlab/README.md](matlab/README.md) 참고

### 3단계: 테스트 및 실행

```bash
# 터미널 1: 시뮬레이션 실행
cd simulation
./run_simulation.sh

# 터미널 2: MATLAB 실행
cd matlab
matlab -r "aircraft_state_receiver"
```

## 참고 자료

### 공식 문서
- [ArduPilot SITL with Gazebo](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
- [기상청 ASOS API](https://data.kma.go.kr/api/selectApiDetail.do?pgmNo=42&openApiNo=241)
- [ROS2 공식 문서](https://docs.ros.org/en/humble/)

### 주요 개념
- **SITL** (Software In The Loop): 하드웨어 없이 소프트웨어로 비행체 시뮬레이션
- **MAVLink**: 자율주행 차량의 통신 프로토콜
- **온톨로지 기반 모델링**: 날씨 요소와 항공기 성능의 매핑

## 라이센스

OBDM Project - Open Source

## 문의 및 피드백

프로젝트 관련 질문이나 제안은 이슈 트래커를 통해 등록해주세요.
