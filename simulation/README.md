# 시뮬레이션 환경 설정

Gazebo + ROS2 + ArduPilot을 이용한 고정익 항공기 시뮬레이션 환경

## 개요

### 시뮬레이션 구성 요소
| 구성 | 설명 |
|-----|------|
| **ArduPilot SITL** | 소프트웨어 비행 제어 장치 |
| **Gazebo Harmonic** | 물리 기반 시뮬레이션 엔진 |
| **ROS2 Humble** | 로봇 미들웨어 및 통신 프레임워크 |
| **ardupilot_gz** | ArduPilot과 Gazebo 통합 패키지 |

### 고정익 항공기 특성

**선택 이유**: 
- 정확한 항법 요구 (배터리/연료 제약)
- 활공 능력 (배터리 부족 시 안전한 착륙 가능)
- 지역 날씨의 영향을 더 크게 받음 (바람, 기압)

**주요 파라미터**:
- 순항 속도: 약 15-25 m/s
- 최대 체공 시간: 설정에 따라 30-60분
- 배터리 용량 모의: 시뮬레이션에서 소모율 설정 가능

## 설치 및 구성

### 사전 요구사항
- Ubuntu 22.04 LTS 이상
- ROS2 Humble 설치됨
- Gazebo Harmonic 설치됨
- ArduPilot 소스 코드

### 1. ardupilot_gz 패키지 설치

```bash
# 워크스페이스 생성
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws

# 필요한 저장소 클론
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

# 환경 변수 설정 (bashrc에 추가)
export GZ_VERSION=harmonic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ardu_ws/src/ardupilot_gz/models

# 의존성 설치
rosdep install --from-paths src --ignore-src -y

# 빌드
colcon build --packages-up-to ardupilot_gz_bringup
```

### 2. 고정익 항공기 설정

고정익 항공기 모델 설정은 `ardupilot_gz` 패키지의 다음 파일에 정의:
- 모델: `models/` 디렉토리
- URDF/SDF: 항공기 형상 및 물리 특성 정의

## 사용 가능한 예제

### 고정익 항공기 관련 예제
```bash
# Quadplane (고정익 + 멀티콥터 하이브리드)
ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py

# 커스텀 고정익 모델 (필요시 설정)
# (현재 예제에서는 iris_runway 또는 quadplane 활용)
```

## 시뮬레이션 데이터 흐름

### ROS2 Topics (시뮬레이션 → 외부)

#### 항공기 위치 및 자세
```
Topic: /ap/tf
Type: geometry_msgs/TransformStamped
주기: 10 Hz
데이터: 위치(X,Y,Z), 자세(Quaternion)
```

#### 항공기 속도 및 자세각
```
Topic: /ap/imu/sample
Type: sensor_msgs/Imu
주기: 10-50 Hz
데이터: 가속도, 각속도, 자세각
```

#### GPS 정보
```
Topic: /ap/gps/sample
Type: sensor_msgs/NavSatFix
주기: 10 Hz
데이터: 위도, 경도, 고도, GPS 품질
```

#### 기압계 데이터
```
Topic: /ap/baro/sample
Type: sensor_msgs/FluidPressure
주기: 10 Hz
데이터: 대기압, 고도(기압 기반)
```

#### 배터리 정보
```
Topic: /ap/battery/state
Type: sensor_msgs/BatteryState (또는 커스텀)
주기: 1-2 Hz
데이터: 배터리 전압, 전류, 남은 용량(%)
```

### ROS2 Topics (외부 → 시뮬레이션)

#### 자동 조종 명령 (선택사항)
```
Topic: /ap/tf
Type: ardupilot_dds_msgs/GlobalPosition
주기: 필요시
데이터: 목표 위도, 경도, 고도
```

## 항공기 모델 정보

### ArduPilot Plane 설정

**비행 모드**:
- **MANUAL**: 수동 조종
- **CIRCLE**: 현재 위치 주변 원형 비행
- **STABILIZE**: 안정화 모드
- **FLY_BY_WIRE_A**: 자동 항법 (고도 유지)
- **FLY_BY_WIRE_B**: 자동 항법 (속도 유지)
- **AUTO**: 자동 경로 이동

**관측 가능한 상태**:
- 위치: GPS 위도/경도/고도
- 속도: 지상 속도(Ground Speed), 항속(Airspeed)
- 자세: Roll, Pitch, Yaw
- 센서: IMU, GPS, 기압계, 나침반
- 동력원: 배터리 전압/전류 (시뮬레이션)

## 마찬가지로 운영 가능한 스크립트

### 시뮬레이션 시작
```bash
# 1. ROS2 워크스페이스 소스
source ~/ardu_ws/install/setup.bash

# 2. 고정익 예제 실행
# (가장 가까운 예제 사용 - iris_runway 또는 quadplane)
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# 또는 커스텀 런치 파일 (고정익 전용)
# ros2 launch ardupilot_gz_bringup plane_runway.launch.py
```

### 시뮬레이션 모니터링
```bash
# RViz로 3D 시각화
rviz2

# 토픽 모니터링
ros2 topic list  # 사용 가능한 모든 토픽 확인
ros2 topic echo /ap/tf  # 특정 토픽 내용 확인
```

## MATLAB과의 연동

### 데이터 수신 흐름
1. **시뮬레이션**: Gazebo에서 항공기 상태 생성
2. **ROS2**: ArduPilot SITL이 상태를 ROS2 토픽으로 발행
3. **MATLAB**: ROS Toolbox를 이용해 토픽 구독 및 처리

자세한 내용은 [matlab/README.md](../matlab/README.md) 참고

## 문제 해결

### Gazebo 성능 저하
- OpenGL 가속화 확인: `glxinfo | grep "direct rendering"`
- 그래픽 품질 설정 조정

### ROS2 통신 문제
- `ROS_DOMAIN_ID` 환경 변수 확인
- 방화벽 설정 확인 (UDP 포트 7400-7409)

### 토픽 데이터 부재
- ArduPilot 매개변수 설정 확인
- `~/ardu_ws/install/ardupilot_sitl/lib/ardupilot_sitl/ardupilot_sitl` 경로 확인

## 참고 자료

- [ArduPilot ROS2 + Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ArduPilot Plane 매뉴얼](https://ardupilot.org/plane/)
- [Gazebo 공식 문서](https://gazebosim.org/docs/harmonic/install)
