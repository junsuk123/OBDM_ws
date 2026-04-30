# OBDM 프로젝트 - 설치 및 시작 가이드

전체 시뮬레이션 및 MATLAB 통합 환경 설정 및 실행 방법

## 빠른 시작 (Quick Start)

```bash
# 1. 시뮬레이션 실행 (터미널 1)
cd simulation
chmod +x run_simulation.sh
./run_simulation.sh

# 2. MATLAB 예제 실행 (터미널 2)
cd matlab
matlab -r "run('aircraft_state_receiver/examples/example_receive_state.m')"
```

## 사전 요구사항

### 시스템 사양
- Ubuntu 22.04 LTS 이상
- 최소 8GB RAM
- OpenGL 하드웨어 가속화 지원

### 필수 소프트웨어

#### 1. ROS2 Humble
```bash
# 설치 가이드: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Deb.html
sudo apt update && sudo apt upgrade
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-rosdds
```

#### 2. Gazebo Harmonic
```bash
# 저장소 추가
sudo apt install wget
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# 설치
sudo apt update
sudo apt install gz-harmonic
```

#### 3. ArduPilot 및 ardupilot_gz
```bash
# 워크스페이스 생성
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws

# 필수 저장소 클론
vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

# 환경 변수 설정 (~/.bashrc에 추가)
export GZ_VERSION=harmonic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ardu_ws/src/ardupilot_gz/models

# 의존성 설치
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y

# 빌드
colcon build --packages-up-to ardupilot_gz_bringup
```

#### 4. MATLAB 추가 기능
- **Robotics System Toolbox** (ROS2 통신)
- **Aerospace Toolbox** (항공우주 계산)
- **Optimization Toolbox** (경로 최적화)

```matlab
% MATLAB 추가 기능 설치 확인
>> matlabshareddata('toolboxdir','all')
```

## 환경 변수 설정

`~/.bashrc` 또는 `~/.zshrc`에 다음 추가:

```bash
# ROS2 설정
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Gazebo 설정
export GZ_VERSION=harmonic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/ardu_ws/src/ardupilot_gz/models

# ArduPilot SITL 경로
export PATH=$PATH:$HOME/ardu_ws/install/ardupilot_sitl/lib/ardupilot_sitl
```

변경 사항 적용:
```bash
source ~/.bashrc
```

## 시뮬레이션 실행

### 기본 실행 (대화형)
```bash
cd ~/OBDM_ws/simulation
./run_simulation.sh

# 메뉴에서 선택:
# 1. iris_runway (권장 - 멀티콥터)
# 4. alti_transition_runway (고정익/Quadplane)
```

### 직접 실행 (스크립트)
```bash
cd ~/ardu_ws
source install/setup.bash

# Iris (멀티콥터)
ros2 launch ardupilot_gz_bringup iris_runway.launch.py

# Quadplane (고정익/멀티콥터 하이브리드)
ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py
```

### 시뮬레이션 모니터링
```bash
# 새로운 터미널에서
source ~/ardu_ws/install/setup.bash

# 토픽 목록 확인
ros2 topic list

# 특정 토픽 구독
ros2 topic echo /ap/tf          # 위치/자세
ros2 topic echo /ap/imu/sample  # IMU 데이터
ros2 topic echo /ap/gps/sample  # GPS
ros2 topic echo /ap/battery/state # 배터리
```

## MATLAB 실행

### 1단계: 환경 설정
```matlab
% MATLAB 시작
cd ~/OBDM_ws/matlab

% 경로 추가
addpath(genpath(pwd))

% 설정 확인
SimulationConfig.print_config();
SimulationConfig.validate_ros2_environment();
```

### 2단계: 항공기 상태 수신 (기본 예제)
```matlab
% 예제 1: 상태 수신
cd ~/OBDM_ws/matlab/aircraft_state_receiver/examples
run('example_receive_state.m')

% 또는 직접 코드:
receiver = AircraftStateReceiver('verbose', true);
receiver.subscribe_to_topics();

% 상태 조회
for i = 1:60
    state = receiver.get_current_state();
    if state.is_valid
        fprintf('Alt: %.1f m, Bat: %.1f%%\n', ...
            state.position.altitude, state.battery.percent);
    end
    pause(2);
end

receiver.print_state();
```

### 3단계: 의사결정 로직 테스트
```matlab
% 충전 스테이션 관리자 테스트
charge_mgr = ChargeStationManager('verbose', true);
charge_mgr.print_stations();

% 가장 가까운 충전소 찾기
[station_id, distance] = charge_mgr.find_nearby_stations(...
    37.5665, 126.9780);

% 날씨 데이터 처리 테스트
weather_proc = WeatherDataProcessor('verbose', true);
weather = weather_proc.get_weather(37.5665, 126.9780);
weather_proc.print_weather(weather);

% 경로 최적화 테스트
route_optimizer = RouteOptimizer(...
    'charge_manager', charge_mgr, ...
    'weather_processor', weather_proc, ...
    'verbose', true);

% 최적 충전소 결정
state = receiver.get_current_state();
[best_station_id, decision_info] = ...
    route_optimizer.find_optimal_charge_station(...
    [state.position.latitude, state.position.longitude, ...
     state.position.altitude], ...
    state, weather);
```

## 데이터 흐름 검증

### 1. 시뮬레이션 데이터 확인
```bash
# 시뮬레이션 실행 상태 확인
source ~/ardu_ws/install/setup.bash

# 토픽 데이터 모니터링 (5초)
ros2 topic echo /ap/gps/sample -c 5
ros2 topic echo /ap/battery/state -c 5
```

### 2. MATLAB 수신 확인
```matlab
% 간단한 수신 테스트
receiver = AircraftStateReceiver();
receiver.subscribe_to_topics();

% 1초 대기 후 상태 확인
pause(1);
state = receiver.get_current_state();

% 데이터 유효성 확인
fprintf('데이터 유효: %s\n', onoff(state.is_valid));
fprintf('배터리: %.1f%%\n', state.battery.percent);
fprintf('고도: %.1f m\n', state.position.altitude);
```

## 문제 해결

### ROS2 연결 문제

**증상**: "토픽 찾을 수 없음" 또는 "구독 실패"

**해결**:
```bash
# 1. 환경 변수 확인
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY

# 2. 토픽 확인 (시뮬레이션 실행 중)
ros2 topic list

# 3. 방화벽 확인 (필요시)
# ROS2는 UDP 포트 7400-7409 사용

# 4. ROS_DOMAIN_ID 재설정
export ROS_DOMAIN_ID=0
ros2 topic list
```

### MATLAB ROS2 인식 실패

**증상**: "Failed to subscribe" 또는 "Could not find message definition"

**해결**:
```matlab
% 1. ROS2 설정 확인
ros2('gettimesync')

% 2. 메시지 타입 확인
ros2 msg list | grep sensor_msgs

% 3. 메시지 정의 확인
ros2 msg show sensor_msgs/msg/Imu

% 4. MATLAB 재시작 및 경로 재설정
clear all
addpath(genpath(fullfile(pwd,'matlab')))
```

### Gazebo 성능 문제

**증상**: 매우 느린 시뮬레이션 속도

**해결**:
```bash
# 1. OpenGL 가속화 확인
glxinfo | grep "direct rendering"

# 2. 그래픽 품질 조정 (Gazebo 실행 시)
export IGNITION_RENDERING_ENGINE_PATH=/usr/lib/x86_64-linux-gnu/ign-rendering-6/
ros2 launch ardupilot_gz_bringup iris_runway.launch.py ign_args:="-r -s -v 2"

# 3. 시스템 성능 확인
top  # CPU/메모리 사용량 확인
```

## 다음 단계

### 1. 기본 기능 검증 ✓
- [x] 시뮬레이션 실행
- [x] 항공기 상태 수신
- [x] 데이터 로깅

### 2. 의사결정 로직 개발
- [ ] 실시간 경로 최적화
- [ ] 날씨 API 통합 (기상청)
- [ ] 배터리 소모 모델링

### 3. 고급 기능
- [ ] 경로 시뮬레이션 시각화
- [ ] 시뮬레이션 ↔ MATLAB 양방향 통신
- [ ] 실시간 의사결정 피드백

## 유용한 명령어

```bash
# ROS2 토픽 모니터링 (주기적)
watch -n 0.5 'ros2 topic list'

# 특정 노드의 토픽만 확인
ros2 node info /ardupilot_gz_bridge

# 토픽 주기 확인
ros2 topic hz /ap/imu/sample

# 토픽 대역폭 확인
ros2 topic bw /ap/imu/sample
```

## 참고 자료

### 공식 문서
- [ArduPilot ROS2+Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ROS2 Humble 공식 문서](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic 문서](https://gazebosim.org/docs/harmonic)
- [MATLAB ROS Toolbox](https://www.mathworks.com/help/ros/)

### 추가 자료
- [기상청 ASOS API](https://data.kma.go.kr/api/selectApiDetail.do?pgmNo=42&openApiNo=241)
- [MAVLink 프로토콜](https://mavlink.io/)
- [고정익 항공기 항법](https://ardupilot.org/plane/)

## 지원 및 문의

문제가 발생하면:
1. 로그 파일 확인: `logs/`
2. [ArduPilot 포럼](https://discuss.ardupilot.org/)
3. [ROS Answers](https://answers.ros.org/)
4. GitHub Issues (프로젝트 저장소)
