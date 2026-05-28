# MATLAB 통합 모듈

## 개요

MATLAB을 이용한 Iris 쿼드콥터 상태 수신 및 의사결정 로직 구현

### 주요 기능
1. **항공기 상태 수신**: ROS2 토픽으로부터 실시간 상태 데이터 수집
2. **데이터 처리**: 수신한 데이터 전처리 및 필터링
3. **의사결정 로직**: 위치, 배터리, 날씨를 고려한 경로 결정
4. **명령 전송**: 필요시 항공기에 명령 전송 (선택사항)

## 사전 요구사항

- MATLAB R2021b 이상 (ROS2 지원)
- MATLAB 추가 기능:
  - **Robotics System Toolbox** (ROS2 통신)
  - **Aerospace Toolbox** (항공우주 계산)
  - **Control System Toolbox** (제어 알고리즘)
  - **Optimization Toolbox** (최적화)

```matlab
% MATLAB에서 설치된 추가 기능 확인
>> matlabshareddata('toolboxdir','all')
```

## 프로젝트 구조

```
matlab/
├── aircraft_state_receiver/
│   ├── AircraftStateReceiver.m          # 메인 클래스
│   ├── ros2_subscriber_setup.m          # ROS2 구독 설정
│   └── examples/
│       └── example_receive_state.m      # 사용 예제
├── decision_logic/
│   ├── ChargeStationManager.m           # 충전소 관리
│   ├── WeatherDataProcessor.m           # 날씨 데이터 처리
│   └── RouteOptimizer.m                 # 경로 최적화
├── utils/
│   ├── DataLogger.m                     # 데이터 로깅
│   └── CoordinateConverter.m            # 좌표 변환
├── config/
│   └── simulation_config.m              # 설정 파일
└── README.md                            # 이 파일
```

## 시작하기

### 1단계: MATLAB 프로젝트 열기

```matlab
% 프로젝트 디렉토리로 이동
cd('/path/to/OBDM_ws/matlab')

% 현재 폴더를 MATLAB 경로에 추가
addpath(genpath(pwd))
```

### 2단계: ROS2 환경 설정

```matlab
% ROS2 디스트로 확인
ros2('gettimesync')

% 토픽 확인
ros2 topic list

% 메시지 타입 확인
ros2 msg list sensor_msgs/msg/
```

### 3단계: 항공기 상태 수신 시작

```matlab
% 간단한 사용 예제
run('aircraft_state_receiver/examples/example_receive_state.m')
```

## 주요 모듈 설명

### AircraftStateReceiver 클래스

**목적**: ROS2에서 항공기 상태 데이터 수신 및 관리

**주요 메서드**:
```matlab
% 객체 생성 및 초기화
receiver = AircraftStateReceiver('ros_domain_id', 0);

% 구독 시작
receiver.subscribe_to_topics();

% 현재 상태 조회
state = receiver.get_current_state();

% 상태 정보
state.position      % [lat, lon, alt] (도, 도, 미터)
state.velocity      % [vx, vy, vz] (m/s)
state.attitude      % [roll, pitch, yaw] (라디안)
state.battery_percent   % 배터리 남은 용량 (%)
state.timestamp     % 데이터 타임스탬프
```

**ROS2 구독 토픽**:
- `/ap/tf_static`: 고정 변환 정보
- `/ap/imu/experimental/data`: IMU 데이터 (Imu)
- `/ap/navsat`: GPS 데이터 (NavSatFix)
- `/ap/airspeed`: 대기속도 추정 (Airspeed)
- `/ap/battery`: 배터리 정보 (BatteryState)
- `/ap/rc`: RC 상태 (Rc)
- `/ap/status`: 차량 상태 (Status)

토픽 해석은 현재 ROS2 topic list를 기준으로 이루어지며, `/ap` 브리지뿐 아니라 `/mavros` 브리지의 일반적인 센서 토픽도 자동으로 후보에 포함됩니다.

ArduPilot ROS 2 인터페이스 문서와 workspace의 `AP_DDS` 구현 기준으로는 센서가 `/ap/navsat`, `/ap/battery`, `/ap/imu/experimental/data`, `/ap/airspeed`, `/ap/rc`, `/ap/status`, `/ap/tf_static` 쪽으로 나오고, 제어는 `/ap/cmd_gps_pose`, `/ap/cmd_vel`, `/ap/joy`를 MATLAB이 발행하는 구조입니다. 서비스는 `/ap/arm_motors`, `/ap/mode_switch`, `/ap/prearm_check`, `/ap/experimental/takeoff`를 사용합니다.

**ArduCopter 제어/피드백 토픽 후보**:
- `/ap/attitude`: 실제 자세 피드백
- `/ap/attitude_target`: 목표 자세 피드백
- `/ap/nav_controller_output`: 내비게이션 제어 출력
- `/ap/wind`: 풍향/풍속 추정
- `/ap/pid_tuning`: PID 튜닝 피드백

ArduCopter 소스 기준으로 제어 입력은 주로 GUIDED 계열 명령과 MAVLink 외부 제어 경로로 들어가며, 위 토픽들은 상태 및 제어 결과를 MATLAB에서 추적할 때 참고하는 텔레메트리입니다.

제어 경로는 브리지에 따라 자동 전환됩니다. ArduPilot 전용 브리지는 기존 `arm_motors` / `mode_switch` / `cmd_gps_pose` 흐름을 쓰고, MAVROS 브리지는 `/mavros/cmd/arming`, `/mavros/set_mode`, `/mavros/setpoint_position/local`을 우선 사용합니다.

MATLAB에서 AP DDS 센서 활성화 값을 바꾸려면 `SimulationManager.start_simulation`에 `ap_dds_sensor_config`를 넘기면 됩니다. 예를 들면 아래처럼 특정 센서만 켜고 나머지를 끌 수 있습니다.

```matlab
sim = SimulationManager('verbose', true);
sim = sim.start_simulation( ...
    'auto_launch', true, ...
    'wait_for_topics', true, ...
    'ap_dds_sensor_config', struct( ...
        'navsat', true, ...
        'imu', true, ...
        'battery', true, ...
        'airspeed', false, ...
        'pose', true, ...
        'twist', true, ...
        'geopose', false, ...
        'static_tf', true));
```

이 설정이 바뀌면 MATLAB이 `~/ardu_ws/src/ardupilot/libraries/AP_DDS/AP_DDS_config.h`를 갱신하고, 기본값대로면 `ardupilot_sitl`을 다시 빌드한 뒤 런치를 시작합니다.

원형 궤적 추종 제어 테스트는 [control/circle_flight.m](control/circle_flight.m)에서 실행할 수 있습니다. 이 스크립트는 원형 웨이포인트를 MATLAB에서 만들고 `/ap/cmd_gps_pose` waypoint goal 인터페이스로 순차 발행합니다.

### ChargeStationManager 클래스

**목적**: 충전 스테이션 정보 관리 및 거리 계산

```matlab
% 충전소 생성 및 등록
manager = ChargeStationManager();
manager.add_station('Station_A', 37.5665, 126.9780, 50);  % 서울 위치, 고도 50m
manager.add_station('Station_B', 37.4979, 127.0276, 50);  % 강남 위치

% 현재 위치에서 가장 가까운 충전소 찾기
[nearest_id, distance] = manager.find_nearest_station(current_lat, current_lon);
```

### WeatherDataProcessor 클래스

**목적**: 기상청 API로부터 데이터 수집 및 처리

```matlab
% 날씨 데이터 처리기 생성
weather = WeatherDataProcessor();

% 특정 지역의 날씨 데이터 조회
weather_data = weather.get_weather(lat, lon);
% 반환: 온도, 풍속, 강수량, 습도, 기압 등

% 항공기 성능에 미치는 영향도 계산
impact = weather.calculate_impact(wind_speed, altitude);
```

### RouteOptimizer 클래스

**목적**: 항공기 상태, 충전소, 날씨를 고려한 최적 경로 결정

```matlab
% 경로 최적화기 생성
optimizer = RouteOptimizer();

% 최적 충전소 결정
best_route = optimizer.find_optimal_route(...
    current_position, ...
    destination, ...
    available_stations, ...
    aircraft_state, ...
    weather_data);
```

## 데이터 흐름 다이어그램

```matlab
시뮬레이션 (Gazebo)
    ↓ ROS2 Topics
AircraftStateReceiver
    ↓ 상태 데이터
데이터 로거 → 저장/모니터링
    ↓
의사결정 로직
  ├─ ChargeStationManager (충전소 위치)
  ├─ WeatherDataProcessor (날씨 데이터)
  └─ RouteOptimizer (경로 최적화)
    ↓
결정된 경로 또는 명령
    ↓ 필요시 전송
시뮬레이션 (명령 수행)
```

## 설정 파일 (simulation_config.m)

```matlab
% 시뮬레이션 설정 파일
% 다음 항목들을 수정하여 시뮬레이션 커스터마이징

% ROS2 설정
ros_domain_id = 0;          % ROS_DOMAIN_ID 환경변수와 동일해야 함

% 항공기 설정
aircraft.cruise_speed = 20; % m/s
aircraft.battery_capacity = 5000;  % mAh
aircraft.consumption_rate = 50;    % mAh/min at cruise

% 충전소 설정
stations = struct();
stations.count = 3;
stations.locations = [37.5665 126.9780 50;   % Station A
                      37.4979 127.0276 50;   % Station B
                      37.5513 126.9725 50];  % Station C

% 날씨 데이터 기상청 API
weather.api_key = 'YOUR_KMA_API_KEY';
weather.update_interval = 300;  % 300초 마다 업데이트

% 데이터 로깅
logging.enabled = true;
logging.output_dir = './logs/';
logging.sample_rate = 1;  % 1 Hz
```

## 실행 예제

### 예제 1: 단순 상태 수신 및 표시

```matlab
% aircraft_state_receiver/examples/example_receive_state.m 참고
run example_receive_state.m
```

### 예제 2: 실시간 모니터링

```matlab
% 시뮬레이션과 함께 실행
% 터미널 1: 시뮬레이션 실행
% cd ~/OBDM_ws && ./scripts/system/run_simulation.sh

% MATLAB: 모니터링 스크립트 실행
cd ~/OBDM_ws/matlab
example_monitoring = true;
run_live_monitoring
```

## 문제 해결

### ROS2 연결 불가
```matlab
% 1. ROS_DOMAIN_ID 확인 (시뮬레이션과 동일해야 함)
>> getenv('ROS_DOMAIN_ID')

% 2. 토픽 확인
>> ros2 topic list

% 3. 토픽 데이터 확인
>> ros2 topic echo /ap/tf
```

### 토픽 메시지 타입 불일치
```matlab
% 메시지 타입 확인
>> ros2 msg show sensor_msgs/msg/BatteryState
```

### 데이터 수신 지연
```matlab
% 구독 옵션 조정
subscriber = ros2subscriber('/ap/tf', @callback_function, ...
    'Reliability', 'besteffort', ...
    'History', 'keepLast', ...
    'Depth', 1);
```

## 다음 단계

1. **기본 상태 수신 확인**: 항공기 위치, 속도, 자세 모니터링
2. **충전소 위치 설정**: 실제 운영 환경의 충전소 좌표 입력
3. **날씨 API 연동**: 기상청 API 키 설정 및 데이터 수신 확인
4. **경로 최적화 알고리즘 개발**: 의사결정 로직 구현
5. **시뮬레이션 검증**: 최적 경로대로 항공기 이동 확인

## 참고 자료

- [MATLAB ROS Toolbox 문서](https://www.mathworks.com/help/ros/index.html)
- [MATLAB ROS2 튜토리얼](https://www.mathworks.com/help/ros/ug/get-started-with-ros-2.html)
- [기상청 API 가이드](../docs/기상청01_지상(종관ASOS)시간자료_조회서비스_오픈API활용가이드_(1).pdf)
