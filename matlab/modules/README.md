# MATLAB 모듈 구조 가이드

## 📁 폴더 구성

```
matlab/
├── modules/              # 재사용 가능한 모듈들
│   ├── ROS2Manager.m     # ROS2 네임스페이스 감지 및 토픽 관리
│   ├── ProcessManager.m  # 프로세스 정리 및 안전 종료
│   ├── SimulationManager.m # 시뮬레이션 자동 실행/종료
│   └── README.md         # 이 파일
├── demo/                 # 테스트 및 데모 스크립트
│   ├── quick_test.m      # 빠른 ROS2 연결 테스트
│   ├── circle_flight_demo.m # 원형 비행 데모 (자동 시뮬 실행)
│   └── full_verification_demo.m # 완전한 검증 데모 (예정)
├── main/                 # 최종 실행 스크립트
│   ├── integrated_main.m # 통합 시뮬레이션 (메인 예제)
│   └── circle_flight_main.m # 원형 비행 메인 (예정)
├── config/               # 설정 파일
│   └── SimulationConfig.m
├── logging/              # 로깅 관련 모듈
│   └── DataLogger.m
├── aircraft_state_receiver/  # 상태 수신 모듈
│   └── AircraftStateReceiver.m
├── decision_logic/       # 의사결정 로직
│   ├── ChargeStationManager.m
│   ├── RouteOptimizer.m
│   └── WeatherDataProcessor.m
└── verification/         # 검증 도구 (기존)
    ├── verify_ros2_connection.m
    ├── verify_data_quality.m
    └── check_topics.sh
```

## 🚀 핵심 매니저 클래스

### 1. **ROS2Manager**
- **역할**: ROS2 환경 관리 및 네임스페이스 자동 감지
- **주요 메서드**:
  - `initialize()` - ROS2 환경 초기화
  - `auto_detect_namespace()` - 네임스페이스 자동 감지
  - `get_topic(name)` - 전체 토픽 경로 반환
  - `get_service(name)` - 전체 서비스 경로 반환
  - `get_standard_topics()` - 표준 토픽 구조 반환
  - `check_topic_available(topic)` - 토픽 가용성 확인

### 2. **ProcessManager**
- **역할**: 프로세스 정리 및 리소스 해제
- **주요 메서드**:
  - `cleanup_all_simulation()` - 모든 시뮬레이션 프로세스 정리
  - `cleanup_gazebo_processes()` - Gazebo 프로세스 정리
  - `cleanup_ardupilot_processes()` - ArduPilot 프로세스 정리
  - `cleanup_ros2_nodes()` - ROS2 노드 정리
  - `cleanup_temp_files()` - 임시 파일 정리
  - `full_cleanup()` - 완전한 정리 (모든 항목)

### 3. **SimulationManager**
- **역할**: 시뮬레이션(Gazebo + ArduPilot) 자동 실행/관리
- **주요 메서드**:
  - `initialize()` - 초기화 (이전 프로세스 정리 포함)
  - `start_simulation()` - 시뮬레이션 자동 시작
  - `wait_for_topics()` - 토픽 준비 대기
  - `stop_simulation()` - 시뮬레이션 안전하게 종료
  - `check_simulation_status()` - 실행 상태 확인
  - `get_topics()` - 표준 토픽 반환
  - `get_services()` - 표준 서비스 반환
  - `get_namespace()` - 현재 네임스페이스 반환

## 💡 사용 예제

### 예제 1: 빠른 테스트

```matlab
% 경로 추가
addpath(genpath(pwd))

% 시뮬레이션 매니저 생성
sim = SimulationManager('verbose', true);

% 초기화 (이전 프로세스 정리 포함)
sim = sim.initialize();

% 시뮬레이션 자동 시작
sim = sim.start_simulation('auto_launch', true);

% 토픽 사용
topics = sim.get_topics();
sub = ros2subscriber(topics.imu);
```

### 예제 2: 안전한 종료

```matlab
% 프로세스 매니저로 정리
pm = ProcessManager('verbose', true);
pm = pm.cleanup_all_simulation();
pm.print_cleanup_log();
```

### 예제 3: 메인 스크립트 실행

```matlab
% 메인 스크립트 실행 (모든 기능 통합)
cd ~/OBDM_ws/matlab
addpath(genpath(pwd))
run('main/integrated_main.m')
```

## 🔄 실행 흐름

### 전형적인 워크플로우

1. **매니저 생성**
   ```matlab
   sim = SimulationManager('verbose', true);
   pm = ProcessManager('verbose', true);
   ```

2. **초기화** (안전 정리 포함)
   ```matlab
   sim = sim.initialize();
   ```

3. **시뮬레이션 시작**
   ```matlab
   sim = sim.start_simulation('auto_launch', true);
   ```

4. **ROS2 작업**
   ```matlab
   topics = sim.get_topics();
   sub = ros2subscriber(topics.imu);
   % ... 데이터 처리 ...
   ```

5. **안전 종료**
   ```matlab
   pm = pm.cleanup_all_simulation();
   ```

## 📋 데모 스크립트

### 1. `demo/quick_test.m`
- 빠른 ROS2 연결 테스트
- 시뮬레이션 자동 실행
- 토픽 구독 및 데이터 수신 확인
- 실행 시간: ~5-10초

### 2. `demo/circle_flight_demo.m`
- 원형 비행 제어 데모
- 시뮬레이션 자동 실행
- ROS2 서비스 연결 및 기본 테스트
- 웨이포인트 생성

### 3. `demo/full_verification_demo.m` (예정)
- 완전한 검증 (30초)
- 데이터 품질 분석
- 통계 리포팅

## 🎯 메인 스크립트

### 1. `main/integrated_main.m`
- 통합 시뮬레이션 및 제어 예제
- 모든 매니저 활용
- 10초 데이터 수집 및 분석
- 자동 정리 (onCleanup 사용)

### 2. `main/circle_flight_main.m` (예정)
- 실제 원형 비행 제어
- 완전한 이륙 → 비행 → 착륙 시퀀스
- 데이터 로깅

## ✅ 모범 사례

1. **항상 초기화**
   ```matlab
   sim = sim.initialize();  % 이전 프로세스 정리 포함
   ```

2. **자동 정리 설정**
   ```matlab
   cleanup_obj = onCleanup(@cleanup_function);
   ```

3. **에러 핸들링**
   ```matlab
   try
       % 작업
   catch ME
       pm = ProcessManager('verbose', true);
       pm = pm.cleanup_all_simulation();
   end
   ```

4. **네임스페이스 자동 감지**
   ```matlab
   % SimulationManager가 자동으로 처리
   ns = sim.get_namespace();  % '/ap' 또는 '/plane1' 등
   ```

## 🔧 문제 해결

### 시뮬레이션이 시작되지 않음
```matlab
% 강제 정리
pm = ProcessManager('verbose', true);
pm = pm.full_cleanup();

% 다시 시작
sim = sim.initialize();
```

### 토픽을 찾을 수 없음
```matlab
% 네임스페이스 확인
disp(sim.get_namespace());

% 활성 토픽 나열
topics_list = sim.ros2_manager.list_available_topics();
disp(topics_list);
```

### ROS2 연결 실패
```matlab
% ROS2 정보 확인
sim.ros2_manager.print_info();

% ROS_DOMAIN_ID 확인
disp(getenv('ROS_DOMAIN_ID'));
```

---

**문의**: 각 모듈의 메서드 상세 정보는 MATLAB에서 `help ClassName.methodName` 명령으로 확인 가능합니다.
