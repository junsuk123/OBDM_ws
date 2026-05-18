# 📚 MATLAB 구조 재정리 완료 가이드

## 🎯 완료된 작업

### ✅ 1. 폴더 구조 생성
```
matlab/
├── modules/              ← 재사용 모듈 (새로 생성)
│   ├── ROS2Manager.m     (네임스페이스/토픽 관리)
│   ├── ProcessManager.m  (프로세스 정리)
│   ├── SimulationManager.m (시뮬레이션 자동 관리)
│   └── README.md         (모듈 가이드)
├── demo/                 ← 테스트 스크립트 (새로 생성)
│   ├── quick_test.m      (빠른 테스트)
│   └── circle_flight_demo.m (비행 데모)
├── main/                 ← 메인 스크립트 (새로 생성)
│   └── integrated_main.m (통합 제어)
├── config/               (기존)
├── logging/              (기존)
├── aircraft_state_receiver/ (기존)
├── decision_logic/       (기존)
└── verification/         (기존)
```

### ✅ 2. 핵심 매니저 모듈 3개 개발

#### ROS2Manager
- 네임스페이스 자동 감지
- 토픽/서비스 경로 관리
- 토픽 가용성 확인

#### ProcessManager
- 시뮬레이션 프로세스 정리
- Gazebo/ArduPilot 프로세스 종료
- 임시 파일 정리
- 안전 종료 보장

#### SimulationManager
- Gazebo + ArduPilot 자동 실행
- 토픽 준비 대기
- 상태 모니터링
- ROS2Manager/ProcessManager 통합

### ✅ 3. 데모 스크립트 개발

#### quick_test.m
```matlab
run('demo/quick_test.m')
```
- 시뮬레이션 자동 시작
- 빠른 연결 테스트
- 5초 데이터 수신

#### circle_flight_demo.m
```matlab
run('demo/circle_flight_demo.m')
```
- 시뮬레이션 자동 시작
- 웨이포인트 생성
- 서비스 연결 테스트

### ✅ 4. 메인 스크립트 개발

#### integrated_main.m
```matlab
run('main/integrated_main.m')
```
- 완전한 통합 워크플로우
- 10초 데이터 수집
- 자동 정리 (onCleanup)

---

## 🚀 빠른 시작 가이드

### 최신 방식 (권장)

**1️⃣ 빠른 테스트**
```matlab
cd matlab
addpath(genpath(pwd))
run('demo/quick_test.m')
```

**2️⃣ 통합 메인 실행**
```matlab
run('main/integrated_main.m')
```

**3️⃣ 수동 제어 (고급)**
```matlab
sim = SimulationManager('verbose', true);
sim = sim.initialize();
sim = sim.start_simulation();

topics = sim.get_topics();
sub = ros2subscriber(topics.imu);

% ... 데이터 처리 ...

pm = ProcessManager('verbose', true);
pm = pm.cleanup_all_simulation();
```

### 기존 방식 (호환성 유지)

**검증 스크립트**
```matlab
run('verification/verify_ros2_connection.m')
```

**원형 비행 제어**
```matlab
run('control/circle_flight.m')
```

---

## 📋 모듈별 특징

### ROS2Manager
```matlab
% 사용 예
ros2_mgr = ROS2Manager('verbose', true);
ros2_mgr = ros2_mgr.initialize();

% 네임스페이스 자동 감지
ns = ros2_mgr.get_namespace();  % '/ap', '/plane1' 등

% 토픽 관리
topic = ros2_mgr.get_topic('imu/sample');
svc = ros2_mgr.get_service('arm_motors');

% 토픽 가용성 확인
is_available = ros2_mgr.check_topic_available(topic);
```

### ProcessManager
```matlab
% 사용 예
pm = ProcessManager('verbose', true);

% 개별 정리
pm = pm.cleanup_gazebo_processes();
pm = pm.cleanup_ardupilot_processes();
pm = pm.cleanup_ros2_nodes();

% 완전 정리
pm = pm.full_cleanup();

% 로그 확인
pm.print_cleanup_log();
```

### SimulationManager
```matlab
% 사용 예
sim = SimulationManager('verbose', true);

% 초기화 + 정리
sim = sim.initialize();

% 자동 시작
sim = sim.start_simulation('auto_launch', true);

% 토픽 대기
sim = sim.wait_for_topics(30);  % 30초

% 상태 조회
ns = sim.get_namespace();
topics = sim.get_topics();
services = sim.get_services();

% 종료
sim = sim.stop_simulation();
```

---

## 🔄 워크플로우 예시

### 워크플로우 1: 데모 스크립트
```
demo/quick_test.m 실행
  ↓
[시뮬레이션 자동 실행]
  ↓
[ROS2 연결 테스트]
  ↓
[데이터 수신]
  ↓
[자동 정리]
```

### 워크플로우 2: 메인 스크립트
```
main/integrated_main.m 실행
  ↓
[이전 프로세스 정리]
  ↓
[ROS2 초기화]
  ↓
[시뮬레이션 자동 시작]
  ↓
[데이터 수집 (10초)]
  ↓
[통계 분석]
  ↓
[자동 정리]
  ↓
[프로그램 종료]
```

### 워크플로우 3: 사용자 정의
```matlab
% 1. 초기화
sim = SimulationManager('verbose', true);
sim = sim.initialize();

% 2. 시작
sim = sim.start_simulation();

% 3. 사용자 작업
topics = sim.get_topics();
% ... 맞춤형 로직 ...

% 4. 정리
pm = ProcessManager('verbose', true);
pm = pm.cleanup_all_simulation();
```

---

## 💾 저장 위치

| 파일 | 위치 |
|------|------|
| ROS2Manager | `matlab/modules/ROS2Manager.m` |
| ProcessManager | `matlab/modules/ProcessManager.m` |
| SimulationManager | `matlab/modules/SimulationManager.m` |
| 모듈 문서 | `matlab/modules/README.md` |
| 빠른 테스트 | `matlab/demo/quick_test.m` |
| 비행 데모 | `matlab/demo/circle_flight_demo.m` |
| 통합 메인 | `matlab/main/integrated_main.m` |

---

## 🎓 학습 순서

**입문자**
1. `demo/quick_test.m` 실행 (5분)
2. `main/integrated_main.m` 실행 (15분)
3. 모듈 README 읽기

**사용자**
1. 메인 스크립트 복사 및 수정
2. 필요한 매니저 조합

**개발자**
1. 각 모듈 내부 학습
2. 커스텀 확장

---

## ❓ 자주 묻는 질문

**Q: 기존 스크립트는 여전히 사용 가능한가?**
```
A: 예, 기존 파일들은 모두 유지됩니다.
   verification/, control/ 폴더의 스크립트도 그대로 실행 가능합니다.
```

**Q: 새로운 방식과 기존 방식 중 어떤 것을 사용해야 하나?**
```
A: 새로운 방식 권장:
   - 더 깔끔한 코드
   - 자동 정리 기능
   - 모듈화로 재사용성 높음
   - 유지보수 용이
```

**Q: 시뮬레이션이 자동 실행되지 않으면?**
```
A: launch_plane.sh 확인:
   1. simulation/launch/launch_plane.sh 파일 존재?
   2. 실행 권한 확인: chmod +x launch_plane.sh
   3. 수동 실행 시도: bash launch_plane.sh
```

**Q: 네임스페이스가 /ap가 아닌데?**
```
A: 자동 감지됨:
   - SimulationManager가 자동으로 감지 (/plane1 등)
   - sim.get_namespace()로 확인
   - 모든 토픽이 자동으로 올바른 경로 사용
```

---

## 📞 지원

각 모듈은 다음과 같이 도움을 받을 수 있습니다:

```matlab
% 메서드 도움
help ROS2Manager.get_topic
help ProcessManager.cleanup_all_simulation
help SimulationManager.start_simulation

% 클래스 도움
help ROS2Manager
help ProcessManager
help SimulationManager
```

---

**🎉 구조 재정리 완료! 깔끔하고 확장 가능한 아키텍처를 구성했습니다.**

다음 단계: 데모를 실행해보세요!
