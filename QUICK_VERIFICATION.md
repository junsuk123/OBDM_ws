# 🚀 빠른 검증 시작 가이드

OBDM 시뮬레이션과 MATLAB 통합 환경을 검증하는 가장 빠른 방법

---

## ⚡ 3분 안에 검증하기

### 준비 사항
- Gazebo + ArduPilot 설치 완료
- MATLAB Robotics System Toolbox 설치
- 2개 이상의 터미널 창

---

## 🎬 실행 단계

### 1️⃣ 터미널 1: 시뮬레이션 시작 (1분)

```bash
cd ~/OBDM_ws/simulation
./run_simulation.sh
```

**메뉴 선택**:
```
사용 가능한 시뮬레이션:
1. iris_runway      - 멀티콥터 (Iris) 예제
...
기본값: 1
```

**선택**: `1` 입력 후 Enter

**기대 결과** ✅:
- Gazebo 윈도우 열림
- 항공기 시뮬레이션 시작
- 콘솔에 메시지 출력

---

### 2️⃣ 터미널 2: ROS2 토픽 확인 (1분)

```bash
cd ~/OBDM_ws/simulation/verification
./check_topics.sh
```

**출력 예**:
```
✓ ROS2 설치 확인
✓ ROS_DOMAIN_ID 설정: 0
✓ 다음 토픽이 활성화되어 있습니다:
  • /ap/tf
  • /ap/imu/sample
  • /ap/gps/sample
  • /ap/baro/sample
  • /ap/battery/state

✓ 모든 필수 토픽이 활성화되어 있습니다
```

---

### 3️⃣ 터미널 3: MATLAB ROS2 연결 검증 (1분)

```matlab
cd ~/OBDM_ws/matlab
addpath(genpath(pwd))
run('verification/verify_ros2_connection.m')
```

**실행 완료 후 나타나는 메시지**:
```
🟢 성공! 시뮬레이션과 MATLAB이 정상적으로 연동되고 있습니다.

유효한 데이터: 15/15 (100%)

🟢 매우 우수한 데이터 품질
종합 점수: A
```

---

## ✅ 검증 완료!

모든 3단계가 성공하면:
```
✓ 시뮬레이션 정상 작동
✓ ROS2 토픽 활성화
✓ MATLAB 수신 성공
✓ 데이터 품질 A등급
```

---

## 📊 데이터 품질 상세 분석 (선택)

```matlab
cd ~/OBDM_ws/matlab
run('verification/verify_data_quality.m')
```

**출력**:
```
수집 기간: ~1분
샘플 수: 30개

위도/경도: 정상 범위
고도: 95~106 m (합리적)
속도: 0~22.5 m/s (순항 속도)
배터리: 74.5~75% (정상 소모율)

데이터 완전성: 100%
이상치: 0개

🟢 매우 우수한 데이터 품질
```

---

## 🔴 문제 발생시

### 토픽을 찾을 수 없음
```bash
# 확인
ros2 topic list | grep ap

# 해결: 시뮬레이션이 실행 중인지 확인
ps aux | grep ardupilot
```

### MATLAB 구독 실패
```matlab
% 확인
ros2('gettimesync')  % ROS2 연결 테스트

% 필요시 재시작
clear all
addpath(genpath(pwd))
```

### 데이터 수신 안 됨
```matlab
% 명시적 테스트
receiver = AircraftStateReceiver('verbose', true);
receiver.subscribe_to_topics();

% 10초 대기
pause(10);

% 상태 조회
state = receiver.get_current_state();
fprintf('유효: %d\n', state.is_valid);
```

---

## 📋 체크리스트

검증 후 확인:

- [ ] 시뮬레이션 윈도우 열림?
- [ ] 5개 토픽 모두 활성화?
- [ ] MATLAB 수신 성공 (100%)?
- [ ] 데이터 품질 B등급 이상?

---

## 🎯 검증 의미

| 검증 단계 | 의미 | 결과 |
|---------|------|------|
| 시뮬레이션 | Gazebo 정상 작동 | ✅ |
| ROS2 토픽 | ArduPilot이 데이터 발행 | ✅ |
| MATLAB 연결 | MATLAB이 데이터 수신 | ✅ |
| 데이터 품질 | 신뢰할 수 있는 데이터 | ✅ |

**결론**: 🟢 시뮬레이션 ↔ MATLAB 통합 환경 완벽 검증!

---

## 🚀 다음 단계

검증 성공 후:

1. **의사결정 로직 테스트**
```matlab
charge_mgr = ChargeStationManager('verbose', true);
weather = WeatherDataProcessor();
optimizer = RouteOptimizer();
```

2. **기상청 API 연동** (Phase 6)
```
기상청 ASOS API 키 등록
실시간 날씨 데이터 수신
```

3. **전체 워크플로우 테스트** (Phase 7)
```
시뮬레이션 → ROS2 → MATLAB → 의사결정 → 결과 저장
```

---

## 📞 도움말

### 빠른 문제 해결
```bash
# ROS2 환경 재설정
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

# MATLAB 경로 재설정
cd ~/OBDM_ws/matlab
addpath(genpath(pwd))
```

### 로그 파일 확인
```bash
# 최신 로그 보기
ls -ltr ~/OBDM_ws/matlab/logs/ | tail -1
cat ~/OBDM_ws/matlab/logs/obdm_flight_log_*.csv | head -10
```

---

**⏱️ 총 소요 시간: 약 5분 (모든 검증 포함)**

**검증 완료 후 Phase 6 진행 가능! 🎉**
