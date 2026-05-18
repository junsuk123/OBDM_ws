# 실시간 시뮬레이션 검증 가이드

OBDM 프로젝트의 시뮬레이션과 MATLAB 통합 환경을 검증하는 완벽한 가이드

## 📋 개요

이 가이드는 다음을 검증합니다:
1. **시뮬레이션 환경**: Gazebo + ArduPilot SITL이 정상 작동하는가?
2. **ROS2 토픽**: 시뮬레이션이 올바른 데이터를 발행하는가?
3. **MATLAB 수신**: MATLAB이 ROS2 토픽을 구독하는가?
4. **데이터 품질**: 수신된 데이터가 신뢰할 수 있는가?

---

## 🚀 검증 프로세스 (4단계)

### Step 1: 시뮬레이션 실행

**터미널 1 - 시뮬레이션 시작**:
```bash
cd ~/OBDM_ws/simulation
./run_simulation.sh

# 메뉴 선택
# 1 - iris_runway (기본, 권장)
# 4 - alti_transition_runway (고정익/하위 호환)
```

**예상 결과**:
- ✅ Gazebo 윈도우 열림
- ✅ 쿼드콥터 시뮬레이션 시작 (비행 중)
- ✅ 콘솔에 ArduPilot 메시지 출력

---

### Step 2: ROS2 토픽 확인

**터미널 2 - ROS2 토픽 검증**:
```bash
cd ~/OBDM_ws/simulation/verification
chmod +x check_topics.sh
./check_topics.sh
```

**검사 항목**:
1. ✅ ROS2 설치 확인
2. ✅ ROS_DOMAIN_ID 설정
3. ✅ 활성 토픽 확인
4. ✅ 토픽 데이터 샘플 수집

**예상 결과**:
```
✓ ROS2 설치 확인
✓ ROS_DOMAIN_ID 설정: 0
✓ 다음 토픽이 활성화되어 있습니다:
  • /ap/tf
  • /ap/imu/sample
  • /ap/gps/sample
  • /ap/baro/sample
  • /ap/battery/state
```

**토픽별 확인**:
| 토픽 | 설명 | 예상 데이터 |
|------|------|----------|
| `/ap/tf` | 위치/자세 | TransformStamped |
| `/ap/imu/sample` | IMU 센서 | 가속도, 각속도 |
| `/ap/gps/sample` | GPS | 위도, 경도, 고도 |
| `/ap/baro/sample` | 기압계 | 기압 값 |
| `/ap/battery/state` | 배터리 | 전압, 전류, % |

---

### Step 3: MATLAB ROS2 연결 검증

**터미널 3 - MATLAB 실행**:
```matlab
cd ~/OBDM_ws/matlab
addpath(genpath(pwd))
run('verification/verify_ros2_connection.m')
```

**실행 흐름**:
```
Phase 1: 환경 설정
  ✓ 프로젝트 경로 추가
  ✓ 설정 출력
  ✓ ROS2 환경 검증

Phase 2: 토픽 확인
  (기대되는 5개 토픽 나열)

Phase 3: 상태 수신기 생성
  ✓ AircraftStateReceiver 객체 생성
  ✓ ROS2 토픽 구독

Phase 4: 데이터 수신 (30초)
  [01] 시간: HH:MM:SS
       위치: LAT=37.5665°, LON=126.9780°, ALT=100.0 m
       속도: Vx=0.50, Vy=-0.30, Vz=0.10 m/s
       자세: Roll=0.1°, Pitch=2.5°, Yaw=180.0°
       배터리: 75.0% (12.00 V, 0.50 A)
       상태: ✓ 유효

Phase 5: 데이터 품질 분석
  유효한 데이터: 15/15 (100%)

Phase 6: 토픽별 수신 상태
  ✓ 위치/자세 (TF)
  ✓ IMU 센서
  ✓ GPS
  ✓ 기압계
  ✓ 배터리

Phase 7: 최종 항공기 상태
  (상세 상태 정보 출력)
```

**데이터 범위 분석**:
- 위도: 정상 범위 내 (한반도)
- 경도: 정상 범위 내 (한반도)
- 고도: 50-500 m (합리적)
- 속도: 0-15 m/s (쿼드콥터 이동/호버 속도)
- 배터리: 0-100% (정상)

**최종 결과**:
```
🟢 성공! 시뮬레이션과 MATLAB이 정상적으로 연동되고 있습니다.

다음 단계:
  1. 의사결정 로직 테스트
  2. 날씨 데이터 API 연동
  3. 전체 워크플로우 검증
```

---

### Step 4: 데이터 품질 검증

**MATLAB - 로그 데이터 분석**:
```matlab
% 약 30초 후 verify_ros2_connection.m 완료
cd matlab
run('verification/verify_data_quality.m')
```

**분석 내용**:

#### 4.1 기본 통계
```
수집 기간:
  시작: 14:23:45
  종료: 14:24:15
  수집 시간: ~1분
  샘플 수: 30
```

#### 4.2 위치 데이터
```
위도 (LAT):
  범위: 37.564500 ~ 37.567200
  평균: 37.565800
  표준편차: 0.000800

경도 (LON):
  범위: 126.977800 ~ 126.980500
  평균: 126.979200
  표준편차: 0.000600

고도 (ALT):
  범위: 95.3 ~ 105.7 m
  평균: 100.5 m
  표준편차: 2.1 m
  변화율: 0.35 m/분
```

#### 4.3 속도 데이터
```
속력 (Speed):
  범위: 0.15 ~ 22.50 m/s
  평균: 18.75 m/s
  표준편차: 4.20 m/s

✓ 고정익 항공기 순항 속도 범위 내
```

#### 4.4 자세각 데이터
```
Roll:  -5.2 ~ 3.1°
Pitch: -1.0 ~ 4.5°
Yaw:   178.5 ~ 181.2°

✓ 정상적인 자세각 범위
```

#### 4.5 배터리 데이터
```
배터리 잔량:
  범위: 74.5 ~ 75.0%
  평균: 74.8%
  변화: -0.5%
  소모율: 0.30%/분

✓ 정상적인 배터리 소모율
```

#### 4.6 데이터 품질 평가
```
이상치 분석:
  고도 이상치: 0개 (0%)

위치 연속성:
  평균 위치 변화: 0.015 km
  최대 위치 변화: 0.025 km

데이터 완전성:
  완전한 행: 30/30 (100%)

종합 평가:
  🟢 매우 우수한 데이터 품질
  종합 점수: A
```

---

## 🔍 문제 해결

### 문제 1: ROS2 토픽을 찾을 수 없음

**증상**:
```
✗ ArduPilot 토픽을 찾을 수 없습니다
```

**원인 및 해결**:

| 원인 | 확인 방법 | 해결책 |
|------|---------|--------|
| 시뮬레이션 미실행 | 다른 터미널에서 `ros2 topic list` 실행 | 시뮬레이션 시작 |
| ROS_DOMAIN_ID 불일치 | `echo $ROS_DOMAIN_ID` | 동일하게 설정 (보통 0) |
| 방화벽 차단 | 포트 7400-7409 확인 | 방화벽 규칙 추가 |
| ROS2 경로 미설정 | `source /opt/ros/humble/setup.bash` | 경로 설정 |

**확인 명령**:
```bash
# 1. 활성 토픽 확인
ros2 topic list

# 2. 특정 토픽 구독
ros2 topic echo /ap/tf -c 1

# 3. 네트워크 상태
ifconfig | grep inet

# 4. 방화벽 상태
sudo ufw status
```

---

### 문제 2: MATLAB에서 ROS2 연결 실패

**증상**:
```
✗ 구독 실패
```

**원인 및 해결**:

1. **ROS2 메시지 타입 불일치**
```matlab
% 확인: MATLAB 콘솔에서
ros2 msg list | grep sensor_msgs
ros2 msg show sensor_msgs/msg/Imu
```

2. **MATLAB ROS2 라이브러리 미설치**
```matlab
% Robotics System Toolbox 설치 확인
ver % 모든 설치된 toolbox 표시
```

3. **ROS_DOMAIN_ID 환경 변수**
```matlab
% MATLAB에서 확인
getenv('ROS_DOMAIN_ID')

% 필요시 설정 (MATLAB 시작 전)
setenv('ROS_DOMAIN_ID', '0')
```

---

### 문제 3: 데이터 품질 저하

**증상**:
```
데이터 완전성: 60/100 (60%)
🔴 낮은 데이터 품질
```

**원인 및 해결**:

1. **네트워크 지연**
   - 원인: 시스템 과부하
   - 해결: 다른 프로세스 종료

2. **시뮬레이션 불안정**
   - 원인: Gazebo 그래픽 문제
   - 해결: 그래픽 드라이버 업데이트

3. **ROS2 메시지 큐 오버플로우**
   - 원인: 구독자가 메시지 처리 불가
   - 해결: 샘플링 레이트 감소

---

## ✅ 검증 체크리스트

검증 후 다음을 확인하세요:

### 시뮬레이션 측
- [ ] Gazebo 윈도우가 열렸는가?
- [ ] 항공기가 시뮬레이션 내에서 비행하는가?
- [ ] 콘솔에 ArduPilot 메시지가 출력되는가?

### ROS2 토픽 측
- [ ] 5개의 기대되는 토픽이 모두 활성화되어 있는가?
- [ ] 각 토픽에서 데이터를 수신할 수 있는가?
- [ ] 데이터 형식이 올바른가?

### MATLAB 측
- [ ] `verify_ros2_connection.m` 실행 성공?
- [ ] 30초 동안 데이터 수신 성공?
- [ ] 모든 5개 토픽에서 데이터 수신?

### 데이터 품질 측
- [ ] 데이터 완전성 > 90%?
- [ ] 이상치 < 2개?
- [ ] 종합 점수 ≥ B?

---

## 📊 예상 성능 지표

### 토픽 발행 주기
| 토픽 | 예상 주기 | 실제 주기 |
|------|---------|---------|
| `/ap/tf` | 10 Hz | - |
| `/ap/imu/sample` | 50 Hz | - |
| `/ap/gps/sample` | 10 Hz | - |
| `/ap/baro/sample` | 10 Hz | - |
| `/ap/battery/state` | 2 Hz | - |

### 데이터 범위 (실제 항공기 시뮬레이션)
| 데이터 | 기대 범위 | 실제 범위 |
|-------|---------|---------|
| 속도 | 15-25 m/s | - |
| 고도 | 50-500 m | - |
| 배터리 | 0-100% | - |
| 자세각 | ±180° | - |

---

## 🎯 다음 단계

### 검증 성공 후
1. ✅ **의사결정 로직 테스트**
```matlab
% ChargeStationManager 테스트
charge_mgr = ChargeStationManager('verbose', true);
charge_mgr.print_stations();

% WeatherDataProcessor 테스트
weather = WeatherDataProcessor();
weather_data = weather.get_weather(37.5665, 126.9780);

% RouteOptimizer 테스트
optimizer = RouteOptimizer();
```

2. ✅ **기상청 API 연동**
```matlab
% 기상청 ASOS API 키 설정
weather = WeatherDataProcessor('api_key', 'YOUR_API_KEY');
real_weather = weather.fetch_from_api(37.5665, 126.9780);
```

3. ✅ **전체 워크플로우 테스트**
```
시뮬레이션 → ROS2 → MATLAB → 의사결정 → 로깅
```

---

## 📞 지원

### 로그 파일 확인
```bash
# CSV 로그 확인
cat logs/obdm_flight_log_*.csv | head -20
```

### 트러블슈팅
```bash
# ROS2 환경 변수 확인
echo $ROS_DOMAIN_ID
echo $GZ_VERSION

# ROS2 노드 확인
ros2 node list

# 네트워크 상태 확인
ros2 doctor
```

### 추가 자료
- [ArduPilot 로그 분석](https://ardupilot.org/dev/docs/data-flash-log-wiki.html)
- [ROS2 DDS 설정](https://docs.ros.org/en/humble/Guides/DDS-and-ROS-middleware-stack.html)
- [MATLAB ROS 통신](https://www.mathworks.com/help/ros/)

---

## 📝 검증 기록

검증 결과를 기록하세요:

```
검증 날짜: 2026-04-30
검증자: [이름]

Step 1 - 시뮬레이션: ✓ 성공
Step 2 - ROS2 토픽: ✓ 성공
Step 3 - MATLAB 연결: ✓ 성공
Step 4 - 데이터 품질: ✓ A등급

종합 판정: ✓ 검증 완료
```

---

**검증 완료 후 Phase 6 진행: 기상청 API 연동**
