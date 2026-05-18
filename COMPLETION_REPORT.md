# 🎯 OBDM 프로젝트 - 최종 설정 완료 보고

## 📊 프로젝트 개요

**On-Demand Battery Management (OBDM)** 시뮬레이션 및 MATLAB 통합 환경

### 핵심 목표
고정익 항공기의 최적 경로 결정:
- 🔌 **충전 스테이션** 위치
- 🔋 **항공기 상태** (배터리, 속도, 고도)
- 🌤️ **지역 날씨** (바람, 온도, 기압)

---

## ✅ Phase 1-4 완성 (기반 환경 구성)

### 1️⃣ 시뮬레이션 환경 (simulation/)

**구성 요소**:
- Gazebo Harmonic (물리 시뮬레이션)
- ArduPilot SITL (비행 제어)
- ROS2 Humble (미들웨어)

**제공 기능**:
- ✅ 시뮬레이션 실행 스크립트 (`run_simulation.sh`)
- ✅ ROS2 토픽 자동 발행 (5개)
- ✅ 고정익 항공기 모델 설정

**ROS2 토픽** (MATLAB에서 수신):
```
/ap/tf                    → 위치 (GPS) + 자세 (Euler 각)
/ap/imu/sample            → 가속도 + 각속도
/ap/gps/sample            → 위도/경도/고도
/ap/baro/sample           → 기압 (고도 보조)
/ap/battery/state         → 배터리 정보 (V, A, %)
```

### 2️⃣ MATLAB 통합 환경 (matlab/)

**핵심 모듈**:

| 모듈 | 기능 | 상태 |
|------|------|------|
| `SimulationConfig.m` | 전역 설정 관리 | ✅ 완성 |
| `AircraftStateReceiver.m` | ROS2 토픽 구독 | ✅ 완성 |
| `DataLogger.m` | CSV 데이터 로깅 | ✅ 완성 |
| `ChargeStationManager.m` | 충전소 DB 관리 | ✅ 완성 |
| `WeatherDataProcessor.m` | 날씨 분석 | ✅ 완성 |
| `RouteOptimizer.m` | 최적 경로 계획 | ✅ 완성 |

**주요 기능**:
```matlab
% 1. 항공기 상태 실시간 수신
receiver = AircraftStateReceiver('verbose', true);
receiver.subscribe_to_topics();
state = receiver.get_current_state();

% 2. 충전소 관리
charge_mgr = ChargeStationManager();
[nearest_id, distance] = charge_mgr.find_nearest_station(lat, lon);

% 3. 날씨 영향 분석
weather = WeatherDataProcessor().get_weather(lat, lon);
impact = weather.calculate_impact(state, weather);

% 4. 최적 충전소 선택
optimizer = RouteOptimizer();
[best_station, info] = optimizer.find_optimal_charge_station(pos, state, weather);
```

### 3️⃣ 문서 체계 (docs/)

| 문서 | 내용 | 대상 |
|------|------|------|
| `SETUP_GUIDE.md` | 설치 및 실행 가이드 | 개발자 |
| `TECHNICAL_SPEC.md` | 기술 명세 및 데이터 정의 | 엔지니어 |
| `ROADMAP.md` | 구현 현황 및 계획 | 모든 참여자 |

---

## 🔄 데이터 흐름

```
시뮬레이션 (Gazebo)
  ↓ ArduPilot SITL
  ↓ MAVLink 프로토콜
  ↓
ROS2 Topics (DDS 미들웨어)
  ↓ TF, IMU, GPS, Baro, Battery
  ↓
MATLAB Subscriber
  ├─ 항공기 상태 수신
  ├─ 데이터 로깅
  ├─ 의사결정 로직
  └─ 결과 저장
```

---

## 🚀 빠른 시작 (3단계)

### Step 1️⃣: 시뮬레이션 시작 (터미널 1)
```bash
cd simulation
chmod +x run_simulation.sh
./run_simulation.sh
# → 메뉴에서 선택 (1: iris_runway, 4: alti_transition)
```

### Step 2️⃣: MATLAB 환경 준비 (터미널 2)
```matlab
cd matlab
addpath(genpath(pwd))

% 설정 확인
SimulationConfig.print_config()
```

### Step 3️⃣: 항공기 상태 수신 테스트
```matlab
% 기본 예제 실행
run('aircraft_state_receiver/examples/example_receive_state.m')

% → 60초 동안 실시간 항공기 상태 수신 및 로깅
```

---

## 📦 의사결정 알고리즘

### 충전소 선택 점수 계산

$$\text{점수} = 0.25 \times d + 0.25 \times b + 0.15 \times w + 0.15 \times diff + 0.10 \times a + 0.10 \times c$$

**요소**:
- **d** (거리): 근처일수록 좋음
- **b** (배터리): 도착 시 배터리 잔량
- **w** (날씨): 바람/기압/온도 영향
- **diff** (난이도): 착륙 난이도
- **a** (고도차): 고도 차이
- **c** (충전 속도): 빠를수록 좋음

### 날씨 영향 분석

**위험도 계산** (0 ~ 1):
- 바람 속도 (순항 방향 영향)
- 온도 (양력 변화)
- 기압 (공기 밀도)
- 강수 (시계 악화)
- 습도 (극단값만 영향)

**안전 레벨**:
- 위험도 < 0.2: 🟢 **매우 안전**
- 0.2 ~ 0.4: 🟢 **안전**
- 0.4 ~ 0.6: 🟡 **주의**
- 0.6 ~ 0.8: 🔴 **경고**
- > 0.8: 🔴 **위험**

---

## 📁 최종 프로젝트 구조

```
OBDM_ws/
├── README.md                    # 🎯 프로젝트 개요
├── ROADMAP.md                   # 📋 구현 현황
│
├── simulation/                  # 🎮 Gazebo 시뮬레이션
│   ├── README.md               # 시뮬레이션 가이드
│   └── run_simulation.sh        # 실행 스크립트
│
├── matlab/                      # 🔧 MATLAB 제어
│   ├── config/
│   │   └── SimulationConfig.m   # ⚙️ 전역 설정
│   │
│   ├── aircraft_state_receiver/ # 📡 상태 수신
│   │   ├── AircraftStateReceiver.m
│   │   └── examples/
│   │       └── example_receive_state.m
│   │
│   ├── decision_logic/          # 🧠 의사결정
│   │   ├── ChargeStationManager.m
│   │   ├── WeatherDataProcessor.m
│   │   └── RouteOptimizer.m
│   │
│   └── utils/
│       └── DataLogger.m         # 📊 데이터 로깅
│
└── docs/                        # 📚 문서
    ├── SETUP_GUIDE.md           # 설치 가이드
    └── TECHNICAL_SPEC.md        # 기술 명세
```

---

## 🎯 구현 현황

### ✅ 완료된 기능

- [x] 시뮬레이션 환경 설정 (Gazebo + ROS2)
- [x] ROS2 토픽 정의 및 매핑
- [x] MATLAB ROS2 구독 구현
- [x] 항공기 상태 실시간 수신
- [x] 데이터 CSV 로깅
- [x] 충전소 관리 (거리 계산)
- [x] 날씨 영향 분석 (6가지 요소)
- [x] 최적 충전소 선택 알고리즘
- [x] 경로 계획 (충전소 경유)
- [x] 종합 문서화

### 🔜 향후 계획 (Phase 5+)

- [ ] 실시간 시뮬레이션 검증 (데이터 흐름 확인)
- [ ] 기상청 ASOS API 연동
- [ ] 시각화 (3D 경로, 상태 그래프)
- [ ] 양방향 통신 (MATLAB → 시뮬레이션 명령)
- [ ] 성능 최적화 및 파라미터 튜닝

---

## 💡 핵심 기술

### 항공기 모델
- **기본**: Iris (멀티콥터)
- **고정익**: Quadplane (고정익 + 멀티콥터)
- **커스텀**: 추후 고정익 전용 모델 추가 가능

### 좌표계 변환
- GPS (위도/경도/고도) ↔ 지역 좌표계 (NED)
- Quaternion (자세) ↔ Euler 각도 (Roll/Pitch/Yaw)

### 거리 계산
- **Haversine 공식**: GPS 기반 정확한 거리 계산
- **고도 차이**: 3D 거리 고려

### 배터리 모델
```
도달 가능 거리 = (배터리% × 용량) ÷ 소비율 ÷ 순항속도 × 안전마진(0.8)
```

---

## 📊 주요 지표

| 항목 | 값 |
|------|-----|
| **ROS2 토픽** | 5개 (TF, IMU, GPS, Baro, Battery) |
| **샘플링 레이트** | 2 ~ 50 Hz |
| **의사결정 요소** | 6개 (거리, 배터리, 날씨, 난이도, 고도, 충전 속도) |
| **충전소 점수** | 0 ~ 1 (1이 최적) |
| **날씨 위험도** | 0 ~ 1 (0이 안전) |
| **배터리 마진** | 15% |

---

## 🔐 에러 처리

**잠재적 문제와 대응**:

| 문제 | 원인 | 해결책 |
|------|------|---------|
| ROS2 토픽 없음 | 시뮬레이션 미실행 | `ros2 topic list` 확인 |
| MATLAB 구독 실패 | 메시지 타입 불일치 | `ros2 msg show` 확인 |
| 데이터 부재 | 네트워크 문제 | ROS_DOMAIN_ID 확인 |
| 경로 계획 실패 | 충전소 없음 | 충전소 추가 또는 검색 반경 확대 |

---

## 📖 사용 예제

### 예제 1: 기본 상태 수신
```matlab
% 1. 수신기 생성 및 시작
receiver = AircraftStateReceiver('verbose', true);
receiver.subscribe_to_topics();

% 2. 상태 조회 (1초마다)
for i = 1:60
    state = receiver.get_current_state();
    fprintf('고도: %.1f m, 배터리: %.1f%%\n', ...
        state.position.altitude, state.battery.percent);
    pause(1);
end

% 3. 상태 출력
receiver.print_state();
```

### 예제 2: 최적 충전소 결정
```matlab
% 1. 매니저 생성
charge_mgr = ChargeStationManager('verbose', true);
weather_proc = WeatherDataProcessor();
optimizer = RouteOptimizer('charge_manager', charge_mgr);

% 2. 현재 상태와 날씨 가져오기
state = receiver.get_current_state();
weather = weather_proc.get_weather(37.5665, 126.9780);

% 3. 최적 충전소 결정
[best_id, info] = optimizer.find_optimal_charge_station(...
    [state.position.latitude, state.position.longitude, state.position.altitude], ...
    state, weather);

% 4. 결과 출력
best_station = charge_mgr.get_station_info(best_id);
fprintf('선택된 충전소: %s (점수: %.3f)\n', ...
    best_station.name, info.best_station.total_score);
```

---

## 🎓 학습 자료

### 필수 개념
- [ROS2 DDS 통신](https://docs.ros.org/en/humble/)
- [고정익 항공기 항법](https://ardupilot.org/plane/)
- [MAVLink 프로토콜](https://mavlink.io/)
- [GPS 좌표 변환](https://en.wikipedia.org/wiki/Geographic_coordinate_system)

### 기상 데이터
- [기상청 ASOS API 가이드](https://data.kma.go.kr/api/selectApiDetail.do?pgmNo=42&openApiNo=241)
- [항공기 성능 영향 요인](https://en.wikipedia.org/wiki/Aircraft_flight_mechanics)

---

## 📞 지원 및 다음 단계

### 즉시 시작 가능
```bash
# 1. 시뮬레이션 실행
cd ~/OBDM_ws/simulation && ./run_simulation.sh

# 2. MATLAB 실행
cd ~/OBDM_ws/matlab && matlab -r "run('aircraft_state_receiver/examples/example_receive_state.m')"
```

### 추가 정보
- 설치 가이드: [docs/SETUP_GUIDE.md](../docs/SETUP_GUIDE.md)
- 기술 명세: [docs/TECHNICAL_SPEC.md](../docs/TECHNICAL_SPEC.md)
- 로드맵: [ROADMAP.md](../ROADMAP.md)

---

## 🎉 결론

**OBDM 프로젝트 기반 환경 설정 완료!**

✅ 시뮬레이션 환경 준비 완료  
✅ MATLAB 통합 모듈 구현 완료  
✅ 의사결정 알고리즘 설계 완료  
✅ 종합 문서화 완료  

**다음 단계**: Phase 5-6 (실시간 검증 및 기상청 API 연동)

---

*OBDM Project - 2024*
