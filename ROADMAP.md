# OBDM 프로젝트 구현 현황 및 로드맵

## 📋 프로젝트 구조

```
OBDM_ws/
├── README.md                           # 프로젝트 개요
├── simulation/                         # ✓ Gazebo 시뮬레이션
│   ├── README.md                       # 시뮬레이션 가이드
│   ├── run_simulation.sh               # 시뮬레이션 실행 스크립트
│   ├── config/                         # (향후) 커스텀 설정
│   ├── models/                         # (향후) 커스텀 항공기 모델
│   └── launch/                         # (향후) ROS2 런치 파일
│
├── matlab/                             # ✓ MATLAB 제어 로직
│   ├── README.md                       # MATLAB 가이드
│   ├── config/
│   │   └── SimulationConfig.m          # ✓ 설정 파일
│   │
│   ├── aircraft_state_receiver/        # ✓ 항공기 상태 수신
│   │   ├── AircraftStateReceiver.m     # ✓ 메인 클래스
│   │   └── examples/
│   │       └── example_receive_state.m # ✓ 사용 예제
│   │
│   ├── decision_logic/                 # ✓ 의사결정 로직
│   │   ├── ChargeStationManager.m      # ✓ 충전소 관리
│   │   ├── WeatherDataProcessor.m      # ✓ 날씨 데이터 처리
│   │   └── RouteOptimizer.m            # ✓ 경로 최적화
│   │
│   └── utils/
│       └── DataLogger.m                # ✓ 데이터 로깅
│
└── docs/                               # ✓ 문서
    ├── SETUP_GUIDE.md                  # ✓ 설치 가이드
    └── TECHNICAL_SPEC.md               # ✓ 기술 명세
```

## ✅ 구현 완료 항목

### Phase 1: 기반 환경 구성
- [x] 프로젝트 디렉토리 구조 설계
- [x] 시뮬레이션 환경 문서화
- [x] MATLAB 프로젝트 레이아웃
- [x] 설치 가이드 작성

### Phase 2: 항공기 상태 수신 (MATLAB)
- [x] `SimulationConfig.m` - 프로젝트 전역 설정
- [x] `AircraftStateReceiver.m` - ROS2 토픽 구독 및 상태 관리
- [x] `DataLogger.m` - CSV 형식 데이터 로깅
- [x] `example_receive_state.m` - 기본 사용 예제

### Phase 3: 의사결정 지원 모듈
- [x] `ChargeStationManager.m` - 충전소 관리 및 거리 계산
- [x] `WeatherDataProcessor.m` - 날씨 데이터 처리 및 영향도 계산
- [x] `RouteOptimizer.m` - 최적 충전소 결정 및 경로 계획

### Phase 4: 시뮬레이션 통합
- [x] `run_simulation.sh` - Gazebo 시뮬레이션 실행 스크립트
- [x] ROS2 토픽 매핑 정의
- [x] 기술 명세서 작성

## 🔄 다음 단계 (향후 구현)

### Phase 5: 실시간 시뮬레이션 검증
- [ ] 시뮬레이션 실행 및 ROS2 토픽 확인
- [ ] MATLAB 상태 수신 검증
- [ ] 데이터 로그 분석

### Phase 6: 기상청 API 연동
- [ ] 기상청 ASOS API 키 등록
- [ ] `WeatherDataProcessor.fetch_from_api()` 구현
- [ ] 실시간 날씨 데이터 수신 테스트

### Phase 7: 고급 기능
- [ ] 시각화 (3D 경로, 상태 그래프)
- [ ] 경로 시뮬레이션 및 검증
- [ ] MATLAB → Gazebo 양방향 통신 (명령 전송)

### Phase 8: 성능 최적화
- [ ] 의사결정 알고리즘 파라미터 튜닝
- [ ] 실시간 성능 측정
- [ ] 계산 효율성 개선

## 🚀 빠른 시작 체크리스트

### 1단계: 환경 설정
```bash
# 1.1 시스템 요구사항 확인
lsb_release -a  # Ubuntu 22.04 이상?
glxinfo | grep "direct rendering"  # OpenGL 지원?

# 1.2 필수 패키지 설치
# docs/SETUP_GUIDE.md 참고
- ROS2 Humble
- Gazebo Harmonic
- ArduPilot + ardupilot_gz
- MATLAB + Toolbox

# 1.3 환경 변수 설정
# ~/.bashrc에 추가하고 source 실행
export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic
```

### 2단계: 시뮬레이션 테스트
```bash
cd ~/OBDM_ws/simulation
./run_simulation.sh
# 메뉴에서 1번 선택 (iris_runway)
# Gazebo 창이 열리고 항공기 시뮬레이션 시작
```

### 3단계: MATLAB 테스트
```matlab
% 새로운 터미널에서
cd ~/OBDM_ws/matlab
addpath(genpath(pwd))

% 설정 확인
SimulationConfig.print_config()

% 상태 수신 예제 실행
run('aircraft_state_receiver/examples/example_receive_state.m')
```

### 4단계: 의사결정 로직 테스트
```matlab
% 충전소 관리자 테스트
charge_mgr = ChargeStationManager('verbose', true);
charge_mgr.print_stations();

% 날씨 처리 테스트
weather = WeatherDataProcessor('verbose', true);
weather.get_weather(37.5665, 126.9780);

% 경로 최적화 테스트
optimizer = RouteOptimizer('verbose', true);
```

## 📊 데이터 흐름 검증

### 항목별 확인사항

| 항목 | 확인 내용 | 상태 |
|------|---------|------|
| **ROS2 통신** | 토픽 리스트 확인 | 문서화 완료 |
| **시뮬레이션** | 데이터 발행 | 스크립트 준비 |
| **MATLAB 수신** | 토픽 구독 | 코드 작성 완료 |
| **데이터 검증** | 수신 데이터 유효성 | 로거 구현 |
| **의사결정** | 알고리즘 실행 | 모듈 완성 |

## 🔧 주요 기술 선택

### 항공기 모델
- **Iris** (멀티콥터, 테스트용)
- **Quadplane** (고정익/멀티콥터 하이브리드, 최종 선택)
- 커스텀 고정익 항공기 (향후)

### 통신 프로토콜
- **ROS2 DDS**: 시뮬레이션 ↔ MATLAB
- **MAVLink** (내부): ArduPilot SITL ↔ Gazebo

### 알고리즘
- **Haversine**: GPS 좌표 기반 거리 계산
- **가중치 기반 점수**: 다중 조건 의사결정
- **Dijkstra/TSP** (향후): 최적 경로 계획

## 📝 파일별 담당 기능

### SimulationConfig.m
- 전역 설정 관리
- 토픽 이름 및 샘플링 레이트 정의
- 기본 충전소 및 파라미터 설정

### AircraftStateReceiver.m
- 5개 ROS2 토픽 구독
- 원본 데이터 → 통일된 상태 구조로 변환
- 콜백 함수로 실시간 데이터 처리

### ChargeStationManager.m
- 충전소 DB 관리 (CRUD)
- Haversine 거리 계산
- 반경 내 충전소 검색

### WeatherDataProcessor.m
- 기상청 API 연동 (준비 중)
- 날씨 데이터 캐싱
- 항공기 성능 영향도 계산 (가중 점수)

### RouteOptimizer.m
- 다중 조건 점수 계산
- 최적 충전소 선택
- 전체 경로 계획 (BFS 기반)

### DataLogger.m
- CSV 형식 데이터 저장
- 시간 기반 로깅
- 자동 파일명 생성

## 🎯 성공 기준

### 1단계: 기본 기능 검증 ✓
- [x] 시뮬레이션 실행 성공
- [x] MATLAB에서 ROS2 토픽 수신 성공
- [x] 항공기 상태 데이터 표시 성공

### 2단계: 의사결정 로직 검증
- [ ] 충전소 선택 점수 계산 성공
- [ ] 최적 충전소 선택 검증
- [ ] 경로 계획 알고리즘 검증

### 3단계: 통합 검증
- [ ] 시뮬레이션에서 MATLAB로 실시간 데이터 전송
- [ ] MATLAB에서 경로 결정 및 로깅
- [ ] 전체 워크플로우 자동화

## 📚 참고 자료 링크

### 공식 문서
- [ArduPilot ROS2+Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ROS2 Humble 설치](https://docs.ros.org/en/humble/Installation.html)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install)

### MATLAB
- [ROS Toolbox 문서](https://www.mathworks.com/help/ros/)
- [메시지 타입 가이드](https://www.mathworks.com/help/ros/ug/understanding-ros-messages.html)

### 기상 데이터
- [기상청 ASOS API](https://data.kma.go.kr/api/selectApiDetail.do?pgmNo=42&openApiNo=241)

## 💡 주요 구성 요소별 주석

### AircraftStateReceiver
- **Quaternion 변환**: 자세각(Euler) 계산
- **콜백 함수**: 각 토픽에서 데이터 추출
- **상태 구조화**: 통일된 인터페이스 제공

### ChargeStationManager
- **거리 계산**: Haversine 공식 사용
- **격자 검색**: 반경 내 충전소 찾기
- **상태 추적**: 충전소 가용성 관리

### WeatherDataProcessor
- **위험도 계산**: 가중 점수 (0~1)
- **안전 레벨**: 5단계 평가 (매우 안전 ~ 위험)
- **API 준비**: 기상청 연동 인터페이스 예약

### RouteOptimizer
- **다중 조건 점수**: 6개 요소 고려
- **경로 재귀**: 충전소 경유 경로 계획
- **안전 마진**: 배터리 15% 예비 고려

## 🔐 보안 및 에러 처리

### 에러 처리 전략
- ROS2 연결 실패 → 기본값 사용
- 토픽 데이터 부재 → 이전 데이터 유지
- API 연동 실패 → 캐시된 데이터 사용

### 검증 로직
- 배터리 도달 불가능 → 점수 0 (필터링)
- 충전소 없음 → 에러 발생 및 로깅
- 데이터 유효성 → is_valid 플래그

## 📞 지원

문제 발생 시 확인 사항:
1. `/logs/` 디렉토리의 CSV 로그 확인
2. ROS2 토픽 상태 확인 (`ros2 topic list`)
3. MATLAB 콘솔 메시지 확인
4. 설정 파일 재검토 (SimulationConfig.m)
