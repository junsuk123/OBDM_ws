**개요**
- **목표**: 코드베이스를 모듈화하여 각 책임(데이터 수집, 저장, 시각화, 제어, 경로생성 등)을 분리합니다.
- **원칙**: 각 모듈은 독립 실행 가능하고, 명확한 초기화/종료 API를 가지며 다른 모듈에 의존할 때는 인터페이스를 통해 호출합니다.

**모듈 분류(권장)**
- Data Interface: ROS/Gazebo 통신을 담당 (`DataInterface`)
- Data Logger: 모든 데이터를 영구 저장 (`DataLogger`)
- Visualizer: 실시간 시각화 (`RealtimeVisualizer`)
- Controller: 추종/제어 알고리즘 (`Controller`)
- Planner: 경로 생성 및 계획 (`Planner`)
- Utils/Shared: 공통 유틸리티, 구성(`SimulationConfig` 등)

**모듈 책임과 인터페이스**
- 공통 라이프사이클 메서드: `init(config)`, `start()`, `stop()`, `cleanup()`
- 의존성 선언: 각 모듈은 `requires()` 또는 `dependencies`로 필요한 모듈/토픽을 명시
- 통신: ROS 구독/발행은 모듈 내부에서 캡슐화하고, 메시지 파싱은 모듈 경계에서 처리

예: `DataInterface` 역할
- `init(config)`: ROS2 네임스페이스, QoS 설정, 토픽 매핑
- `subscribe(topic, callback)`, `publish(topic, msg)` 래퍼 제공
- 센서 메시지를 표준화된 내부 포맷(예: `struct` 또는 `timetable`)으로 변환하여 제공

예: `DataLogger` 역할
- `init(config)`: 로그 디렉토리/파일 준비
- `log_trajectory(row)`, `log_wind(row)`, `flush()` 등의 API
- 비동기/버퍼링 전략 권장 (배치/주기 저장)

**통신 규칙 및 QoS**
- 센서 스트림(예: IMU, Airspeed): Best-effort 또는 적절한 큐 사이즈
- 중요한 제어/명령: Reliable, 적절한 큐 설정
- 토픽 네이밍: `/<namespace>/<sensor>` 패턴을 통일

**파일/폴더 구조 권장**
- `matlab/modules/` : 모듈별 클래스/파일
- `matlab/main/` : 통합 실행 스크립트(예: `integrated_main.m`)
- `matlab/config/` : `SimulationConfig.m`, 토픽 우선순위 목록
- `docs/` : 개발 가이드 및 아키텍처 문서

**개발 및 확장 지침**
- 새 모듈 생성 시 `ModuleBase` 상속(또는 템플릿 복사)하고, `init/start/stop/cleanup`을 구현
- 외부 의존성이 필요한 경우 `SimulationConfig`를 통해 토픽/네임스페이스 조회
- 유닛 테스트는 MATLAB 스크립트로 작성, 가능한 모듈별로 독립 실행 확인
- 배치 실행: GUI/타이머를 조건문(`usejava('desktop')`)으로 분기

**예시 작업 흐름**
1. `DataInterface.init(config)` → 토픽 구독 준비
2. `DataLogger.init(config)` → 로그 파일 준비
3. `Visualizer.init(config)` (선택적)
4. `Controller.init(config)` 및 `Planner.init(config)`
5. `DataInterface.start()` → 실시간 메시지 수신
6. 수신 콜백에서 `DataLogger.log_*()` 및 컨트롤러에 데이터 전달
7. 작업 종료 시 `stop()` → `cleanup()` 순서로 리소스 해제

**디버깅 체크리스트**
- 토픽이 실제 런타임 overlay(`/home/j/ardu_ws`)에서 존재하는지 확인
- 배치에서는 비주얼라이저/GUI를 비활성화하여 블로킹 회피
- 파일 핸들러는 `fclose()` 후 파일 ID를 `-1`로 리셋

**컨벤션(간단)**
- 클래스/모듈명: CamelCase (예: `DataLogger`)
- 파일명: 클래스명과 동일 (예: `DataLogger.m`)
- 로그 디렉토리: `matlab/data/<mode>/<timestamp>/`

**모듈 템플릿 위치**
- `matlab/modules/ModuleTemplates/` 에 기본 템플릿과 사용 예시가 포함되어 있습니다.

**어댑터(기존 코드 통합)**
- 기존의 레거시 모듈을 재작성하지 않고 표준화하려면 어댑터를 추가하세요. 예제 위치:
	- `matlab/modules/DataLoggerModule.m` (기존 `DataLogger` 감싸기)
	- `matlab/modules/RealtimeVisualizerModule.m` (기존 `RealtimeVisualizer` 감싸기)
	- `matlab/modules/DataCollectorModuleAdapter.m` (기존 `DataCollectionModule` 감싸기)
	- `matlab/modules/WindControlModuleAdapter.m` (기존 `WindControlModule` 감싸기)

**통합 실행 예시**
- 간단한 모듈 초기화/시작/정리 흐름 예시는 `matlab/main/integrated_example.m`에 있습니다.
