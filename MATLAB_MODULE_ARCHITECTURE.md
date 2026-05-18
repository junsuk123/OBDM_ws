# MATLAB 모듈 아키텍처 가이드

공식 참조: https://ardupilot.org/dev/docs/sitl-with-gazebo.html

## 개요

**문제점:**  기존에는 `integrated_main.m`이 데이터 수집, 제어, 렌더링 등 모든 기능을 직접 구현하고 있어서:
- 파일이 복잡해짐
- 수정할 때마다 전체 통합 테스트 필요
- 각 기능의 독립 테스트 어려움
- 코드 재사용성 낮음

**해결책:**  각 기능을 독립적인 **모듈(클래스)**로 분리하고, `integrated_main.m`은 **오케스트레이션**만 담당

---

## 아키텍처

```
integrated_main.m (orchestrator)
    ├── SimulationManager (existing)
    ├── DataCollectionModule (new) 
    ├── CircleControlModule (new)
    └── 기타 모듈들 (future)
```

### 각 모듈의 역할

| 모듈 | 파일 | 역할 |
|------|------|------|
| `DataCollectionModule` | `matlab/modules/DataCollectionModule.m` | 센서 데이터 수집 |
| `CircleControlModule` | `matlab/modules/CircleControlModule.m` | 원형 비행 제어 |
| `SimulationManager` | `matlab/modules/SimulationManager.m` | 시뮬레이터 시작/중지 |


## 모듈 개발 패턴

모든 모듈은 다음 패턴을 따릅니다:

```matlab
%% MyModule - 기능 설명
% 
% 용도: 무엇을 하는가
% 
% 사용법:
%   obj = MyModule('param1', value1, 'param2', value2);
%   obj = obj.initialize();
%   result = obj.execute_operation();
%   obj.cleanup();

classdef MyModule < handle
    properties (Access = private)
        % 내부 상태
    end
    
    properties (Access = public)
        % 공개 상태
        is_initialized = false
    end
    
    methods
        % 생성자
        function obj = MyModule(varargin)
            p = inputParser;
            addParameter(p, 'param1', default_value, @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});
            
            % 초기화
        end
        
        % 초기화
        function obj = initialize(obj)
            % ROS2/시스템 연결
            obj.is_initialized = true;
        end
        
        % 주요 동작
        function result = execute_operation(obj)
            % 실제 작업 수행
        end
        
        % 정리
        function cleanup(obj)
            % 리소스 해제
        end
    end
    
    methods (Access = private)
        % 내부 헬퍼 함수들
        function helper_func(obj)
        end
    end
end
```


## 데이터 수집 모듈 사용법

### 1. 독립적 테스트 (모듈만 실행)

```matlab
% test_data_collection_module.m
collector = DataCollectionModule('namespace', '/iris', 'verbose', true);
collector = collector.subscribe_to_topics();
data = collector.collect_data(10);
collector.print_summary(data);
collector.unsubscribe_from_topics();
```

### 2. 통합 테스트에서 호출

```matlab
% integrated_main.m 내부
collector = DataCollectionModule('namespace', '/iris', 'verbose', true);
collector = collector.subscribe_to_topics();
data = collector.collect_data(10);
collector.print_summary(data);
collector.unsubscribe_from_topics();
```

### 주요 메서드

| 메서드 | 설명 | 반환값 |
|--------|------|--------|
| `subscribe_to_topics()` | 모든 센서 구독 | obj (체인 가능) |
| `collect_data(duration_sec)` | 지정 시간 동안 데이터 수집 | data 구조체 |
| `print_summary(data)` | 수집 결과 출력 | - |
| `unsubscribe_from_topics()` | 모든 구독 해제 | obj |


## 제어 모듈 사용법

### 1. 독립적 테스트

```matlab
% test_circle_control_module.m
controller = CircleControlModule('namespace', '/iris', 'verbose', true);
controller = controller.initialize();

success = controller.execute_circle_flight(...
    'radius_m', 100, ...
    'altitude_agl_m', 30, ...
    'num_points', 36, ...
    'tolerance_m', 30, ...
    'waypoint_timeout_s', 20);

if success
    fprintf('Circle flight successful\n');
end

controller.cleanup();
```

### 2. 통합 테스트에서 호출

```matlab
% integrated_main.m 내부
controller = CircleControlModule('namespace', '/iris', 'verbose', true);
controller = controller.initialize();
success = controller.execute_circle_flight(...);
controller.cleanup();
```

### 주요 메서드

| 메서드 | 설명 |
|--------|------|
| `initialize()` | ROS2 퍼블리셔/구독자 생성 |
| `execute_circle_flight(options)` | 원형 비행 실행 |
| `cleanup()` | 리소스 해제 |


## 통합 테스트 실행

### 데이터 수집만 실행

```bash
cd ~/OBDM_ws/matlab
matlab -batch "run('main/integrated_main.m');"
```

### 데이터 수집 + 제어 테스트 실행

```bash
export OBDM_CONTROL_TEST=circle
matlab -batch "run('main/integrated_main.m');"
```

### 다른 시뮬레이션 모드

```bash
export OBDM_RUN_MODE=gazebo_bridge
matlab -batch "run('main/integrated_main.m');"
```


## 새로운 모듈 추가 단계

### 1. 모듈 파일 생성

`matlab/modules/YourNewModule.m` 생성:

```matlab
classdef YourNewModule < handle
    properties (Access = private)
        namespace
        verbose
    end
    
    properties (Access = public)
        is_initialized = false
    end
    
    methods
        function obj = YourNewModule(varargin)
            p = inputParser;
            addParameter(p, 'namespace', '/iris', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});
            
            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
        end
        
        function obj = initialize(obj)
            % 초기화 로직
            obj.is_initialized = true;
        end
        
        function result = execute_operation(obj)
            % 실제 작업
        end
        
        function cleanup(obj)
            % 정리
        end
    end
end
```

### 2. 독립 테스트 파일 생성

`matlab/verification/test_your_new_module.m`:

```matlab
addpath(genpath(pwd));
module = YourNewModule('namespace', '/iris', 'verbose', true);
module = module.initialize();
result = module.execute_operation();
module.cleanup();
```

### 3. integrated_main.m에서 호출

```matlab
% integrated_main.m 내부
module = YourNewModule('namespace', ns, 'verbose', true);
module = module.initialize();
result = module.execute_operation();
module.cleanup();
```


## 모범 사례 (Best Practices)

### ✅ DO (좋은 예)

1. **각 모듈은 하나의 책임만 가짐**
   ```matlab
   % Good: DataCollectionModule은 데이터 수집만 담당
   collector = DataCollectionModule(...);
   data = collector.collect_data(10);
   ```

2. **모듈은 독립적으로 테스트 가능**
   ```matlab
   % 각 모듈을 독립적으로 실행 가능
   test_data_collection_module.m
   test_circle_control_module.m
   ```

3. **명확한 인터페이스**
   ```matlab
   % 공개 메서드는 명확한 목적
   obj = initialize()      % 초기화
   result = execute()      % 실행
   obj = cleanup()         % 정리
   ```

### ❌ DON'T (나쁜 예)

1. **integrated_main에 구현 로직 추가**
   ```matlab
   % Bad: 여기다 기능 구현하면 복잡해짐
   % ❌ DO NOT DO THIS in integrated_main:
   % for i = 1:100
   %     data = receive(sub, 1.0);
   %     ...
   % end
   ```

2. **전역 변수 사용**
   ```matlab
   % Bad: 모듈 간 상태 공유 어려움
   % global some_state;
   ```

3. **모듈이 너무 많은 책임**
   ```matlab
   % Bad: DataCollectionModule이 제어까지?
   % class DataCollectionModule
   %     function execute_control(obj)
   %         % 이건 CircleControlModule의 책임!
   %     end
   % end
   ```


## 디버깅 팁

### 특정 모듈만 테스트

```bash
# DataCollectionModule만 테스트
cd ~/OBDM_ws/matlab
matlab -batch "addpath(genpath(pwd)); test_data_collection_module;"
```

### 환경 변수로 동작 제어

```bash
# 상세 출력 모드
export MATLAB_VERBOSE=1

# 특정 기능만 실행
export OBDM_CONTROL_TEST=circle
```

### 로그 파일 확인

```bash
tail -f /tmp/obdm_quadcopter_launch.log
```


## 마이그레이션 체크리스트

기존 코드를 모듈로 변환할 때:



## FAQ

**Q: 모듈 간에 데이터를 공유하려면?**

A: 모듈의 공개 properties를 통해 반환:
```matlab
data = collector.collect_data(10);  % 데이터 반환
controller.execute_circle_flight(); % 컨트롤러 실행
```

**Q: 모듈 간 의존성이 있으면?**

A: 통합 테스트(integrated_main)에서 순서 조정:
```matlab
collector.collect_data();     % 먼저 실행
controller.execute_control(); % 그 다음 실행
```

**Q: 왜 모든 메서드가 `obj` 반환?**

A: 메서드 체인(method chaining) 가능:
```matlab
obj = DataCollectionModule(...).initialize().subscribe_to_topics();
```


## 참고

  - `SimulationManager.m` - 시뮬레이터 관리
  - `ProcessManager.m` - 프로세스 관리
  - `ROS2Manager.m` - ROS2 토픽/네임스페이스 관리
  - `AircraftStateReceiver.m` - 항공기 상태 수신

  - `DataCollectionModule.m` - 센서 데이터 수집
  - `CircleControlModule.m` - 원형 비행 제어
