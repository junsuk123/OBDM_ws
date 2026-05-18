# MATLAB 모듈 아키텍처 - 완성 요약

## 🎯 작업 완료

기존의 단일 `integrated_main.m`에 모든 기능이 혼재된 구조를 **모듈 기반 아키텍처**로 완전히 리팩토링했습니다.

---

## 📁 생성된 파일 목록

### 새로운 모듈 (Modules)

| 파일 | 역할 | 상태 |
|------|------|------|
| `matlab/modules/DataCollectionModule.m` | 센서 데이터 수집 | ✅ 완성 |
| `matlab/modules/CircleControlModule.m` | 원형 비행 제어 | ✅ 완성 |
| `matlab/modules/MODULE_TEMPLATE.m` | 새 모듈 템플릿 | ✅ 제공 |

### 테스트 스크립트 (Verification)

| 파일 | 역할 | 상태 |
|------|------|------|
| `matlab/verification/test_data_collection_module.m` | DataCollectionModule 독립 테스트 | ✅ 완성 |
| `matlab/verification/test_circle_control_module.m` | CircleControlModule 독립 테스트 | ✅ 완성 |
| `matlab/verification/TEST_MODULE_TEMPLATE.m` | 테스트 템플릿 | ✅ 제공 |

### 문서 (Documentation)

| 파일 | 역할 |
|------|------|
| `MATLAB_MODULE_ARCHITECTURE.md` | 상세 개발 가이드 |
| `MATLAB_QUICK_START.md` | 빠른 시작 가이드 (이 파일) |

### 수정된 파일

| 파일 | 변경사항 |
|------|----------|
| `matlab/main/integrated_main.m` | 모든 구현 로직 제거 → 순수 오케스트레이션만 남김 |
| `matlab/modules/AircraftStateReceiver.m` | 클래스 정의 오류 수정 (methods 블록 정리) |
| `matlab/modules/DataCollectionModule.m` (신규) | 센서 수집 로직 모듈화 |

---

## 🚀 빠른 사용법

### 1. 데이터 수집만 실행

```bash
cd ~/OBDM_ws/matlab
matlab -batch "addpath(genpath(pwd)); run('main/integrated_main.m');"
```

### 2. 데이터 수집 + 원형 비행 테스트

```bash
export OBDM_CONTROL_TEST=circle
matlab -batch "addpath(genpath(pwd)); run('main/integrated_main.m');"
```

### 3. 특정 모듈만 테스트

```bash
# DataCollectionModule 테스트
matlab -batch "addpath(genpath(pwd)); test_data_collection_module;"

# CircleControlModule 테스트 (시뮬레이터 필요)
matlab -batch "addpath(genpath(pwd)); test_circle_control_module;"
```

---

## 📐 아키텍처 구조

```
matlab/
├── main/
│   └── integrated_main.m
│       ├─ 시뮬레이션 시작
│       ├─ DataCollectionModule 호출
│       ├─ CircleControlModule 호출 (선택적)
│       └─ 정리
│
├── modules/
│   ├── SimulationManager.m (기존)
│   ├── ProcessManager.m (기존)
│   ├── DataCollectionModule.m (신규)
│   ├── CircleControlModule.m (신규)
│   ├── MODULE_TEMPLATE.m (템플릿)
│   └── ...
│
└── verification/
    ├── test_data_collection_module.m (신규)
    ├── test_circle_control_module.m (신규)
    ├── TEST_MODULE_TEMPLATE.m (템플릿)
    └── ...
```

---

## ✅ 모듈 개발 표준

모든 모듈은 다음 인터페이스를 따릅니다:

```matlab
classdef MyModule < handle
    % 1. 생성자
    function obj = MyModule(varargin)
        % 파라미터 파싱
    end
    
    % 2. 초기화 (ROS2 연결, 서비스/구독자 생성)
    function obj = initialize(obj)
        obj.is_initialized = true;
    end
    
    % 3. 주요 동작
    function result = execute_operation(obj)
        % 실제 작업 수행
    end
    
    % 4. 정리 (리소스 해제)
    function cleanup(obj)
        % 정리 코드
    end
end
```

---

## 🔄 워크플로우

### 기존 방식 (❌ 안됨)
```
integrated_main.m 직접 수정
    ↓
파일 복잡해짐
    ↓
전체 테스트 필요
    ↓
다른 부분 영향받음
```

### 새로운 방식 (✅ 권장)
```
1. 새 모듈 생성 (MyModule.m)
    ↓
2. 독립 테스트 (test_my_module.m)
    ↓
3. integrated_main에서만 호출
    ↓
4. 통합 테스트
```

---

## 📝 다음 단계

### 새로운 기능 추가 체크리스트

```
□ 1. matlab/modules/NewFeatureModule.m 생성
      └─ MODULE_TEMPLATE.m 복사 후 수정
      
□ 2. matlab/verification/test_new_feature_module.m 생성
      └─ TEST_MODULE_TEMPLATE.m 복사 후 수정
      
□ 3. 모듈 독립 테스트 실행
      └─ test_new_feature_module();
      
□ 4. integrated_main.m에 호출 코드 추가
      └─ feature = NewFeatureModule(...);
          feature = feature.initialize();
          result = feature.execute_operation();
          feature.cleanup();
      
□ 5. 전체 통합 테스트
      └─ run('main/integrated_main.m');
      
□ 6. 가이드 문서 업데이트
      └─ MATLAB_MODULE_ARCHITECTURE.md 수정
```

---

## 🐛 디버깅

### 특정 모듈 디버깅

```matlab
% 1. 모듈 생성
module = MyModule(...);

% 2. 초기화
module = module.initialize();

% 3. breakpoint 설정
dbstop in MyModule.m at 50  % 50번 줄에서 멈춤

% 4. 실행
result = module.execute_operation();

% 5. 디버깅 모드
dbstep  % 한 줄씩 진행
```

### 로그 확인

```bash
# 시뮬레이션 로그
tail -f /tmp/obdm_plane_launch.log

# ROS2 토픽 확인
ros2 topic list
ros2 topic hz /alti_transition/imu
```

---

## 📚 참고 문서

| 문서 | 내용 |
|------|------|
| `MATLAB_MODULE_ARCHITECTURE.md` | 상세 개발 가이드 (베스트 프랙티스 포함) |
| `MATLAB_QUICK_START.md` | 이 문서 (빠른 시작) |
| `matlab/modules/MODULE_TEMPLATE.m` | 새 모듈 작성 템플릿 |
| `matlab/verification/TEST_MODULE_TEMPLATE.m` | 테스트 작성 템플릿 |

---

## ⚠️ 주의사항

### ❌ 하면 안 되는 것

1. **`integrated_main.m`에 기능 구현 추가**
   ```matlab
   % ❌ DO NOT:
   % integrated_main.m에 이렇게 하면 안됨:
   % for i = 1:100
   %     data = receive(sub, 1.0);
   %     ...
   % end
   
   % ✅ DO:
   % 대신 DataCollectionModule.m에 구현
   ```

2. **전역 변수 사용**
   ```matlab
   % ❌ DO NOT:
   % global some_state;
   
   % ✅ DO:
   % 모듈의 properties 사용
   ```

3. **모듈 간 강한 결합**
   ```matlab
   % ❌ DO NOT:
   % collector.control_data = controller.result;
   
   % ✅ DO:
   % data = collector.collect_data();
   % controller.execute(data);
   ```

### ✅ 해야 하는 것

1. **각 모듈은 하나의 책임만**
2. **모듈별 독립 테스트 스크립트 제공**
3. **`initialize()` → `execute()` → `cleanup()` 패턴 따르기**
4. **공개(Public)/비공개(Private) 메서드 구분**

---

## 💡 예제

### 예제 1: 새로운 배터리 모니터링 모듈 추가

```matlab
% 1. matlab/modules/BatteryMonitorModule.m 생성
classdef BatteryMonitorModule < handle
    methods
        function obj = BatteryMonitorModule(varargin)
            % 파라미터 처리
        end
        
        function obj = initialize(obj)
            % 배터리 모니터링 초기화
        end
        
        function status = check_battery(obj)
            % 배터리 상태 확인
        end
        
        function cleanup(obj)
            % 정리
        end
    end
end

% 2. 독립 테스트
battery = BatteryMonitorModule(...);
battery = battery.initialize();
status = battery.check_battery();
battery.cleanup();

% 3. integrated_main.m에 추가
battery_monitor = BatteryMonitorModule(...);
battery_monitor = battery_monitor.initialize();
battery_status = battery_monitor.check_battery();
```

---

## 📞 문제 해결

**Q: 모듈이 작동하지 않아요**
- A: 해당 모듈의 독립 테스트 스크립트 먼저 실행해서 격리
  ```bash
  matlab -batch "addpath(genpath(pwd)); test_module_name;"
  ```

**Q: `integrated_main.m`에서 모듈 호출이 안 되요**
- A: addpath 확인
  ```matlab
  addpath(genpath(pwd));  % 모든 matlab 폴더 추가
  ```

**Q: 여러 모듈 동시 사용 가능?**
- A: 네, 순서만 맞추면 됨
  ```matlab
  collector.collect_data();
  controller.execute_circle();  % 데이터 수집 후 제어
  ```

---

## 🎉 완성!

이제 다음 번엔 이 모듈 기반 아키텍처를 따르면 됩니다:

✅ 각 기능 → 독립 모듈
✅ 모듈별 테스트 → 독립 테스트 스크립트
✅ 통합 → `integrated_main.m`에서만 호출

**Happy coding! 🚀**
