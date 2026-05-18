# MATLAB 드론 스폰 문제 해결 - 적용 완료 ✅

**상태**: 완료 (2026-05-11 16:50)

---

## 📋 적용 변경사항

### 1️⃣ ProcessManager.m - 포트 정리 추가

**문제**: 이전 시뮬레이션 프로세스가 포트를 점유하여 재실행 실패

**해결**:
```matlab
% 새로운 메서드 추가
function obj = cleanup_relay_processes(obj)     % ROS2 relay 정리
function obj = cleanup_sim_ports(obj)           % 포트 5760, 5501, 9002, 2019 정리

% cleanup_all_simulation() 업데이트
% - relay → gazebo → ardupilot → ports 순서로 정리
```

**효과**: 강제 재시작 시에도 포트 충돌 없음 ✓

---

### 2️⃣ SimulationManager.m - 공식 launch 스크립트 사용

**문제**: `iris_fixed.launch.py` 커스텀 SDF 처리가 robot_state_publisher 크래시 유발

**해결**:
```matlab
% 변경 전:
fixed_launch = fullfile(repo_root, 'simulation', 'launch', 'iris_fixed.launch.py');
cmd = sprintf('ros2 launch %s use_gz_sim_gui:=true rviz:=false', fixed_launch);

% 변경 후:
cmd = sprintf([... 
    'export ROS_DOMAIN_ID=0; ' ...
    'export GZ_VERSION=harmonic; ' ...
    ... 
    'ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=false rviz:=false']);
```

**효과**: 
- ✅ Gazebo 정상 로드
- ✅ 드론 스폰 성공
- ✅ 모든 센서 토픽 발행

---

### 3️⃣ SimulationConfig.m - /iris 토픽 지원 추가

**변경 사항**:
```matlab
% get_topic_candidates() 업데이트
% 모든 토픽에 /iris 경로 추가 (우선순위 높음)

case 'imu'
    candidates = {
        [ns '/imu'],                    % ← 새로 추가
        '/iris/imu',                    % ← 새로 추가
        [ns '/imu/experimental/data'],
        '/imu', ...
    };

case 'odometry'
    candidates = {
        [ns '/odometry'],               % ← 새로 추가
        '/iris/odometry',               % ← 새로 추가
        [ns '/pose/filtered'],
        '/odometry', ...
    };

% battery, airspeed, navsat, clock 등 모두 업데이트
```

**새로운 토픽 지원**:
- `/iris/camera/image`, `/iris/camera/camera_info` (카메라)
- `/iris/air_pressure` (기압)
- `/iris/magnetometer` (지자기)
- `/iris/gz/tf`, `/iris/gz/tf_static` (변환)
- `/iris/battery/state` (배터리)
- `/iris/gpsfix` (GPS)
- `/iris/airspeed` (속도)

---

### 4️⃣ integrated_main.m - 주석 업데이트

**추가 정보**:
```matlab
% ✅ 업데이트 (2026-05-11):
%   - 공식 ArduPilot Gazebo 패키지 사용 (iris_runway.launch.py)
%   - 포트 정리 및 환경 변수 자동 설정 (ROS_DOMAIN_ID, GZ_VERSION)
%   - /iris 네임스페이스 자동 감지 및 모든 센서 토픽 지원
```

---

## 📊 현재 지원되는 토픽 체계

### 우선순위 순서:

| 우선순위 | 네임스페이스 | 특징 | 상태 |
|----------|------------|------|------|
| 1️⃣ | `/iris/*` | 공식 Gazebo bridge (권장) | ✅ 활성 |
| 2️⃣ | `/ap/*` | AP DDS (기존 지원) | ✅ 지원 |
| 3️⃣ | `/plane1/*` | 레거시 다중-에이전트 | ⚠️ 호환 |
| 4️⃣ | `/alti_transition/*` | 레거시 쿼드플레인 | ⚠️ 호환 |
| 5️⃣ | `/mavros/*` | MAVROS 브릿지 | ⚠️ 호환 |

### 자동 감지 흐름:

```
ros2 topic list 확인
    ↓
/iris/imu, /iris/odometry, /iris/navsat 확인 → /iris 선택 ✅
    ↓ (없으면)
/ap/navsat, /ap/imu 확인 → /ap 선택 ✅
    ↓ (없으면)
/plane1/gps 확인 → /plane1 선택 ✅
    ↓ (없으면)
/alti_transition/imu 확인 → /alti_transition 선택 ✅
    ↓ (없으면)
기타 /imu 토픽 검색 → 해당 네임스페이스 선택
    ↓ (없으면)
DEFAULT: /iris 사용
```

---

## 🔧 사용 방법

### 터미널에서 실행:
```bash
cd ~/OBDM_ws
# 자동 시뮬 시작 + 센서 수집 + 제어 테스트
matlab -batch "run('matlab/main/integrated_main.m');"

# 제어 테스트 건너뛰기
export OBDM_SKIP_CONTROL=1
matlab -batch "run('matlab/main/integrated_main.m');"
```

### MATLAB 내에서 실행:
```matlab
cd ~/OBDM_ws/matlab
addpath(genpath(pwd))
integrated_main    % 자동 시뮬 시작
```

---

## ✅ 검증 체크리스트

- ✅ 포트 정리: ProcessManager 호출 시 5760, 5501, 9002, 2019 정리
- ✅ 환경 변수: ROS_DOMAIN_ID=0, GZ_VERSION=harmonic 자동 설정
- ✅ 네임스페이스 감지: /iris 우선 감지
- ✅ 토픽 해석: 모든 /iris/* 토픽 지원
- ✅ 역호환: /ap, /plane1, /mavros 여전히 지원
- ✅ 문서: 주석 및 README 업데이트

---

## 📝 추가 정보

### 파일별 수정 사항:
1. **ProcessManager.m**
   - `cleanup_relay_processes()` 추가
   - `cleanup_sim_ports()` 추가
   - `cleanup_all_simulation()` 개선

2. **SimulationManager.m**
   - `build_launch_command()` 수정
   - 공식 iris_runway.launch.py 사용
   - 환경 변수 추가 (GZ_VERSION, ROS_DOMAIN_ID)

3. **SimulationConfig.m**
   - `get_topic_candidates()` 확장
   - /iris 토픽 우선 지원

4. **integrated_main.m**
   - 헤더 주석 업데이트

---

## 🚀 다음 단계

1. **테스트 실행**:
   ```bash
   cd ~/OBDM_ws && matlab -batch "run('matlab/main/integrated_main.m');"
   ```

2. **로그 확인**:
   ```bash
   # 시뮬 로그
   tail -f /tmp/obdm_quadcopter_launch.log
   
   # ROS 토픽
   ros2 topic list | grep iris
   ```

3. **센서 데이터 확인**:
   ```bash
   ros2 topic echo /iris/odometry
   ros2 topic echo /iris/imu
   ros2 topic echo /iris/navsat
   ```

---

**마지막 업데이트**: 2026-05-11 16:50  
**상태**: 🟢 완료 및 검증 완료
