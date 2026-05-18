# ROS2 + Gazebo 시뮬레이션 설정 가이드

## 현재 상황 분석

### 필요한 구성 요소
1. **ROS2 Humble** - `/opt/ros/humble/setup.bash` ✓
2. **ArduPilot Gazebo Workspace** - `~/ardu_ws/` ✓
3. **MATLAB Flight Control Code** - `~/OBDM_ws/matlab/` ✓

### ArduPilot 공식 문서
- https://ardupilot.org/dev/docs/ros2-gazebo.html

## 운영 방식

### 방법 1: ardu_ws 사용 (권장 - 현재)
직접 `ros2 launch` 명령 사용

```bash
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic

# 옵션 1: 직접 실행
ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=true rviz:=false

# 옵션 2: 스크립트 사용
bash ~/OBDM_ws/simulation/launch/launch_quadcopter.sh
```

### MATLAB에서 자동 실행
```matlab
cd ~/OBDM_ws/matlab
run('main/integrated_main.m')
```

## 파일 구조

```
~/ardu_ws/
├── install/                              # 설치된 패키지들
│   ├── setup.bash                        # ROS2 환경 설정
│   ├── ardupilot_gz_bringup/             # 메인 런치 패키지
│   ├── ardupilot_gz_gazebo/              # Gazebo 플러그인
│   └── ...
├── src/
│   └── ardupilot_gz/
│       └── ardupilot_gz_bringup/
│           ├── launch/
│           │   ├── iris_runway.launch.py              # ← 메인 런치 파일
│           │   └── robots/
│           │       └── iris.launch.py
│           ├── config/
│           │   └── iris_bridge.yaml
│           ├── rviz/
│           │   └── iris.rviz

~/OBDM_ws/
│   └── modules/
│       ├── SimulationManager.m           # 수정됨: 직접 ros2 launch 사용
│       ├── ROS2Manager.m
│       └── ProcessManager.m
├── simulation/
│   └── launch/
│       └── launch_quadcopter.sh          # 런치 스크립트 (옵션)
└── docs/
    ├── LAUNCH_TROUBLESHOOTING.md
    └── MATLAB_SYSTEM_COMMANDS.md
```

## SimulationManager 개선사항

### 이전 방식
- 런치 스크립트 파일(`launch_quadcopter.sh`) 실행
- 환경 변수 스크립트 내 임베드

### 신규 방식
- MATLAB에서 직접 `setenv()` 사용
- 빌드된 ros2 launch 명령 직접 실행
- 더 명확한 오류 메시지

```matlab
% 환경 변수 설정
setenv('ROS_DOMAIN_ID', '0');
setenv('GZ_VERSION', 'harmonic');

% ros2 launch 명령 직접 구성 및 실행
launch_cmd = 'source /opt/ros/humble/setup.bash; source ~/ardu_ws/install/setup.bash; ros2 launch ardupilot_gz_bringup iris_runway.launch.py use_gz_sim_gui:=true rviz:=false';

% 백그라운드 실행
system(sprintf('nohup bash -c ''%s'' > /tmp/launch.log 2>&1 & echo $! > /tmp/launch.pid', launch_cmd));
```

## 문제 해결

### 1. "ardu_ws not found"
```bash
# 확인
ls ~/ardu_ws/install/setup.bash

# 없으면 설치
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_gz_bringup
```

### 2. "topics not detected"
```bash
# 로그 확인
tail -f /tmp/obdm_quadcopter_launch.log

# Gazebo 프로세스 확인
ps aux | grep gz

# 토픽 확인
ros2 topic list
```

### 3. "ROS2 command not found"
```bash
# ROS2 설치 확인

## 필요시 파일 복사

현재는 ardu_ws를 직접 참조하므로 복사 불필요합니다. 

필요한 경우만 복사:
```bash
# 모델 파일 복사 (선택사항)
cp -r ~/ardu_ws/src/ardupilot_gz_bringup/models ~/OBDM_ws/simulation/

# 론치 파일 복사 (선택사항)
cp -r ~/ardu_ws/src/ardupilot_gz_bringup/launch ~/OBDM_ws/simulation/
```

## 검증 체크리스트

- [ ] `/opt/ros/humble/setup.bash` 존재
- [ ] `~/ardu_ws/install/setup.bash` 존재
- [ ] `~/OBDM_ws/matlab/main/integrated_main.m` 존재
- [ ] MATLAB R2024a 이상 설치
- [ ] ROS2 네트워크 설정 확인 (`ROS_DOMAIN_ID=0`)
- [ ] Gazebo 그래픽 가능 여부 (`DISPLAY` 변수 설정)

## 실행 명령어

```matlab
% 1. MATLAB 시작
% 2. 현재 디렉토리 변경
cd ~/OBDM_ws/matlab

% 3. 통합 테스트 실행
run('main/integrated_main.m')
```

## 예상 출력

```
╔════════════════════════════════════╗
║    Integrated Flight Control       ║
╚════════════════════════════════════╝

Setting up environment variables...

═════════════════════════════════════
1️⃣ Initializing Managers
═════════════════════════════════════

🚀 Launching simulation via ROS2...
  Waiting for launch process...
  ✓ Launch process started (PID: 12345)

⏳ Waiting for topics (max 30 seconds)...
  Elapsed: 1.2 s
  ...
  ✓ Topics available!

...
```

## 참고 자료

- [MathWorks - Run External Commands](https://kr.mathworks.com/help/matlab/matlab_external/run-external-commands-scripts-and-programs.html)
- [ArduPilot ROS2 + Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
