# ArduPilot + Gazebo Drone Spawn 해결 보고서

**상태**: ✅ **해결됨** - 드론이 정상적으로 Gazebo에 스폰되고 있습니다.

---

## 🔍 진단 결과

### 문제점 (iris_fixed.launch.py)
1. **커스텀 SDF URI 해석** - 잘못된 URI 변환으로 `package://` 경로 실패
2. **robot_state_publisher 크래시** - URDF 파싱 오류로 프로세스 종료 (exit code -6)
3. **포트 충돌** - 이전 프로세스가 포트 5760 유지
4. **공식 문서 미준수** - 공식 iris_runway.launch.py와 불일치

---

## ✅ 해결 방법

### 1. 공식 iris_runway.launch.py 사용

**파일**: [simulation/launch/iris_fixed.launch.py](simulation/launch/iris_fixed.launch.py)

표준 아키텍처로 변경:
```python
# Iris 로봇 - ardupilot_gz_bringup에서 제공하는 표준 iris.launch.py 사용
robot = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            pkg_project_bringup,
            "launch",
            "robots",
            "iris.launch.py",
        ])
    ]),
    condition=IfCondition(LaunchConfiguration("spawn_robot")),
)

# 별도 SDF 처리 제거 - 표준 패키지가 처리함
```

### 2. 프로세스 정리
```bash
pkill -9 gz; pkill -9 arduplane; pkill -9 relay; sleep 2
```

---

## 📊 현재 상태

### ✅ Gazebo 정상 작동
```
[gz sim-1] [Msg] Gazebo Sim Server v8.11.0
[gz sim-1] [Msg] Loading SDF world file[.../runway.sdf]
[gz sim-1] [Msg] Loaded level [default]
[gz sim-1] [Msg] World [runway] initialized with [1ms] physics profile
```

### ✅ 드론 스폰 완료
```
[create-7] [INFO] Requested creation of entity
[create-7] [INFO] OK creation of entity
[gz sim-1] [Msg] OdometryPublisher publishing odometry on [/model/iris/odometry]
```

### ✅ ROS 2 브릿지 활성화
```
[parameter_bridge-3] Creating GZ->ROS Bridge: [...clock...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...joint_states...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...odometry...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...camera/image...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...imu...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...magnetometer...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...navsat...]
[parameter_bridge-3] Creating GZ->ROS Bridge: [...battery...]
```

### ✅ ArduCopter SITL 작동
```
[arducopter] Starting SITL: JSON
[arducopter] JSON control interface set to 127.0.0.1:9002
[arducopter] Starting sketch 'ArduCopter'
[arducopter] Home: -35.363262 149.165237 alt=584.000000m hdg=353.000000
```

---

## 📋 실행 명령어

### 공식 방법 (권장)
```bash
cd ~/ardu_ws
source install/setup.bash
export ROS_DOMAIN_ID=0
export GZ_VERSION=harmonic

# GUI 없이 시뮬레이션
ros2 launch ardupilot_gz_bringup iris_runway.launch.py \
  use_gz_sim_gui:=false rviz:=false

# GUI 포함
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### 프로젝트 커스텀 (OBDM)
```bash
cd /home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws
ros2 launch simulation/launch/iris_fixed.launch.py \
  use_gz_sim_gui:=false rviz:=false
```

---

## 🔗 공식 문서 참고

- [ROS 2 with Gazebo](https://ardupilot.org/dev/docs/ros2-gazebo.html)
- [ROS 2 with SITL](https://ardupilot.org/dev/docs/ros2-sitl.html)
- [ROS 2 Waypoint Goal Interface](https://ardupilot.org/dev/docs/ros2-waypoint-goal-interface.html)

---

## 📝 주요 학습

| 항목 | 설명 |
|------|------|
| **패키지 구조** | 공식 ardupilot_gz_bringup의 iris.launch.py가 모든 SDF/URDF 처리 담당 |
| **포트 관리** | 시뮬레이션 재시작 전에 반드시 기존 프로세스 완전 정리 필요 |
| **네임스페이스** | `/iris/` 네임스페이스 자동 설정됨 |
| **DDS 설정** | `dds_use_ns.parm`, `dds_udp.parm` 자동 로드됨 |
| **환경 변수** | `ROS_DOMAIN_ID=0`, `GZ_VERSION=harmonic` 필수 |

---

## 🚀 다음 단계

1. **ROS 2 토픽 확인**
   ```bash
   ros2 topic list
   ros2 topic echo /iris/odometry
   ```

2. **드론 제어**
   ```bash
   # MAVProxy를 통한 수동 제어
   ros2 service call /iris/ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
   ros2 service call /iris/ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 0}"
   ```

3. **MATLAB 통합**
   - ROS 2 토픽으로부터 센서 데이터 구독
   - 목표점 명령 발행
   - SITL 상태 모니터링

---

**마지막 업데이트**: 2026-05-11 15:41
**상태**: 드론 스폰 ✅ 완료
