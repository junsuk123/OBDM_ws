# MATLAB-ROS2-Gazebo 통합 테스트 진단 보고서

## 요약
현재 시스템은 **IMU 센서 데이터만 수집 가능**한 상태입니다. 
- ✅ IMU: 10개 메시지 수집 성공 (1Hz)
- ❌ ODOM: 0개 (발행 미설정)
- ❌ TF: 0개 (발행 미설정)
- ❌ BATTERY: 0개 (bridge 설정 미정의)
- ❌ Gazebo: 5-15초 후 강제 종료 (Exit code -9/SIGKILL)

---

## 상세 분석

### 1. IMU 데이터 수집 (성공)
**구조:**
```
Gazebo World
  ↓ (sensor simulation)
/world/runway/model/alti_transition/link/imu_link/sensor/imu_sensor/imu
  ↓ (ros_gz_bridge parameter_bridge)
/alti_transition/imu (ROS2)
  ↓ (MATLAB subscriber)
IMU 메시지 수신
```

**원인:** SceneBroadcaster + Gazebo IMU 센서 시뮬레이션이 정상 작동

---

### 2. ODOM/TF 미수집 분석

#### Bridge 설정 (alti_transition_bridge.yaml)
```yaml
- ros_topic_name: "odometry"
  gz_topic_name: "/model/{{ robot_name }}/odometry"
  gz_type_name: "gz.msgs.Odometry"
```

#### 문제점
1. Bridge는 `/model/alti_transition/odometry` 토픽을 찾고 있음
2. 하지만 Gazebo 모델/월드에서 이 토픽을 발행하는 것이 **없음**

#### 근본 원인
- `model.sdf`: JointStatePublisher, LiftDrag, ApplyJointForce만 있음
  - odometry/pose 발행 플러그인 미설정
- `runway.sdf`: SceneBroadcaster는 있지만 pose/odometry 발행 설정 없음
- ArduPilotPlugin: 비행 제어 데이터만 처리, odometry 발행 안 함

#### 해결 방안
**옵션 A:** 모델에 odometry 발행 플러그인 추가 (requires model rebuild)
```xml
<plugin filename="gz-sim-odometry-publisher-system"
  name="gz::sim::systems::OdometryPublisher">
  <publish_frequency>10</publish_frequency>
</plugin>
```

**옵션 B:** Bridge 설정에서 odometry 매핑 제거 (현재 상태 유지)

**권장:** 옵션 A (향후 내비게이션 시스템 필요)

---

### 3. BATTERY 미수집 분석

#### 문제점
Bridge 설정(`alti_transition_bridge.yaml`)에 배터리 토픽 매핑 없음

#### 테스트 코드의 시도
```matlab
battery_sub = rossubscriber('/alti_transition/battery/state', 'sensor_msgs/BatteryState');
```
이 토픽은 Gazebo에서 발행되지 않음 (설정 없음)

#### 해결 방안
1. Gazebo 모델에 배터리 시뮬레이션 추가
2. Bridge 설정에 배터리 토픽 추가
3. MATLAB 테스트에서 구독 제거 (현재)

---

### 4. Gazebo 프로세스 강제 종료 분석

#### 증상
- Exit code: **-9** (SIGKILL = 강제 종료)
- Timing: 5-15초 후 일관되게 발생
- 로그: 정상 초기화 후 "Creating PostUpdate worker threads: 9" 단계에서 중단
- 발생 패턴: 간단한 SDF에서도 동일하게 발생

#### 진단 결과
| 원인 | 검사 결과 | 결론 |
|------|---------|------|
| OOM (Out of Memory) | 23Gi 가용 메모리 | ❌ 불가 |
| 커널 SIGKILL | journalctl -k 검색 결과 없음 | ❌ 불가 |
| 시스템 리소스 | ulimit -a 정상 | ❌ 리소스 부족 아님 |
| 디스크 | 226Gi 가용 | ❌ 부족 아님 |
| 멀티스레드 | GZ_THREADS=1 후에도 발생 | ❌ 스레드 수 문제 아님 |
| Plugin 특정 | 단순 SDF도 크래시 | ❌ Plugin 문제 아님 |

**근본 원인 후보:**
1. Gazebo 8.11.0 버전 버그 (Ruby 래퍼 또는 C++ 라이브러리)
2. Launch 시스템의 암묵적 타임아웃 (watchdog)
3. 환경 특정 이슈 (쿠버네티스, Docker, VM 리소스 제약)

**해결 방안:**
1. Gazebo 버전 업그레이드 (8.11 → 8.12 또는 9.x)
2. Launch 타임아웃 명시적 설정
3. Gazebo 디버그 심볼과 함께 재빌드

---

## 권장 조치 순서

### 즉시 (현재 제약 하에서 최대 효과)
1. ✅ IMU 데이터 10초 간 수집 성공
2. MATLAB test_data_collection.m 수정:
   - ODOM/TF/BATTERY 구독 제거 또는 에러 처리
   - IMU만 수집하도록 정리

### 단기 (1-2시간)
1. Model SDF에 OdometryPublisher 플러그인 추가
2. Bridge 설정 재검증
3. MATLAB에 ODOM/TF 구독 재추가 및 테스트

### 중기 (본격 디버깅)
1. Gazebo 8.12 또는 9.x 테스트
2. Gazebo 빌드 로그 상세 검사
3. Ruby gz 스크립트 디버그 모드 실행

### 장기 (안정화)
1. ArduPilot 모델 odometry 네이티브 발행
2. 배터리 시뮬레이션 추가
3. 통합 테스트 자동화

---

## 핵심 발견사항

**성공 증거:**
- MATLAB ROS2 노드 정상 작동
- IMU 구독 및 메시지 수신 성공
- ROS2 네임스페이스(/alti_transition) 정상
- Gazebo-ROS2 브리지 기본 구조 정상

**제약 조건:**
- Gazebo 프로세스: 5-15초만 유지 (단기 데이터만 수집 가능)
- ODOM/TF: Gazebo 모델 설정 미완료
- Battery: 배터리 시뮬레이션 미구현

---

## 다음 단계

1. **ODOM 활성화 (우선순위: HIGH)**
   - 파일: `/home/j/ardu_ws/src/ardupilot_sitl_models/models/alti_transition_quad/model.sdf`
   - 액션: OdometryPublisher 플러그인 추가
   - 소요시간: 15분

2. **Gazebo 크래시 디버깅 (우선순위: CRITICAL)**
   - 근본 원인 파악 시 시스템 안정화 가능
   - 현재: 임시 제약 (5-15초 데이터만 수집)

3. **MATLAB 테스트 개선 (우선순위: MEDIUM)**
   - 에러 처리 강화
   - 타임아웃 동적 조정
   - 재시도 로직 추가

