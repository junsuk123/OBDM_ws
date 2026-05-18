# MATLAB-Gazebo-ROS2 통합 시스템 상태 보고서

## 📋 작업 완료 요약

### 진행 상황
- ✅ **IMU 센서**: 데이터 수집 성공 (10 메시지/10초)
- ✅ **ROS2 인프라**: 정상 작동
- ✅ **ODOM 토픽 발행 활성화**: 모델에 OdometryPublisher 플러그인 추가
- ⚠️  **Gazebo 안정성**: 5-15초 후 크래시 (근본 원인 미파악, SIGKILL)
- ❌ **Battery**: 시뮬레이션 미구현
- ❌ **TF 토픽**: 발행 미설정

---

## 🔧 수행된 작업

### 1. 상세 진단 분석
**파일**: `analysis_report.md`
- Gazebo 크래시 근본 원인 분석 (OOM 아님, 커널 SIGKILL 아님)
- ODOM/TF/Battery 미수집 원인 파악
- 시스템 리소스 상태 검증

**결론**: 
- IMU: 정상 (Gazebo 센서 시뮬레이션 작동)
- ODOM: Gazebo 모델에서 발행하지 않아서 0 메시지
- Battery: Bridge 설정에 정의되지 않음
- Gazebo 크래시: 외부 SIGKILL, 스레드 생성 단계에서 발생

### 2. ODOM 토픽 활성화
**파일 수정**: `/home/j/ardu_ws/src/ardupilot_sitl_models/Gazebo/models/alti_transition_quad/model.sdf`

**변경 내용**:
```xml
<!-- 추가된 플러그인 -->
<plugin filename="gz-sim-odometry-publisher-system"
  name="gz::sim::systems::OdometryPublisher">
  <publish_frequency>10</publish_frequency>
  <target_frame>odom</target_frame>
</plugin>
```

**효과**: Gazebo가 `/model/alti_transition/odometry` 토픽을 10Hz로 발행하도록 설정

### 3. 향상된 MATLAB 테스트 스크립트
**파일**: `test_data_collection_enhanced.m`

**개선 사항**:
- IMU + ODOM 센서 포괄 (TF/Battery 제외)
- 개별 센서 에러 처리 (한 센서 실패해도 나머지 계속)
- 실시간 메시지 카운팅 및 주기 표시
- 상세한 샘플 데이터 출력 (가속도, 각속도, 위치 등)
- 명확한 성공/실패 표시

### 4. 통합 테스트 스크립트
**파일**: `run_integrated_test.sh`

**기능**:
1. 이전 프로세스 정리 (강제 종료)
2. ROS2 환경 설정
3. Gazebo 시뮬레이션 백그라운드 실행
4. 8초 대기 (Gazebo 초기화)
5. MATLAB 테스트 실행 (45초 타임아웃)
6. 완료 후 자동 정리

**사용 방법**:
```bash
/home/j/SynologyDrive/INCSL/devel/INCSL/OBDM_ws/run_integrated_test.sh
```

---

## 📊 예상 결과

### 이전 테스트 결과
```
IMU messages:      10 ✓
ODOM messages:     0  ✗
TF messages:       0  ✗
BATTERY messages:  0  ✗
```

### 예상 개선 후 결과
```
IMU messages:      10 ✓ (불변)
ODOM messages:     ~10 ✓ (NEW - 플러그인 추가로 활성화)
```

---

## ⚠️ 알려진 제약

### 1. Gazebo 프로세스 크래시
- **증상**: 5-15초 후 exit code -9 (SIGKILL)
- **영향**: 데이터 수집 시간 제한
- **원인**: 미파악 (Ruby wrapper 문제 또는 라이브러리 버그)
- **현재 해결책**: 타임아웃 기반 구성 (8초 대기 후 테스트 시작)

### 2. Battery 토픽 미지원
- Bridge 설정에 배터리 매핑 없음
- Gazebo 모델에 배터리 시뮬레이션 없음
- 추가 작업 필요 (우선순위: LOW)

### 3. TF 토픽 미설정
- ODOM 활성화로 일부 해결 가능
- 추가 설정 작업 필요 (우선순위: MEDIUM)

---

## 🚀 다음 단계

### 즉시 (1시간 이내)
1. **통합 테스트 실행**
   ```bash
   ./run_integrated_test.sh
   ```
   - ODOM 데이터가 이제 수집되는지 확인
   - IMU 데이터 계속 수집 확인

2. **결과 검증**
   - 10개 메시지 ODOM 수신 확인
   - 양쪽 센서 데이터 품질 확인

### 단기 (2-4시간)
1. **Gazebo 크래시 해결** (우선순위: CRITICAL)
   - Gazebo 8.12 또는 9.x 버전 테스트
   - Ruby wrapper 디버그 모드 실행
   - Gazebo 빌드 로그 상세 검사

2. **TF 토픽 활성화** (우선순위: HIGH)
   - Model SDF에 TF 발행 플러그인 추가
   - MATLAB 테스트에 TF 구독 추가

### 장기 (1일 이상)
1. **Battery 시뮬레이션** (우선순위: MEDIUM)
   - 배터리 시뮬레이션 모듈 추가
   - Bridge 설정에 배터리 매핑 추가

2. **전체 시스템 안정화**
   - 통합 테스트 자동화
   - 각 센서별 예상치 검증

---

## 📁 파일 위치

### 분석 및 보고
- `analysis_report.md` - 상세 진단 분석
- `README.md` - 시스템 개요

### MATLAB 스크립트
- `test_data_collection.m` - 기본 테스트 (IMU/ODOM/TF/BATTERY)
- `test_data_collection_enhanced.m` - 개선 테스트 (IMU/ODOM만)

### 자동화 스크립트
- `run_integrated_test.sh` - 통합 테스트 실행 스크립트

### 수정된 소스
- 소스: `/home/j/ardu_ws/src/ardupilot_sitl_models/Gazebo/models/alti_transition_quad/model.sdf`
- 설치: `/home/j/ardu_ws/install/ardupilot_sitl_models/share/ardupilot_sitl_models/models/alti_transition_quad/model.sdf`
  - 추가: OdometryPublisher 플러그인 (10Hz)

---

## 🔍 진단 명령어

### ROS2 토픽 확인
```bash
source /opt/ros/humble/setup.bash
source /home/j/ardu_ws/install/setup.bash
export ROS_DOMAIN_ID=0

# 모든 토픽 나열
ros2 topic list

# 특정 토픽 정보
ros2 topic info /alti_transition/imu -v
ros2 topic info /alti_transition/odometry -v

# 토픽 데이터 보기 (5개, 3초 타임아웃)
timeout 3 ros2 topic echo /alti_transition/imu --count 5
timeout 3 ros2 topic echo /alti_transition/odometry --count 5
```

### Gazebo 로그 확인
```bash
tail -f /tmp/alti_gz_fresh.log
# 또는 가장 최근 로그
tail -f ~/.ros/log/$(ls -t ~/.ros/log | head -1)/
```

### 프로세스 상태
```bash
ps aux | grep -E 'gz sim|ros_gz_bridge|mavproxy|parameter_bridge'
```

---

## ✅ 검증 체크리스트

- [ ] IMU 데이터 수집 (10 메시지 이상)
- [ ] ODOM 데이터 수집 (10 메시지 이상) **← NEW**
- [ ] ROS2 노드 정상 작동
- [ ] Gazebo 8초 이상 유지
- [ ] MATLAB 타임아웃 없이 완료
- [ ] Gazebo 강제 정리 (프로세스 제거)

---

## 📞 연락처 및 참고사항

**최종 상태**: 
- IMU 수집 ✓ 성공
- ODOM 수집 ✓ 활성화 (재테스트 필요)
- 시스템 기본 인프라 ✓ 작동

**다음 우선순위**: 
1. ODOM 데이터 수집 재확인
2. Gazebo 크래시 해결
3. TF 토픽 활성화
4. Battery 구현

**예상 완료 시간**: 
- ODOM 테스트: 5-10분
- Gazebo 디버깅: 1-2시간
- 전체 시스템: 4-6시간

