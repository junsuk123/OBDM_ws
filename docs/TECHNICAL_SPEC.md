# OBDM 기술 명세서 및 데이터 정의

## 1. 시뮬레이션-MATLAB 데이터 인터페이스

### 1.1 데이터 흐름

```
Gazebo (물리 시뮬레이션)
         ↓
ArduPilot SITL (비행 제어 소프트웨어)
         ↓
ROS2 토픽 (DDS 미들웨어)
         ↓
MATLAB ROS Toolbox (구독자)
```

### 1.2 ROS2 토픽 정의

#### TF (Transform) 토픽
**토픽**: `/ap/tf`  
**메시지 타입**: `geometry_msgs/TransformStamped`  
**주기**: 10 Hz  
**용도**: 항공기의 위치(XYZ) 및 자세(Quaternion)

**데이터 구조**:
```
Header
  seq: uint32                      # 메시지 순번
  stamp: builtin_interfaces/Time   # 타임스탬프
  frame_id: string                 # "map" (기준 프레임)
Transform
  translation:
    x: float64                      # 미터
    y: float64
    z: float64
  rotation:
    x, y, z, w: float64            # Quaternion (w, x, y, z 순서)
```

**MATLAB 코드**:
```matlab
% Quaternion → Euler 각도 변환
quat = [w, x, y, z];
[roll, pitch, yaw] = quat2euler(quat);  % 라디안
```

#### IMU 토픽
**토픽**: `/ap/imu/sample`  
**메시지 타입**: `sensor_msgs/Imu`  
**주기**: 50 Hz  
**용도**: 가속도, 각속도, 자세 covariance

**데이터 구조**:
```
Header
  seq: uint32
  stamp: builtin_interfaces/Time
  frame_id: string                 # "body" (항공기 몸체 기준)
Orientation (자세각)
  x, y, z, w: float64             # Quaternion
OrientationCovariance: float64[9] # 3x3 행렬
AngularVelocity (각속도)
  x, y, z: float64                # rad/s (Roll, Pitch, Yaw)
AngularVelocityCovariance: float64[9]
LinearAcceleration (선형 가속도)
  x, y, z: float64                # m/s²
LinearAccelerationCovariance: float64[9]
```

#### GPS 토픽
**토픽**: `/ap/gps/sample`  
**메시지 타입**: `sensor_msgs/NavSatFix`  
**주기**: 10 Hz  
**용도**: GPS 위도, 경도, 고도

**데이터 구조**:
```
Header
  seq, stamp, frame_id
Status
  status: int8                     # 0: no fix, 2: 3D fix
  service: uint16                  # 위성 개수
Latitude: float64                  # 도 (°)
Longitude: float64                 # 도 (°)
Altitude: float64                  # 미터 (AMSL - 평균 해면 기준)
PositionCovariance: float64[9]    # 3x3 공분산 행렬
```

#### 기압계 토픽
**토픽**: `/ap/baro/sample`  
**메시지 타입**: `sensor_msgs/FluidPressure`  
**주기**: 10 Hz  
**용도**: 기압 데이터 (고도 보조)

**데이터 구조**:
```
Header
  seq, stamp, frame_id
Fluid_pressure: float64            # 파스칼(Pa)
Variance: float64                  # 측정 불확도
```

#### 배터리 토픽
**토픽**: `/ap/battery/state`  
**메시지 타입**: `sensor_msgs/BatteryState`  
**주기**: 2 Hz  
**용도**: 배터리 전압, 전류, 잔량

**데이터 구조**:
```
Header
  seq, stamp, frame_id
Voltage: float32                   # 볼트(V)
Temperature: float32               # 섭씨(°C)
Current: float32                   # 앰프(A) - 음수는 방전
Charge: float32                    # 쿨롱(C)
Capacity: float32                  # 쿨롱(C)
Design_capacity: float32           # 설계 용량 (쿨롱)
Percentage: float32                # 백분율 (0.0 ~ 1.0)
Power_supply_status: uint8         # 0: unknown, 1: charging, 2: discharging
Power_supply_health: uint8         # 0: unknown, 1: good, 2: overheat
Power_supply_technology: uint8     # 0: unknown, 1: NiMH, 2: LiIon
Present: bool                      # 배터리 연결 상태
Cell_voltage: float32[]            # 각 셀 전압
Cell_temperature: float32[]        # 각 셀 온도
Location: string                   # 배터리 위치 설명
Serial_number: string              # 일련번호
```

## 2. MATLAB 데이터 구조 정의

### 2.1 항공기 상태 (AircraftState)

```matlab
struct AircraftState
    % 위치 정보
    position
        latitude: double            % 도 (°)
        longitude: double           % 도 (°)
        altitude: double            % 미터 (AMSL)
        x: double                   % 지역 좌표 X (미터)
        y: double                   % 지역 좌표 Y (미터)
        z: double                   % 지역 좌표 Z (미터)
    
    % 속도 정보
    velocity
        vx: double                  % m/s (X축 속도)
        vy: double                  % m/s (Y축 속도)
        vz: double                  % m/s (Z축 속도)
        speed: double               % m/s (스칼라 속력)
    
    % 자세 정보
    attitude
        roll: double                % 라디안 (±π)
        pitch: double               % 라디안 (±π/2)
        yaw: double                 % 라디안 (±π)
    
    % IMU 센서
    imu
        accel_x: double             % m/s²
        accel_y: double
        accel_z: double
        gyro_x: double              % rad/s
        gyro_y: double
        gyro_z: double
    
    % GPS 정보
    gps
        status: int32               % 0: no fix, 2: 3D fix
        num_satellites: int32
    
    % 기압 정보
    barometer
        pressure: double            % 파스칼 (Pa)
        altitude: double            % 미터 (기압 기반 계산)
    
    % 배터리 정보
    battery
        percent: double             % 0-100 (%)
        voltage: double             % 볼트 (V)
        current: double             % 앰프 (A)
        charge: double              % 쿨롱 (C)
        capacity: double            % 쿨롱 (C)
        design_capacity: double     % 쿨롱 (C)
    
    % 메타데이터
    timestamp: datetime             % 데이터 수집 시간
    is_valid: boolean               % 데이터 유효성
end
```

### 2.2 날씨 데이터 (WeatherData)

```matlab
struct WeatherData
    latitude: double                % 도 (°)
    longitude: double               % 도 (°)
    timestamp: datetime
    
    % 온도 및 습도
    temperature: double             % 섭씨 (°C)
    dew_point: double               % 섭씨 (°C)
    humidity: double                % 백분율 (0-100)
    
    % 기압
    pressure: double                % 파스칼 (Pa)
    
    % 바람
    wind_speed: double              % m/s
    wind_direction: double          % 도 (0-360), 0=북쪽
    wind_gust: double               % m/s (돌풍 속도)
    
    % 강수
    precipitation: double           % mm
    precipitation_type: string      % '비', '눈', '없음'
    
    % 시정
    visibility: double              % 미터 (m)
end
```

### 2.3 의사결정 정보 (DecisionInfo)

```matlab
struct DecisionInfo
    timestamp: datetime
    current_state: AircraftState
    current_weather: WeatherData
    
    % 후보 충전소 정보
    num_candidates: int32
    best_station
        station_id: int32
        station_name: string
        distance: double            % km
        altitude_diff: double       % 미터
        expected_battery: double    % % (도착 시 예상 배터리)
        
        % 점수 정보
        distance_score: double
        altitude_score: double
        battery_score: double
        weather_score: double
        difficulty_score: double
        charging_score: double
        total_score: double
    
    all_candidates: array of struct % 모든 후보 정보
end
```

## 3. 좌표계 정의

### 3.1 GPS 좌표계
- **위도 (Latitude)**: -90° ~ +90° (남쪽 음수, 북쪽 양수)
- **경도 (Longitude)**: -180° ~ +180° (서쪽 음수, 동쪽 양수)
- **고도 (Altitude)**: AMSL (Average Mean Sea Level) 기준

### 3.2 항공기 체계 좌표계 (Body Frame)
```
        ↑ Z (Up)
        
    X → (Forward)
    
Y → (Right)
```

- **Roll (Φ)**: X축 중심 회전 (우측 스틱)
- **Pitch (θ)**: Y축 중심 회전 (전후 스틱)
- **Yaw (Ψ)**: Z축 중심 회전 (좌우 러더)

### 3.3 지역 좌표계 (Local NED)
- **N**: 북쪽 방향
- **E**: 동쪽 방향
- **D**: 아래쪽 (고도의 음수)

**변환 공식**:
```matlab
% GPS → NED (근처 지점)
lat0 = 37.5665; lon0 = 126.9780;  % 기준점 (서울)
R = 6371e3;  % 지구 반지름 (m)

dlat = (lat - lat0) * pi / 180;
dlon = (lon - lon0) * pi / 180;

N = R * dlat;
E = R * dlon * cos(lat0 * pi / 180);
D = -(alt - alt0);  % 아래쪽이 양수
```

## 4. 배터리 모델링

### 4.1 배터리 소모 모델
```
배터리 소모율 = 기본 소비 + 비행 조건별 추가 소비

기본 소비 (유휴): ~10 mA
순항 비행: ~50-100 mA
가속/급상승: ~200-300 mA
```

### 4.2 남은 시간 계산
```matlab
battery_percent = 75;           % %
battery_capacity = 5000;        % mAh
consumption_rate = 75;          % mAh/min (순항 + 추가)

remaining_mah = battery_percent / 100 * battery_capacity;
remaining_time_min = remaining_mah / consumption_rate;
remaining_time_hour = remaining_time_min / 60;

% 안전 마진 적용 (15%)
safe_remaining = remaining_time_hour * 0.85;
```

### 4.3 도달 가능 거리
```matlab
cruise_speed = 20;              % m/s
max_flight_time = safe_remaining_time * 3600;  % 초
max_distance_km = (cruise_speed * max_flight_time) / 1000;
```

## 5. 의사결정 알고리즘

### 5.1 충전소 선택 점수 (0 ~ 1)

$$\text{total\_score} = w_d \cdot d + w_a \cdot a + w_b \cdot b + w_w \cdot w + w_{diff} \cdot diff + w_c \cdot c$$

- $d$: 거리 점수 = $\frac{1}{1 + \text{distance(km)}}$
- $a$: 고도 차이 점수 = $\frac{1}{1 + \text{altitude\_diff}/100}$
- $b$: 배터리 도달 가능성 = $\frac{\text{expected\_battery}}{100}$ (도달 불가능 시 0)
- $w$: 날씨 점수 = $(1 + \text{wind\_factor}) / 2 \times \text{normalized\_wind}$
- $diff$: 착륙 난이도 = $\frac{1}{1 + \text{difficulty}}$
- $c$: 충전 속도 = $\frac{\text{charging\_rate}}{200}$

**기본 가중치**:
- $w_d = 0.25$ (거리 중요도)
- $w_a = 0.10$ (고도 차이)
- $w_b = 0.25$ (배터리 우선)
- $w_w = 0.15$ (날씨)
- $w_{diff} = 0.15$ (착륙 난이도)
- $w_c = 0.10$ (충전 속도)

### 5.2 경로 가능성 판정

```
도달 가능 거리 > 실제 거리 + 20% 안전 마진 = 직접 도달 가능
도달 가능 거리 < 실제 거리 = 충전 정지 필요
```

## 6. 날씨의 항공기 성능 영향

### 6.1 양력에 미치는 영향

$$L = \frac{1}{2} \rho v^2 S C_L$$

- $\rho$: 공기 밀도 (온도, 기압 함수)
- $v$: 항속
- $S$: 날개 면적
- $C_L$: 양력 계수

**수정 계수**:
- 온도 ±10°C: ~3% 양력 변화
- 기압 변화: $\rho$ 에 정비례
- 습도: 부차적 (극한 값에서만 영향)

### 6.2 항속에 미치는 영향

**바람 효과**:
```
지상속도 = 항속 + 바람 속도 (vector)

역풍: 지상속도 감소 → 비행 시간 증가
순풍: 지상속도 증가 → 비행 시간 감소
```

## 7. 테스트 시나리오

### 시나리오 1: 정상 날씨, 충전소 도달 가능
```
배터리: 80%
바람: 5 m/s (순풍)
시정: 10 km
예상 결과: 최단거리 충전소 선택
```

### 시나리오 2: 악천후, 배터리 부족
```
배터리: 30%
바람: 15 m/s (역풍)
강수: 5 mm
시정: 2 km
예상 결과: 안전성을 우선으로 가장 가까운 충전소 선택
```

### 시나리오 3: 장거리 항행
```
현재 위치 → 목적지 거리: 150 km
배터리: 100%
날씨: 정상
예상 결과: 3개 충전소 경유하는 경로 계획
```
