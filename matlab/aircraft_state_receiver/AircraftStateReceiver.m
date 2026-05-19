%% AircraftStateReceiver - ROS2 Aircraft State Receiver
% ROS2 토픽에서 항공기 상태 정보를 수신하는 클래스

classdef AircraftStateReceiver < handle
    properties (Access = private)
        subscriber_node
        subscriber_tf
        subscriber_imu
        subscriber_gps
        subscriber_geopose
        subscriber_baro
        subscriber_battery
        subscriber_airspeed
        subscriber_wind
        subscriber_odometry
        
        current_state
        state_updated = false
        altitude_reference_m = NaN
        
        data_logger
        last_update_time
    end
    
    properties (Access = public)
        is_subscribed = false
        verbose = false
    end
    
    methods
        function obj = AircraftStateReceiver(varargin)
            %% AircraftStateReceiver 생성자
            p = inputParser;
            addParameter(p, 'verbose', false, @islogical);
            addParameter(p, 'enable_logging', true, @islogical);
            parse(p, varargin{:});
            
            obj.verbose = p.Results.verbose;
            obj.last_update_time = tic;
            obj.current_state = obj.create_empty_state();
            
            if obj.verbose
                fprintf('[AircraftStateReceiver] Initialized\n');
            end
        end
        
        function subscribe_to_topics(obj)
            %% ROS2 토픽 구독 시작
            if obj.verbose
                fprintf('[AircraftStateReceiver] Starting ROS2 topic subscription...\n');
            end
            
            try
                % ROS2 노드 생성
                obj.subscriber_node = ros2node('/aircraft_state_receiver');
                
                % SimulationConfig에서 토픽 이름 확인
                ros2_mgr = ROS2Manager('verbose', obj.verbose);
                ros2_mgr = ros2_mgr.Initialize();
                current_ns = ros2_mgr.get_namespace();
                
                % 토픽 이름 해석
                resolved_tf = SimulationConfig.resolve_topic('tf', current_ns);
                resolved_imu = SimulationConfig.resolve_topic('imu', current_ns);
                resolved_odom = SimulationConfig.resolve_topic('odometry', current_ns);
                resolved_baro = SimulationConfig.resolve_topic('baro', current_ns);
                resolved_battery = SimulationConfig.resolve_topic('battery', current_ns);
                resolved_airspeed = SimulationConfig.resolve_topic('airspeed', current_ns);
                resolved_wind = [current_ns '/wind_sensor'];
                
                % GPS: 직접 경로 구성 (resolve_topic_if_available 타이밍 문제 회피)
                % 우선순위: /iris/navsat → /iris/gpsfix → 다른 후보들
                gps_candidates = {
                    [current_ns '/navsat/navsat0'], [current_ns '/navsat'], ...
                    '/iris/navsat/navsat0', '/iris/navsat', '/navsat', ...
                    [current_ns '/gpsfix'], '/iris/gpsfix', '/gps', ...
                    [current_ns '/gps/sample'], '/gps/sample'
                };
                resolved_gps = '';
                for j = 1:length(gps_candidates)
                    if obj.topic_exists_check(gps_candidates{j})
                        resolved_gps = gps_candidates{j};
                        break;
                    end
                end
                
                % GEOPOSE: 역시 직접 경로 구성
                geopose_candidates = {
                    [current_ns '/geopose/filtered'], [current_ns '/geopose'], ...
                    '/iris/geopose/filtered', '/iris/geopose', ...
                    '/ap/geopose/filtered', '/ap/geopose'
                };
                resolved_geopose = '';
                for j = 1:length(geopose_candidates)
                    if obj.topic_exists_check(geopose_candidates{j})
                        resolved_geopose = geopose_candidates{j};
                        break;
                    end
                end
                
                % 구독자 생성
                obj.subscriber_tf = obj.try_subscribe(resolved_tf, 'tf2_msgs/TFMessage', @obj.callback_tf);
                obj.subscriber_imu = obj.try_subscribe(resolved_imu, 'sensor_msgs/Imu', @obj.callback_imu);
                obj.subscriber_odometry = obj.try_subscribe(resolved_odom, 'nav_msgs/Odometry', @obj.callback_odom);
                
                % GPS 구독 (NavSatFix 우선)
                if ~isempty(resolved_gps)
                    obj.subscriber_gps = obj.try_subscribe(resolved_gps, 'sensor_msgs/NavSatFix', @obj.callback_gps);
                    % NavSatFix 실패시 GPSFix fallback
                    if isempty(obj.subscriber_gps)
                        obj.subscriber_gps = obj.try_subscribe(resolved_gps, 'gps_msgs/GPSFix', @obj.callback_gps);
                    end
                end

                if ~isempty(resolved_geopose)
                    obj.subscriber_geopose = obj.try_subscribe(resolved_geopose, 'geographic_msgs/GeoPoseStamped', @obj.callback_geopose);
                end
                
                obj.subscriber_baro = obj.try_subscribe(resolved_baro, 'sensor_msgs/FluidPressure', @obj.callback_baro);
                obj.subscriber_battery = obj.try_subscribe(resolved_battery, 'sensor_msgs/BatteryState', @obj.callback_battery);
                obj.subscriber_airspeed = obj.try_subscribe(resolved_airspeed, 'ardupilot_msgs/Airspeed', @obj.callback_airspeed);
                if isempty(obj.subscriber_airspeed)
                    obj.subscriber_airspeed = obj.try_subscribe(resolved_airspeed, 'geometry_msgs/Vector3Stamped', @obj.callback_airspeed);
                end
                obj.subscriber_wind = obj.try_subscribe(resolved_wind, 'geometry_msgs/Vector3Stamped', @obj.callback_wind);
                
                % 구독 성공 여부 확인
                obj.is_subscribed = ~isempty(obj.subscriber_tf) || ~isempty(obj.subscriber_imu) || ...
                    ~isempty(obj.subscriber_odometry);
                
                if obj.verbose
                    fprintf('[AircraftStateReceiver] Subscription status:\n');
                    fprintf('  TF: %s\n', resolved_tf);
                    fprintf('  IMU: %s\n', resolved_imu);
                    fprintf('  ODOM: %s\n', resolved_odom);
                    fprintf('  GPS/NAVSAT: %s\n', resolved_gps);
                    fprintf('  GEOPOSE: %s\n', resolved_geopose);
                    fprintf('  BARO: %s\n', resolved_baro);
                    fprintf('  BATTERY: %s\n', resolved_battery);
                    fprintf('  AIRSPEED: %s\n', resolved_airspeed);
                    fprintf('  WIND SENSOR: %s\n', resolved_wind);
                end
                
                if ~obj.is_subscribed
                    warning('[AircraftStateReceiver] No topics successfully subscribed');
                end
                
            catch ME
                fprintf('[AircraftStateReceiver] Subscription failed: %s\n', ME.message);
                rethrow(ME);
            end
        end
        
        function state = get_current_state(obj)
            %% 현재 항공기 상태 반환
            state = obj.current_state;
            state.is_valid = obj.state_updated;
        end
        
        function print_state(obj)
            %% 항공기 상태 출력
            state = obj.current_state;
            
            fprintf('\n════════════════════════════════\n');
            fprintf('Aircraft State Summary\n');
            fprintf('════════════════════════════════\n');
            
            fprintf('\nPosition:\n');
            fprintf('  Latitude:  %.6f°\n', state.position.latitude);
            fprintf('  Longitude: %.6f°\n', state.position.longitude);
            fprintf('  Altitude (relative):  %.2f m\n', state.position.altitude);
            if isfinite(state.position.absolute_altitude)
                fprintf('  Altitude (absolute):  %.2f m\n', state.position.absolute_altitude);
            end
            
            fprintf('\nVelocity:\n');
            fprintf('  Vx: %.2f m/s, Vy: %.2f m/s, Vz: %.2f m/s\n', ...
                state.velocity.vx, state.velocity.vy, state.velocity.vz);
            fprintf('  Speed: %.2f m/s\n', state.velocity.speed);
            
            fprintf('\nAttitude:\n');
            fprintf('  Roll:  %.2f°\n', rad2deg(state.attitude.roll));
            fprintf('  Pitch: %.2f°\n', rad2deg(state.attitude.pitch));
            fprintf('  Yaw:   %.2f°\n', rad2deg(state.attitude.yaw));
            
            fprintf('════════════════════════════════\n\n');
        end
        
        function cleanup(obj)
            %% 리소스 정리
            try
                if ~isempty(obj.subscriber_node)
                    clear obj.subscriber_node;
                end
                if obj.verbose
                    fprintf('[AircraftStateReceiver] topic subscribe release\n');
                end
            catch
            end
        end

        function unsubscribe_from_topics(obj)
            %% 기존 코드 호환용 별칭
            obj.cleanup();
        end
    end
    
    methods (Access = private)
        function exists = topic_exists_check(obj, topic_name)
            %% ROS2 토픽 존재 여부 빠른 확인 (캐시 사용)
            persistent topic_cache last_check_time
            if isempty(topic_cache)
                topic_cache = containers.Map('KeyType', 'char', 'ValueType', 'logical');
                last_check_time = now;
            end
            
            % 캐시 유효성 (30초)
            if (now - last_check_time) > 30/(24*3600)
                topic_cache = containers.Map('KeyType', 'char', 'ValueType', 'logical');
                last_check_time = now;
            end
            
            % 캐시 확인
            if isKey(topic_cache, topic_name)
                exists = topic_cache(topic_name);
                return;
            end
            
            % ros2 topic list 실행 및 확인
            exists = false;
            try
                [status, output] = system('ros2 topic list 2>/dev/null');
                if status == 0 && ~isempty(strtrim(output))
                    topics = strsplit(strtrim(output), newline);
                    exists = any(strcmp(strtrim(topics), strtrim(topic_name)));
                end
            catch
            end
            
            % 캐시에 저장
            topic_cache(topic_name) = exists;
        end
        
        function sub = try_subscribe(obj, topic_name, message_type, callback_handle)
            %% ROS2 구독 시도
            sub = [];
            try
                sub = ros2subscriber(obj.subscriber_node, topic_name, message_type, callback_handle, ...
                    "Reliability", "besteffort", ...
                    "Durability", "volatile");
                
                if obj.verbose
                    fprintf('[AircraftStateReceiver] subscribed: %s [%s]\n', topic_name, message_type);
                end
            catch ME
                fprintf('[AircraftStateReceiver] subscribe failed: %s [%s]\n', topic_name, message_type);
                if obj.verbose
                    fprintf('  reason: %s\n', ME.message);
                end
            end
        end
        
        function callback_tf(obj, msg)
            %% TF 콜백 - Transform 메시지 처리
            try
                % 위치 업데이트
                obj.current_state.position.x = msg.transforms(1).transform.translation.x;
                obj.current_state.position.y = msg.transforms(1).transform.translation.y;
                obj.current_state.position.z = msg.transforms(1).transform.translation.z;
                
                % Quaternion to Euler 각도 변환
                quat = [msg.transforms(1).transform.rotation.w, ...
                    msg.transforms(1).transform.rotation.x, ...
                    msg.transforms(1).transform.rotation.y, ...
                    msg.transforms(1).transform.rotation.z];
                
                [roll, pitch, yaw] = quat2euler(quat);
                obj.current_state.attitude.roll = roll;
                obj.current_state.attitude.pitch = pitch;
                obj.current_state.attitude.yaw = yaw;
                
                obj.current_state.timestamp = now;
                obj.state_updated = true;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] TF error: %s\n', ME.message);
                end
            end
        end
        
        function callback_imu(obj, msg)
            %% IMU 콜백
            try
                obj.current_state.imu.accel_x = msg.linear_acceleration.x;
                obj.current_state.imu.accel_y = msg.linear_acceleration.y;
                obj.current_state.imu.accel_z = msg.linear_acceleration.z;
                
                obj.current_state.imu.gyro_x = msg.angular_velocity.x;
                obj.current_state.imu.gyro_y = msg.angular_velocity.y;
                obj.current_state.imu.gyro_z = msg.angular_velocity.z;
                
                obj.current_state.timestamp = now;
                obj.state_updated = true;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] IMU error: %s\n', ME.message);
                end
            end
        end
        
        function callback_odom(obj, msg)
            %% Odometry 콜백
            try
                obj.current_state.position.x = msg.pose.pose.position.x;
                obj.current_state.position.y = msg.pose.pose.position.y;
                obj.current_state.position.z = msg.pose.pose.position.z;
                if isnan(obj.altitude_reference_m) && isfinite(msg.pose.pose.position.z)
                    obj.altitude_reference_m = msg.pose.pose.position.z;
                end
                if isfinite(obj.altitude_reference_m)
                    obj.current_state.position.relative_altitude = msg.pose.pose.position.z - obj.altitude_reference_m;
                    obj.current_state.position.altitude = obj.current_state.position.relative_altitude;
                else
                    obj.current_state.position.relative_altitude = msg.pose.pose.position.z;
                    obj.current_state.position.altitude = msg.pose.pose.position.z;
                end
                if ~isfield(obj.current_state.position, 'has_absolute_position') || ~obj.current_state.position.has_absolute_position
                    obj.current_state.position.position_source = 'odometry';
                end
                
                obj.current_state.velocity.vx = msg.twist.twist.linear.x;
                obj.current_state.velocity.vy = msg.twist.twist.linear.y;
                obj.current_state.velocity.vz = msg.twist.twist.linear.z;
                obj.current_state.velocity.speed = sqrt(obj.current_state.velocity.vx^2 + ...
                    obj.current_state.velocity.vy^2 + obj.current_state.velocity.vz^2);
                
                obj.current_state.timestamp = now;
                obj.state_updated = true;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] ODOM error: %s\n', ME.message);
                end
            end
        end
        
        function callback_gps(obj, msg)
            %% GPS 콜백
            try
                obj.current_state.position.latitude = msg.latitude;
                obj.current_state.position.longitude = msg.longitude;
                obj.current_state.position.absolute_altitude = msg.altitude;
                obj.current_state.position.has_absolute_position = true;
                obj.current_state.position.position_source = 'navsat';
                if ~isfinite(obj.current_state.position.altitude)
                    if isnan(obj.altitude_reference_m) && isfinite(msg.altitude)
                        obj.altitude_reference_m = msg.altitude;
                    end
                    if isfinite(obj.altitude_reference_m)
                        obj.current_state.position.altitude = msg.altitude - obj.altitude_reference_m;
                    else
                        obj.current_state.position.altitude = msg.altitude;
                    end
                end
                
                if isprop(msg, 'status')
                    if isprop(msg.status, 'status')
                        obj.current_state.gps.status = msg.status.status;
                    end
                    if isprop(msg.status, 'satellites_used')
                        obj.current_state.gps.num_satellites = msg.status.satellites_used;
                    elseif isprop(msg.status, 'service')
                        obj.current_state.gps.num_satellites = msg.status.service;
                    end
                end
                
                obj.current_state.timestamp = now;
                obj.state_updated = true;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] GPS error: %s\n', ME.message);
                end
            end
        end

        function callback_geopose(obj, msg)
            %% GeoPose 콜백 - absolute LLH 위치 우선 사용
            try
                obj.current_state.position.latitude = msg.pose.position.latitude;
                obj.current_state.position.longitude = msg.pose.position.longitude;
                obj.current_state.position.absolute_altitude = msg.pose.position.altitude;
                obj.current_state.position.has_absolute_position = true;
                obj.current_state.position.position_source = 'geopose';
                if ~isfinite(obj.current_state.position.altitude)
                    if isnan(obj.altitude_reference_m) && isfinite(msg.pose.position.altitude)
                        obj.altitude_reference_m = msg.pose.position.altitude;
                    end
                    if isfinite(obj.altitude_reference_m)
                        obj.current_state.position.altitude = msg.pose.position.altitude - obj.altitude_reference_m;
                    else
                        obj.current_state.position.altitude = msg.pose.position.altitude;
                    end
                end

                obj.current_state.timestamp = now;
                obj.state_updated = true;

            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] GEOPOSE error: %s\n', ME.message);
                end
            end
        end
        
        function callback_baro(obj, msg)
            %% Barometer (기압) 콜백
            try
                obj.current_state.barometer.pressure = msg.fluid_pressure;
                obj.current_state.barometer.altitude = ...
                    44330 * (1 - (msg.fluid_pressure / 101325) ^ (1/5.255));
                
                obj.current_state.timestamp = now;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] BARO error: %s\n', ME.message);
                end
            end
        end
        
        function callback_battery(obj, msg)
            %% Battery 콜백
            try
                if isfield(msg, 'percentage')
                    obj.current_state.battery.percent = msg.percentage * 100;
                elseif isfield(msg, 'Percentage')
                    obj.current_state.battery.percent = msg.Percentage * 100;
                else
                    obj.current_state.battery.percent = 50;
                end
                
                obj.current_state.battery.voltage = msg.voltage;
                obj.current_state.battery.current = msg.current;
                obj.current_state.battery.charge = msg.charge;
                obj.current_state.battery.capacity = msg.capacity;
                obj.current_state.battery.design_capacity = msg.design_capacity;
                
                obj.current_state.timestamp = now;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] BATTERY error: %s\n', ME.message);
                end
            end
        end
        
        function callback_airspeed(obj, msg)
            %% Airspeed 콜백
            try
                if isfield(msg, 'true_airspeed')
                    obj.current_state.airspeed.true_airspeed_x = msg.true_airspeed.x;
                    obj.current_state.airspeed.true_airspeed_y = msg.true_airspeed.y;
                    obj.current_state.airspeed.true_airspeed_z = msg.true_airspeed.z;
                elseif isprop(msg, 'vector')
                    obj.current_state.airspeed.true_airspeed_x = msg.vector.x;
                    obj.current_state.airspeed.true_airspeed_y = msg.vector.y;
                    obj.current_state.airspeed.true_airspeed_z = msg.vector.z;
                else
                    obj.current_state.airspeed.true_airspeed_x = 0;
                    obj.current_state.airspeed.true_airspeed_y = 0;
                    obj.current_state.airspeed.true_airspeed_z = 0;
                end
                if isfield(msg, 'eas_2_tas')
                    obj.current_state.airspeed.eas_2_tas = msg.eas_2_tas;
                else
                    obj.current_state.airspeed.eas_2_tas = 1.0;
                end
                
                obj.current_state.timestamp = now;
                obj.state_updated = true;
                
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] AIRSPEED error: %s\n', ME.message);
                end
            end
        end

        function callback_wind(obj, msg)
            %% Virtual wind sensor callback
            try
                if isfield(msg, 'vector') || isprop(msg, 'vector')
                    obj.current_state.wind.x = msg.vector.x;
                    obj.current_state.wind.y = msg.vector.y;
                    obj.current_state.wind.z = msg.vector.z;
                    obj.current_state.wind.magnitude = norm([msg.vector.x, msg.vector.y, msg.vector.z]);
                end
                obj.current_state.timestamp = now;
                obj.state_updated = true;
            catch ME
                if obj.verbose
                    fprintf('[AircraftStateReceiver] WIND error: %s\n', ME.message);
                end
            end
        end

        function state = create_empty_state(obj)
            %% 빈 상태 구조체 생성
            state = struct();
            
            state.position = struct(...
                'latitude', NaN, ...
                'longitude', NaN, ...
                'altitude', NaN, ...
                'absolute_altitude', NaN, ...
                'relative_altitude', NaN, ...
                'has_absolute_position', false, ...
                'position_source', 'unknown', ...
                'x', NaN, ...
                'y', NaN, ...
                'z', NaN ...
            );
            
            state.velocity = struct(...
                'vx', 0, ...
                'vy', 0, ...
                'vz', 0, ...
                'speed', 0 ...
            );
            
            state.attitude = struct(...
                'roll', 0, ...
                'pitch', 0, ...
                'yaw', 0 ...
            );
            
            state.imu = struct(...
                'accel_x', 0, ...
                'accel_y', 0, ...
                'accel_z', 0, ...
                'gyro_x', 0, ...
                'gyro_y', 0, ...
                'gyro_z', 0 ...
            );
            
            state.gps = struct(...
                'status', 0, ...
                'num_satellites', 0 ...
            );
            
            state.barometer = struct(...
                'pressure', 101325, ...
                'altitude', 0 ...
            );
            
            state.battery = struct(...
                'percent', 100, ...
                'voltage', 12.0, ...
                'current', 0, ...
                'charge', 5000, ...
                'capacity', 5000, ...
                'design_capacity', 5000 ...
            );
            
            state.airspeed = struct(...
                'true_airspeed_x', 0, ...
                'true_airspeed_y', 0, ...
                'true_airspeed_z', 0, ...
                'eas_2_tas', 1 ...
            );

            state.wind = struct(...
                'x', 0, ...
                'y', 0, ...
                'z', 0, ...
                'magnitude', 0 ...
            );
            
            state.timestamp = now;
            state.is_valid = false;
        end
    end
end

%% Helper: Quaternion to Euler angles
function [roll, pitch, yaw] = quat2euler(quat)
    % quat = [w, x, y, z]
    w = quat(1);
    x = quat(2);
    y = quat(3);
    z = quat(4);
    
    % Roll
    sinr_cosp = 2 * (w * x + y * z);
    cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);
    
    % Pitch
    sinp = 2 * (w * y - z * x);
    if abs(sinp) >= 1
        pitch = pi/2 * sign(sinp);
    else
        pitch = asin(sinp);
    end
    
    % Yaw
    siny_cosp = 2 * (w * z + x * y);
    cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
end
