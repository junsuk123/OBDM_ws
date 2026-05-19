%% DataCollectionModule - Sensor Data Collection
% 센서 데이터 수집 전담 모듈
% 
% 용도: IMU, GPS, Odometry, TF, Battery, Airspeed 등 센서 데이터 수집
% 
% 사용법:
%   collector = DataCollectionModule('namespace', '/alti_transition');
%   collector = collector.subscribe_to_topics();
%   data = collector.collect_data(10);  % 10초 수집
%   collector.print_summary(data);
%   collector.unsubscribe_from_topics();

classdef DataCollectionModule < handle
    properties (Access = private)
        namespace
        world_name
        robot_name
        ros_node
        subscribers
        collected_data
        verbose
        gps_publisher
    end
    
    properties (Access = public)
        is_initialized = false
    end
    
    methods
        function obj = DataCollectionModule(varargin)
            % 생성자
            p = inputParser;
            addParameter(p, 'namespace', '/alti_transition', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});
            
            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
            obj.world_name = getenv('OBDM_WORLD_NAME');
            if isempty(obj.world_name)
                obj.world_name = 'runway';
            end
            obj.robot_name = regexprep(obj.namespace, '^/', '');
            if isempty(obj.robot_name)
                obj.robot_name = 'iris';
            end
            obj.subscribers = struct();
            obj.collected_data = struct();
            obj.collected_data.battery = [];
            obj.collected_data.airspeed = [];
            obj.collected_data.wind = [];
            obj.gps_publisher = [];
            
            if obj.verbose
                fprintf('[DataCollectionModule] Initialized (namespace: %s)\n', obj.namespace);
            end
        end
        
        function obj = subscribe_to_topics(obj)
            % ROS2 노드 생성 및 모든 센서 구독
            try
                obj.ros_node = ros2node("/matlab_data_collector");
                
                % IMU subscriber
                try
                    resolved_imu = SimulationConfig.resolve_topic('imu', obj.namespace);
                    obj.subscribers.imu = ros2subscriber(obj.ros_node, resolved_imu, "sensor_msgs/Imu");
                    if obj.verbose
                        fprintf('  ✓ IMU: %s\n', resolved_imu);
                    end
                catch ME
                    if obj.verbose
                        fprintf('  ✗ IMU subscriber failed: %s\n', ME.message);
                    end
                end
                
                % GPS subscriber
                try
                    resolved_gps = SimulationConfig.resolve_topic('navsat', obj.namespace);
                    obj.subscribers.gps = ros2subscriber(obj.ros_node, resolved_gps, "sensor_msgs/NavSatFix");
                    if obj.verbose
                        fprintf('  ✓ GPS: %s\n', resolved_gps);
                    end
                catch ME
                    if obj.verbose
                        fprintf('  ✗ GPS subscriber failed: %s\n', ME.message);
                    end
                end
                
                % Odometry subscriber
                try
                    resolved_odom = SimulationConfig.resolve_topic('odometry', obj.namespace);
                    obj.subscribers.odom = ros2subscriber(obj.ros_node, resolved_odom, "nav_msgs/Odometry");
                    if obj.verbose
                        fprintf('  ✓ ODOM: %s\n', resolved_odom);
                    end
                catch ME
                    if obj.verbose
                        fprintf('  ✗ ODOM subscriber failed: %s\n', ME.message);
                    end
                end
                
                % TF subscriber
                try
                    resolved_tf = SimulationConfig.resolve_topic('tf', obj.namespace);
                    obj.subscribers.tf = ros2subscriber(obj.ros_node, resolved_tf, "tf2_msgs/TFMessage");
                    if obj.verbose
                        fprintf('  ✓ TF: %s\n', resolved_tf);
                    end
                catch ME
                    if obj.verbose
                        fprintf('  ✗ TF subscriber failed: %s\n', ME.message);
                    end
                end
                
                % Battery subscriber
                try
                    resolved_battery = SimulationConfig.resolve_topic('battery', obj.namespace);
                    obj.subscribers.battery = ros2subscriber(obj.ros_node, resolved_battery, "sensor_msgs/BatteryState");
                    if obj.verbose
                        fprintf('  ✓ Battery: %s\n', resolved_battery);
                    end
                catch ME
                    if obj.verbose
                        fprintf('  ✗ Battery subscriber failed: %s\n', ME.message);
                    end
                end
                
                % Airspeed subscriber (optional - may not be available in all bridges)
                try
                    resolved_airspeed = SimulationConfig.resolve_topic('airspeed', obj.namespace);
                    try
                        obj.subscribers.airspeed = ros2subscriber(obj.ros_node, resolved_airspeed, "ardupilot_msgs/Airspeed");
                    catch
                        obj.subscribers.airspeed = ros2subscriber(obj.ros_node, resolved_airspeed, "geometry_msgs/Vector3Stamped");
                    end
                    if obj.verbose
                        fprintf('  ✓ Airspeed: %s\n', resolved_airspeed);
                    end
                catch ME
                    % Airspeed often not available - gracefully handle
                    if obj.verbose
                        fprintf('  ⚠ Airspeed not available (optional): %s\n', ME.message);
                    end
                end

                % Virtual wind sensor: apparent wind vector acting on vehicle.
                try
                    resolved_wind = sprintf('%s/wind_sensor', obj.namespace);
                    obj.subscribers.wind = ros2subscriber(obj.ros_node, resolved_wind, "geometry_msgs/Vector3Stamped");
                    if obj.verbose
                        fprintf('  ✓ Wind sensor: %s\n', resolved_wind);
                    end
                catch ME
                    if obj.verbose
                        fprintf('  ⚠ Wind sensor not available (optional): %s\n', ME.message);
                    end
                end
                
                obj.is_initialized = true;
                
                    % GPS 테스트 데이터 퍼블리셔
                    try
                        obj.gps_publisher = ros2publisher(obj.ros_node, sprintf('%s/navsat', obj.namespace), "sensor_msgs/NavSatFix");
                        if obj.verbose
                            fprintf('  ✓ GPS Publisher created (test data)\n');
                        end
                    catch
                    end
            catch ME
                fprintf('  ✗ Subscription initialization failed: %s\n', ME.message);
            end
        end
        
        function data = collect_data(obj, duration_sec)
            % 지정된 시간 동안 센서 데이터 수집
            % 입력: duration_sec - 수집 시간(초)
            % 출력: data - 수집된 데이터 구조체
            
            if ~obj.is_initialized
                error('DataCollectionModule: 먼저 subscribe_to_topics() 호출 필요');
            end
            
            % 데이터 초기화
            data.imu = [];
            data.gps = [];
            data.odom = [];
            data.tf = [];
            data.battery = [];
            data.airspeed = [];
            data.wind = [];

            last_stamp.imu = NaN;
            last_stamp.gps = NaN;
            last_stamp.odom = NaN;
            last_stamp.tf = NaN;
            last_stamp.battery = NaN;
            last_stamp.airspeed = NaN;
            last_stamp.wind = NaN;
            
            if obj.verbose
                fprintf('\n[DataCollectionModule] Collecting data for %d seconds...\n', duration_sec);
            end
            
            for t = 1:duration_sec
                got_airspeed = false;

                % GPS 테스트 데이터 발행 (수집과 동시에)
                if ~isempty(obj.gps_publisher)
                    try
                        gps_msg = ros2message('sensor_msgs/NavSatFix');
                        gps_msg.header.stamp = rostime('now');
                        gps_msg.header.frame_id = 'gps_link';
                        gps_msg.latitude = 37.4979 + (rand() - 0.5) * 0.0001;  % 테스트 위도
                        gps_msg.longitude = 127.0276 + (rand() - 0.5) * 0.0001;  % 테스트 경도
                        gps_msg.altitude = 45.0 + (rand() - 0.5) * 10;  % 테스트 고도
                        gps_msg.position_covariance = diag([0.01, 0.01, 0.01]);
                        gps_msg.position_covariance_type = 2;
                        gps_msg.status.status = 0;
                        send(obj.gps_publisher, gps_msg);
                    catch
                    end
                end
                
                % GPS 빠른 발행 (5회 반복)
                for gps_iter = 1:5
                    if ~isempty(obj.gps_publisher)
                        try
                            gps_msg = ros2message('sensor_msgs/NavSatFix');
                            gps_msg.header.stamp = rostime('now');
                            gps_msg.header.frame_id = 'gps_link';
                            gps_msg.latitude = 37.4979 + (rand() - 0.5) * 0.0001;
                            gps_msg.longitude = 127.0276 + (rand() - 0.5) * 0.0001;
                            gps_msg.altitude = 45.0 + (rand() - 0.5) * 10;
                            gps_msg.position_covariance = diag([0.01, 0.01, 0.01]);
                            gps_msg.position_covariance_type = 2;
                            gps_msg.status.status = 0;
                            send(obj.gps_publisher, gps_msg);
                        catch
                        end
                    end
                    pause(0.1);
                end
                
                % IMU
                if isfield(obj.subscribers, 'imu') && ~isempty(obj.subscribers.imu)
                    try
                        [msg, status] = receive(obj.subscribers.imu, 1.0);
                        if status && ~isempty(msg)
                            stamp = obj.get_msg_stamp(msg);
                            if isnan(last_stamp.imu) || stamp ~= last_stamp.imu
                                data.imu = [data.imu; msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, ...
                                    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z];
                                last_stamp.imu = stamp;
                            end
                        end
                    catch
                    end
                end
                
                % GPS
                if isfield(obj.subscribers, 'gps') && ~isempty(obj.subscribers.gps)
                    try
                        [msg, status] = receive(obj.subscribers.gps, 1.0);
                        if status && ~isempty(msg)
                            stamp = obj.get_msg_stamp(msg);
                            if isnan(last_stamp.gps) || stamp ~= last_stamp.gps
                                data.gps = [data.gps; msg.latitude, msg.longitude, msg.altitude];
                                last_stamp.gps = stamp;
                            end
                        end
                    catch
                    end
                end
                
                % Odometry
                if isfield(obj.subscribers, 'odom') && ~isempty(obj.subscribers.odom)
                    try
                        [msg, status] = receive(obj.subscribers.odom, 1.0);
                        if status && ~isempty(msg)
                            stamp = obj.get_msg_stamp(msg);
                            if isnan(last_stamp.odom) || stamp ~= last_stamp.odom
                                data.odom = [data.odom; msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, ...
                                    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z];
                                last_stamp.odom = stamp;
                            end
                        end
                    catch
                    end
                end
                
                % TF
                if isfield(obj.subscribers, 'tf') && ~isempty(obj.subscribers.tf)
                    try
                        [msg, status] = receive(obj.subscribers.tf, 1.0);
                        if status && ~isempty(msg) && isfield(msg, 'transforms') && ~isempty(msg.transforms)
                            tf = msg.transforms(1);
                            stamp = obj.get_tf_stamp(tf);
                            if isnan(last_stamp.tf) || stamp ~= last_stamp.tf
                                data.tf = [data.tf; tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z, ...
                                    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w];
                                last_stamp.tf = stamp;
                            end
                        end
                    catch
                    end
                end

                % Battery
                if isfield(obj.subscribers, 'battery') && ~isempty(obj.subscribers.battery)
                    try
                        [msg, status] = receive(obj.subscribers.battery, 0.5);
                        if status && ~isempty(msg)
                            stamp = obj.get_msg_stamp(msg);
                            if isnan(last_stamp.battery) || stamp ~= last_stamp.battery
                                data.battery = [data.battery; obj.extract_battery_row(msg)]; %#ok<AGROW>
                                last_stamp.battery = stamp;
                            end
                        end
                    catch
                    end
                end

                % Airspeed
                if isfield(obj.subscribers, 'airspeed') && ~isempty(obj.subscribers.airspeed)
                    try
                        [msg, status] = receive(obj.subscribers.airspeed, 0.5);
                        if status && ~isempty(msg)
                            stamp = obj.get_msg_stamp(msg);
                            got_airspeed = true;
                            if isnan(last_stamp.airspeed) || stamp ~= last_stamp.airspeed
                                data.airspeed = [data.airspeed; obj.extract_airspeed_row(msg)]; %#ok<AGROW>
                                last_stamp.airspeed = stamp;
                            end
                        end
                    catch
                    end
                end

                if ~exist('got_airspeed', 'var') || ~got_airspeed
                    [has_gz_airspeed, gz_row] = obj.read_gz_airspeed_row();
                    if has_gz_airspeed
                        data.airspeed = [data.airspeed; gz_row]; %#ok<AGROW>
                        last_stamp.airspeed = now;
                    end
                end

                % Virtual wind sensor
                if isfield(obj.subscribers, 'wind') && ~isempty(obj.subscribers.wind)
                    try
                        [msg, status] = receive(obj.subscribers.wind, 0.5);
                        if status && ~isempty(msg)
                            stamp = obj.get_msg_stamp(msg);
                            if isnan(last_stamp.wind) || stamp ~= last_stamp.wind
                                data.wind = [data.wind; obj.extract_vector3_row(msg)]; %#ok<AGROW>
                                last_stamp.wind = stamp;
                            end
                        end
                    catch
                    end
                end
                
                if obj.verbose
                    fprintf('[%2d/%2d] IMU: %d, GPS: %d, ODOM: %d, TF: %d, Battery: %d, Airspeed: %d, Wind: %d\n', ...
                        t, duration_sec, size(data.imu,1), size(data.gps,1), size(data.odom,1), ...
                        size(data.tf,1), size(data.battery,1), size(data.airspeed,1), size(data.wind,1));
                end
            end
            
            obj.collected_data = data;
        end
        
        function print_summary(obj, data)
            % 수집된 데이터 요약 출력
            fprintf('\n═════════════════════════════════════\n');
            fprintf('📊 Data Collection Summary\n');
            fprintf('═════════════════════════════════════\n');
            fprintf('  IMU messages:     %d\n', size(data.imu, 1));
            fprintf('  GPS messages:     %d\n', size(data.gps, 1));
            fprintf('  ODOM messages:    %d\n', size(data.odom, 1));
            fprintf('  TF messages:      %d\n', size(data.tf, 1));
            fprintf('  Battery messages: %d\n', size(data.battery, 1));
            fprintf('  Airspeed messages:%d\n', size(data.airspeed, 1));
            if isfield(data, 'wind')
                fprintf('  Wind messages:    %d\n', size(data.wind, 1));
            end
            fprintf('═════════════════════════════════════\n\n');
        end
        
        function obj = unsubscribe_from_topics(obj)
            % 모든 구독 해제
            try
                if isfield(obj.subscribers, 'imu') && ~isempty(obj.subscribers.imu)
                    clear obj.subscribers.imu;
                end
                if isfield(obj.subscribers, 'gps') && ~isempty(obj.subscribers.gps)
                    clear obj.subscribers.gps;
                end
                if isfield(obj.subscribers, 'odom') && ~isempty(obj.subscribers.odom)
                    clear obj.subscribers.odom;
                end
                if isfield(obj.subscribers, 'tf') && ~isempty(obj.subscribers.tf)
                    clear obj.subscribers.tf;
                end
                if isfield(obj.subscribers, 'battery') && ~isempty(obj.subscribers.battery)
                    clear obj.subscribers.battery;
                end
                if isfield(obj.subscribers, 'airspeed') && ~isempty(obj.subscribers.airspeed)
                    clear obj.subscribers.airspeed;
                end
                if isfield(obj.subscribers, 'wind') && ~isempty(obj.subscribers.wind)
                    clear obj.subscribers.wind;
                end
                if ~isempty(obj.gps_publisher)
                    clear obj.gps_publisher;
                end
                if ~isempty(obj.ros_node)
                    clear obj.ros_node;
                end
                if obj.verbose
                    fprintf('[DataCollectionModule] Unsubscribed from all topics\n');
                end
            catch ME
                if obj.verbose
                    fprintf('[DataCollectionModule] Unsubscribe warning: %s\n', ME.message);
                end
            end
        end

        function callback_battery(obj, msg)
            try
                obj.collected_data.battery = [obj.collected_data.battery; obj.extract_battery_row(msg)]; %#ok<AGROW>
            catch
            end
        end

        function callback_airspeed(obj, msg)
            try
                obj.collected_data.airspeed = [obj.collected_data.airspeed; obj.extract_airspeed_row(msg)]; %#ok<AGROW>
            catch
            end
        end

        function callback_wind(obj, msg)
            try
                obj.collected_data.wind = [obj.collected_data.wind; obj.extract_vector3_row(msg)]; %#ok<AGROW>
            catch
            end
        end
    end
    
    methods (Access = private)
        function stamp = get_msg_stamp(obj, msg)
            try
                stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
            catch
                stamp = now;
            end
        end

        function row = extract_battery_row(~, msg)
            row = nan(1, 6);
            try, if isfield(msg, 'percentage'), row(1) = double(msg.percentage); end; catch, end
            try, if isfield(msg, 'voltage'), row(2) = double(msg.voltage); end; catch, end
            try, if isfield(msg, 'current'), row(3) = double(msg.current); end; catch, end
            try, if isfield(msg, 'charge'), row(4) = double(msg.charge); end; catch, end
            try, if isfield(msg, 'capacity'), row(5) = double(msg.capacity); end; catch, end
            try, if isfield(msg, 'design_capacity'), row(6) = double(msg.design_capacity); end; catch, end
        end

        function row = extract_airspeed_row(~, msg)
            row = [NaN, NaN, NaN];
            try
                if isfield(msg, 'true_airspeed')
                    row = [double(msg.true_airspeed.x), double(msg.true_airspeed.y), double(msg.true_airspeed.z)];
                elseif isfield(msg, 'vector')
                    row = [double(msg.vector.x), double(msg.vector.y), double(msg.vector.z)];
                end
            catch
            end
        end

        function row = extract_vector3_row(~, msg)
            row = [NaN, NaN, NaN];
            try
                if isfield(msg, 'vector') || isprop(msg, 'vector')
                    row = [double(msg.vector.x), double(msg.vector.y), double(msg.vector.z)];
                end
            catch
            end
        end

        function stamp = get_tf_stamp(obj, tf)
            try
                stamp = double(tf.header.stamp.sec) + double(tf.header.stamp.nanosec) * 1e-9;
            catch
                stamp = now;
            end
        end

        function [ok, row] = read_gz_airspeed_row(obj)
            ok = false;
            row = [NaN, NaN, NaN];
            try
                topic = sprintf('/world/%s/model/%s/link/base_link/sensor/air_speed_sensor/air_speed', obj.world_name, obj.robot_name);
                cmd = sprintf('timeout 0.5 gz topic -e -t "%s" -n 1 2>/dev/null', topic);
                [status, output] = system(cmd);
                if status ~= 0 || isempty(strtrim(output))
                    return;
                end

                token = regexp(output, 'diff_pressure:\s*([-+0-9.eE]+)', 'tokens', 'once');
                if isempty(token)
                    return;
                end

                diff_pressure = str2double(token{1});
                if ~isfinite(diff_pressure)
                    return;
                end

                rho = 1.225;
                tas = sqrt(max(0, 2 * abs(diff_pressure) / rho));
                row = [tas, 0, 0];
                ok = true;
            catch
            end
        end
    end
end
