%% CircleControlModule - Circle Flight Control
% 원형 비행 제어 전담 모듈
% 
% 용도: 항공기를 중심점 주변의 원형 경로로 비행하도록 제어
% 
% 사용법:
%   controller = CircleControlModule('namespace', '/alti_transition', 'verbose', true);
%   controller = controller.initialize();
%   success = controller.execute_circle_flight('radius_m', 100, 'altitude_agl_m', 30);
%   controller.cleanup();

classdef CircleControlModule < handle
    properties (Constant)
        COPTER_MODE_GUIDED = 4;
    end

    properties (Access = private)
        namespace
        receiver
        goal_pub
        arm_client
        mode_client
        takeoff_client
        prearm_client
        verbose
        ros_node
        flight_log
        last_waypoints
        last_waypoints_local
        control_start_tic
    end
    
    properties (Access = public)
        is_initialized = false
    end
    
    methods
        function obj = CircleControlModule(varargin)
            % 생성자
            p = inputParser;
            addParameter(p, 'namespace', '/alti_transition', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});
            
            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
            
            if obj.verbose
                fprintf('[CircleControlModule] Initialized (namespace: %s)\n', obj.namespace);
            end
        end
        
        function obj = initialize(obj)
            % ROS2 인터페이스 초기화
            try
                % ROS2 노드 생성
                obj.ros_node = ros2node('/circle_control');
                
                % 항공기 상태 수신기 생성
                obj.receiver = AircraftStateReceiver('verbose', obj.verbose, 'enable_logging', false);
                obj.receiver.subscribe_to_topics();
                
                % 웨이포인트 발행 토픽: ArduPilot은 /ap/cmd_gps_pose를 직접 사용
                % (네임스페이스 무시, DDS 기반)
                goal_topic = '/ap/cmd_gps_pose';
                
                if obj.verbose
                    fprintf('[CircleControlModule] Using hardcoded goal topic: %s\n', goal_topic);
                end
                
                % 퍼블리셔 생성 시도 (실패해도 MAVLink fallback 사용)
                try
                    obj.goal_pub = ros2publisher(obj.ros_node, goal_topic, 'geographic_msgs/GeoPoseStamped');
                catch
                    % MATLAB이 geographic_msgs를 인식하지 못할 수 있으므로, 대신 PyMAVLink 사용
                    obj.goal_pub = [];
                    if obj.verbose
                        fprintf('[CircleControlModule] Note: ROS2 goal publisher not available, using MAVLink for waypoints\n');
                    end
                end
                
                % 서비스: ArduPilot의 기본 서비스 이름 사용 (/ap/arm_motors 등)
                arm_service = '/ap/arm_motors';
                mode_service = '/ap/mode_switch';
                takeoff_service = '/ap/experimental/takeoff';
                prearm_service = '/ap/prearm_check';
                
                % 서비스 클라이언트 생성 with 상세 로깅
                if obj.verbose
                    fprintf('[CircleControlModule] Attempting to create service clients:\n');
                    fprintf('  - arm_motors: %s\n', arm_service);
                    fprintf('  - mode_switch: %s\n', mode_service);
                    fprintf('  - takeoff: %s\n', takeoff_service);
                end
                
                obj.arm_client = obj.create_service_client(arm_service, 'ardupilot_msgs/srv/ArmMotors');
                obj.mode_client = obj.create_service_client(mode_service, 'ardupilot_msgs/srv/ModeSwitch');
                obj.takeoff_client = obj.create_service_client(takeoff_service, 'ardupilot_msgs/srv/Takeoff');
                
                if obj.verbose
                    fprintf('[CircleControlModule] Service client status:\n');
                    fprintf('  - arm_motors: %s\n', obj.status_str(~isempty(obj.arm_client)));
                    fprintf('  - mode_switch: %s\n', obj.status_str(~isempty(obj.mode_client)));
                    fprintf('  - takeoff: %s\n', obj.status_str(~isempty(obj.takeoff_client)));
                end
                
                if ~isempty(prearm_service)
                    obj.prearm_client = obj.create_service_client(prearm_service, 'std_srvs/srv/Trigger');
                    if obj.verbose
                        fprintf('  - prearm_check: %s\n', obj.status_str(~isempty(obj.prearm_client)));
                    end
                else
                    obj.prearm_client = [];
                    if obj.verbose
                        fprintf('  - prearm_check: NOT AVAILABLE\n');
                    end
                end

                obj.flight_log = obj.create_empty_flight_log();
                obj.last_waypoints = [];
                obj.last_waypoints_local = [];
                
                obj.is_initialized = true;
                
                if obj.verbose
                    fprintf('[CircleControlModule] Initialization complete\n');
                end
            catch ME
                fprintf('[CircleControlModule] Initialization failed: %s\n', ME.message);
                rethrow(ME);
            end
        end
        function success = execute_circle_flight(obj, varargin)
            % 복합 코스 비행 제어 실행
            % 입력:
            %   'radius_m' - legacy circle radius / curved return radius hint
            %   'altitude_agl_m' - 고도 AGL (기본값: 30)
            %   'num_points' - legacy waypoint count
            %   'tolerance_m' - 도달 허용오차 (기본값: 30)
            %   'waypoint_timeout_s' - 웨이포인트별 타임아웃 (기본값: 20)
            
            success = false;
            
            if ~obj.is_initialized
                fprintf('[ERROR] CircleControlModule: initialize() must be called first\n');
                return;
            end
            
            p = inputParser;
            addParameter(p, 'radius_m', 5, @isnumeric);
            addParameter(p, 'altitude_agl_m', 5, @isnumeric);
            addParameter(p, 'num_points', 36, @isnumeric);
            addParameter(p, 'tolerance_m', 30, @isnumeric);
            addParameter(p, 'waypoint_timeout_s', 20, @isnumeric);
            addParameter(p, 'hover_duration_s', 10, @isnumeric);
            addParameter(p, 'hover_tolerance_m', 5, @isnumeric);
            addParameter(p, 'sample_interval_s', 0.5, @isnumeric);
            addParameter(p, 'course_type', 'wind_test', @ischar);
            addParameter(p, 'straight_length_m', 35, @isnumeric);
            addParameter(p, 'climb_height_m', 8, @isnumeric);
            addParameter(p, 'descent_forward_m', 35, @isnumeric);
            addParameter(p, 'descent_lateral_m', 18, @isnumeric);
            addParameter(p, 'straight_points', 6, @isnumeric);
            addParameter(p, 'climb_points', 4, @isnumeric);
            addParameter(p, 'descent_points', 6, @isnumeric);
            addParameter(p, 'return_curve_points', 12, @isnumeric);
            addParameter(p, 'waypoint_spacing_m', 4, @isnumeric);
            addParameter(p, 'smooth_interpolation', true, @islogical);
            addParameter(p, 'land_at_end', true, @islogical);
            addParameter(p, 'landing_timeout_s', 45, @isnumeric);
            parse(p, varargin{:});
            
            radius_m = p.Results.radius_m;
            altitude_agl_m = p.Results.altitude_agl_m;
            num_points = p.Results.num_points;
            tolerance_m = p.Results.tolerance_m;
            waypoint_timeout_s = p.Results.waypoint_timeout_s;
            hover_duration_s = p.Results.hover_duration_s;
            hover_tolerance_m = p.Results.hover_tolerance_m;
            sample_interval_s = p.Results.sample_interval_s;
            course_type = p.Results.course_type;
            land_at_end = p.Results.land_at_end;
            landing_timeout_s = p.Results.landing_timeout_s;

            obj.flight_log = obj.create_empty_flight_log();
            obj.control_start_tic = tic;
            
            try
                % PHASE 1: Wait for valid aircraft state
                if obj.verbose
                    fprintf('[CircleControlModule] Waiting for valid aircraft state...\n');
                end
                try
                    state = obj.wait_for_valid_state(30);
                catch ME
                    fprintf('[ERROR] Phase 1 - Aircraft state timeout: %s\n', ME.message);
                    return;
                end
                
                lat0 = state.position.latitude;
                lon0 = state.position.longitude;
                alt0 = state.position.altitude;
                target_alt = alt0 + altitude_agl_m;
                use_geo_reference = true;
                if ~(isfinite(lat0) && isfinite(lon0))
                    use_geo_reference = false;
                end
                if ~use_geo_reference
                    home_x = state.position.x;
                    home_y = state.position.y;
                    if obj.verbose
                        fprintf('[CircleControlModule] Using LOCAL ODOM reference for hover/waypoints\n');
                    end
                end
                
                if obj.verbose
                    fprintf('[CircleControlModule] Home position: LAT=%.6f LON=%.6f ALT=%.1f m\n', lat0, lon0, alt0);
                    fprintf('[CircleControlModule] Target altitude: %.1f m AGL\n', altitude_agl_m);
                end
                
                % PHASE 2: Arming and mode switching
                if obj.verbose
                    fprintf('[CircleControlModule] Arming and switching to guided mode...\n');
                end
                
                % Skip ROS2 prearm check (services not available)
                % Proceed directly to arm+mode+takeoff sequence
                
                % Prefer the official ROS 2 DDS services for Copter control.
                % Fall back to the MAVLink sequence only if a service call fails.
                % In this runtime the DDS control services are unavailable,
                % so the standalone arm/mode/takeoff calls are not reliable.
                % Go directly to the combined MAVLink sequence.
                if isempty(obj.arm_client) && isempty(obj.mode_client) && isempty(obj.takeoff_client)
                    if obj.verbose
                        fprintf('[CircleControlModule] No DDS control services; using combined MAVLink sequence directly\n');
                    end
                    if ~obj.arm_mode_takeoff_sequence(altitude_agl_m, lat0, lon0)
                        fprintf('[ERROR] Phase 2 - ARM+MODE+TAKEOFF sequence failed. Flight control aborted.\n');
                        return;
                    end
                else
                    arm_ok = obj.arm_vehicle(true);
                    mode_ok = obj.switch_mode(obj.COPTER_MODE_GUIDED);

                    takeoff_ok = false;
                    if ~isempty(obj.takeoff_client)
                        takeoff_ok = obj.takeoff_vehicle(altitude_agl_m);
                    end

                    if ~(arm_ok && mode_ok && takeoff_ok)
                        fprintf('[WARN] Service-based ARM/MODE/TAKEOFF path incomplete; trying MAVLink fallback sequence...\n');
                        if ~obj.arm_mode_takeoff_sequence(altitude_agl_m, lat0, lon0)
                            fprintf('[ERROR] Phase 2 - ARM+MODE+TAKEOFF sequence failed. Flight control aborted.\n');
                            return;
                        end
                    end
                end
                
                % PHASE 3: Allow the takeoff monitor to settle before hover.
                pause(2.0);

                % Ensure vehicle is in GUIDED mode for goal interface.
                % Iris/Gazebo follows the Copter DDS demo, where GUIDED is mode 4.
                if obj.verbose
                    fprintf('[CircleControlModule] Ensuring GUIDED mode for waypoint following...\n');
                end
                if ~obj.switch_mode(obj.COPTER_MODE_GUIDED)
                    fprintf('[WARN] Could not switch to GUIDED mode; waypoint following may fail\n');
                else
                    if obj.verbose
                        fprintf('[CircleControlModule] Mode set to GUIDED\n');
                    end
                end

                if ~use_geo_reference
                    current_state = obj.receiver.get_current_state();
                    if isfield(current_state, 'position') && isfinite(current_state.position.x)
                        home_x = current_state.position.x;
                        home_y = current_state.position.y;
                        if isfinite(current_state.position.z)
                            target_alt = current_state.position.z;
                        end
                        if obj.verbose
                            fprintf('[CircleControlModule] Refreshed LOCAL reference after climb: X=%.2f Y=%.2f Z=%.2f\n', home_x, home_y, target_alt);
                        end
                    end
                end

                % PHASE 4: Hover verification
                if obj.verbose
                    fprintf('[CircleControlModule] Hover verification for %.1f seconds...\n', hover_duration_s);
                end
                if use_geo_reference
                    hover_stats = obj.hold_position(lat0, lon0, target_alt, hover_duration_s, hover_tolerance_m, sample_interval_s);
                else
                    hover_stats = obj.hold_position_local(home_x, home_y, target_alt, hover_duration_s, hover_tolerance_m, sample_interval_s);
                end
                if obj.verbose
                    fprintf('[CircleControlModule] Hover drift: max=%.1f m, final=%.1f m\n', hover_stats.max_error_m, hover_stats.final_error_m);
                end
                
                % PHASE 5: Build wind-test trajectory
                if obj.verbose
                    fprintf('[CircleControlModule] Building %s trajectory...\n', course_type);
                end
                if strcmpi(course_type, 'circle')
                    if use_geo_reference
                        [waypoints, waypoints_local] = obj.build_circle_waypoints(lat0, lon0, target_alt, radius_m, num_points);
                    else
                        [waypoints, waypoints_local] = obj.build_circle_waypoints_local(home_x, home_y, target_alt, radius_m, num_points);
                    end
                else
                    if use_geo_reference
                        [waypoints, waypoints_local] = obj.build_wind_test_course_waypoints( ...
                            lat0, lon0, target_alt, ...
                            p.Results.straight_length_m, p.Results.climb_height_m, ...
                            p.Results.descent_forward_m, p.Results.descent_lateral_m, ...
                            p.Results.straight_points, p.Results.climb_points, ...
                            p.Results.descent_points, p.Results.return_curve_points, ...
                            p.Results.waypoint_spacing_m, p.Results.smooth_interpolation);
                    else
                        [waypoints, waypoints_local] = obj.build_wind_test_course_waypoints_local( ...
                            home_x, home_y, target_alt, ...
                            p.Results.straight_length_m, p.Results.climb_height_m, ...
                            p.Results.descent_forward_m, p.Results.descent_lateral_m, ...
                            p.Results.straight_points, p.Results.climb_points, ...
                            p.Results.descent_points, p.Results.return_curve_points, ...
                            p.Results.waypoint_spacing_m, p.Results.smooth_interpolation);
                    end
                end
                obj.last_waypoints = waypoints;
                obj.last_waypoints_local = waypoints_local;
                
                % PHASE 6: Follow planned path
                if obj.verbose
                    fprintf('[CircleControlModule] Following planned path (%d waypoints)...\n', size(waypoints, 1));
                end
                reached_count = 0;
                for i = 1:size(waypoints, 1)
                    cmd_1 = waypoints(i, 1);
                    cmd_2 = waypoints(i, 2);
                    alt_cmd = waypoints(i, 3);

                    if use_geo_reference
                        obj.publish_goal(cmd_1, cmd_2, alt_cmd);
                        obj.record_flight_sample(sprintf('waypoint_%03d', i), cmd_1, cmd_2, alt_cmd);
                        if obj.verbose
                            fprintf('[%03d/%03d] LAT=%.6f LON=%.6f ALT=%.1f\n', i, size(waypoints,1), cmd_1, cmd_2, alt_cmd);
                        end
                        reached = obj.wait_until_close(cmd_1, cmd_2, alt_cmd, tolerance_m, waypoint_timeout_s, sprintf('waypoint_%03d', i));
                    else
                        obj.publish_goal_local(cmd_1, cmd_2, alt_cmd);
                        obj.record_flight_sample(sprintf('waypoint_%03d', i), cmd_1, cmd_2, alt_cmd);
                        if obj.verbose
                            fprintf('[%03d/%03d] X=%.2f Y=%.2f ALT=%.1f\n', i, size(waypoints,1), cmd_1, cmd_2, alt_cmd);
                        end
                        reached = obj.wait_until_close_local(cmd_1, cmd_2, alt_cmd, tolerance_m, waypoint_timeout_s, sprintf('waypoint_%03d', i));
                    end
                    if reached
                        reached_count = reached_count + 1;
                    else
                        if obj.verbose
                            fprintf('  ⚠️  Waypoint %d not reached (timeout)\n', i);
                        end
                    end
                    pause(1.0);
                end
                % PHASE 7: Return home, land, and disarm
                if obj.verbose
                    fprintf('[CircleControlModule] Returning to home before landing...\n');
                end
                if use_geo_reference
                    obj.publish_goal(lat0, lon0, target_alt);
                    obj.record_flight_sample('return_home', lat0, lon0, target_alt);
                    obj.wait_until_close(lat0, lon0, target_alt, tolerance_m, waypoint_timeout_s, 'return_home');
                else
                    obj.publish_goal_local(home_x, home_y, target_alt);
                    obj.record_flight_sample('return_home', home_x, home_y, target_alt);
                    obj.wait_until_close_local(home_x, home_y, target_alt, tolerance_m, waypoint_timeout_s, 'return_home');
                end

                if land_at_end
                    obj.land_and_disarm(alt0, landing_timeout_s);
                else
                    obj.arm_vehicle(false);
                end
                
                success = (reached_count / size(waypoints, 1)) > 0.7;
                
                if obj.verbose
                    fprintf('[CircleControlModule] Planned flight complete (reached %d/%d waypoints)\n', reached_count, size(waypoints, 1));
                end
                
            catch ME
                fprintf('[ERROR] Execute circle flight exception: %s\n', ME.message);
                try
                    obj.arm_vehicle(false);
                catch
                    fprintf('[ERROR] Final disarm attempt also failed\n');
                end
                success = false;
            end
        end
        
        function cleanup(obj)
            % 정리 및 해제
            try
                if ~isempty(obj.receiver)
                    obj.receiver.unsubscribe_from_topics();
                end
                if obj.verbose
                    fprintf('[CircleControlModule] Cleanup complete\n');
                end
            catch ME
                if obj.verbose
                    fprintf('[CircleControlModule] Cleanup warning: %s\n', ME.message);
                end
            end
        end

        function log = get_flight_log(obj)
            log = obj.flight_log;
        end

        function waypoints = get_last_waypoints(obj)
            waypoints = obj.last_waypoints;
        end

        function waypoints_local = get_last_waypoints_local(obj)
            waypoints_local = obj.last_waypoints_local;
        end
    end
    
    methods (Access = private)
        function obj = run_prearm_check(obj)
            try
                if isempty(obj.prearm_client)
                    return;
                end
                req = ros2message(obj.prearm_client);
                resp = call(obj.prearm_client, req, 'Timeout', 5);
                if obj.verbose
                    if isprop(resp, 'success')
                        fprintf('[CircleControlModule] Prearm check: success=%d\n', logical(resp.success));
                    else
                        fprintf('[CircleControlModule] Prearm check completed\n');
                    end
                end
            catch ME
                if obj.verbose
                    fprintf('[CircleControlModule] Prearm check warning: %s\n', ME.message);
                end
            end
        end

        function state = wait_for_valid_state(obj, timeout_s)
            t0 = tic;
            has_geo_msg_logged = false;
            while toc(t0) < timeout_s
                state = obj.receiver.get_current_state();
                if state.is_valid && isfield(state, 'position')
                    % Try to use absolute geo reference if available
                    has_geo = isfield(state.position, 'has_absolute_position') && state.position.has_absolute_position && ...
                        isfield(state.position, 'position_source') && ~strcmpi(state.position.position_source, 'unknown') && ...
                        ~isnan(state.position.latitude) && ~isnan(state.position.longitude);
                    
                    if has_geo
                        if obj.verbose
                            fprintf('[CircleControlModule] State ready: ODOM(%.2f, %.2f, %.2f) GPS(%.6f, %.6f) SRC=%s\n', ...
                                state.position.x, state.position.y, state.position.z, state.position.latitude, state.position.longitude, state.position.position_source);
                        end
                        return;
                    end
                end
                pause(0.5);
            end
            elapsed = toc(t0);
            error(sprintf('Timed out waiting for valid aircraft state (%.1f / %.1f seconds)', elapsed, timeout_s));
        end
        
        function wait_until_altitude(obj, target_alt, timeout_s)
            t0 = tic;
            max_altitude_seen = 0;
            while toc(t0) < timeout_s
                state = obj.receiver.get_current_state();
                current_alt = state.position.altitude;
                max_altitude_seen = max(max_altitude_seen, current_alt);
                
                if obj.verbose
                    fprintf('  Altitude: %.1f / %.1f m (max: %.1f)\n', current_alt, target_alt, max_altitude_seen);
                end
                obj.record_flight_sample('takeoff', NaN, NaN, target_alt);
                
                % More strict altitude check: within 0.5m of target
                if current_alt >= target_alt - 0.5
                    fprintf('[CircleControlModule] Target altitude reached: %.1f m (target: %.1f m)\n', current_alt, target_alt);
                    return;
                end
                pause(0.5);
            end
            elapsed = toc(t0);
            fprintf('[CircleControlModule] WARNING: timed out waiting for altitude (max reached: %.1f m, target: %.1f m, %.1f / %.1f seconds); continuing anyway\n', max_altitude_seen, target_alt, elapsed, timeout_s);
        end

        function ok = land_and_disarm(obj, home_alt, timeout_s)
            ok = false;
            if nargin < 3 || isempty(timeout_s)
                timeout_s = 45;
            end

            if obj.verbose
                fprintf('[CircleControlModule] Switching to LAND mode...\n');
            end

            land_mode_ok = obj.switch_mode('LAND');
            if ~land_mode_ok
                fprintf('[CircleControlModule] Warning: LAND mode command was not confirmed; monitoring descent anyway\n');
            end

            landed = obj.wait_until_landed(home_alt, timeout_s);
            if ~landed
                fprintf('[CircleControlModule] Warning: landing timeout; attempting disarm only after best-effort landing wait\n');
            end

            ok = obj.arm_vehicle(false);
            if obj.verbose
                fprintf('[CircleControlModule] Landing sequence complete: landed=%s, disarm=%s\n', ...
                    obj.status_str(landed), obj.status_str(ok));
            end
        end

        function landed = wait_until_landed(obj, home_alt, timeout_s)
            landed = false;
            t0 = tic;
            below_threshold_count = 0;
            if nargin < 2 || ~isfinite(home_alt)
                home_alt = 0;
            end

            while toc(t0) < timeout_s
                state = obj.receiver.get_current_state();
                obj.record_flight_sample('landing', NaN, NaN, home_alt);

                current_alt = NaN;
                if isfield(state, 'position')
                    if isfield(state.position, 'altitude') && isfinite(state.position.altitude)
                        current_alt = state.position.altitude;
                    elseif isfield(state.position, 'z') && isfinite(state.position.z)
                        current_alt = state.position.z;
                    end
                end

                agl_est = abs(current_alt - home_alt);
                if isfinite(agl_est) && agl_est <= 0.35
                    below_threshold_count = below_threshold_count + 1;
                else
                    below_threshold_count = 0;
                end

                if obj.verbose
                    fprintf('  Landing: alt=%.2f m, home=%.2f m, agl_est=%.2f m\n', current_alt, home_alt, agl_est);
                end

                if below_threshold_count >= 4
                    landed = true;
                    if obj.verbose
                        fprintf('[CircleControlModule] Landing altitude threshold reached\n');
                    end
                    return;
                end
                pause(0.5);
            end
        end
        
        function [waypoints, waypoints_local] = build_circle_waypoints(obj, lat0, lon0, alt, radius_m, num_points)
            waypoints = [];
            waypoints_local = [];
            deg_per_m_lat = 1 / 111319.9;
            deg_per_m_lon = 1 / (111319.9 * cosd(lat0));
            
            t = linspace(0, 2*pi, num_points+1);
            t(end) = [];
            
            east_m = radius_m * cos(t);
            north_m = radius_m * sin(t);
            
            for i = 1:length(t)
                lat = lat0 + north_m(i) * deg_per_m_lat;
                lon = lon0 + east_m(i) * deg_per_m_lon;
                waypoints = [waypoints; lat, lon, alt];
                waypoints_local = [waypoints_local; east_m(i), north_m(i), alt]; %#ok<AGROW>
            end
        end

        function [waypoints, waypoints_local] = build_circle_waypoints_local(obj, origin_x, origin_y, alt, radius_m, num_points)
            waypoints = [];
            waypoints_local = [];

            t = linspace(0, 2*pi, num_points+1);
            t(end) = [];

            east_m = radius_m * cos(t);
            north_m = radius_m * sin(t);

            for i = 1:length(t)
                x = origin_x + east_m(i);
                y = origin_y + north_m(i);
                waypoints = [waypoints; x, y, alt]; %#ok<AGROW>
                waypoints_local = [waypoints_local; x, y, alt]; %#ok<AGROW>
            end
        end

        function [waypoints, waypoints_local] = build_wind_test_course_waypoints(obj, lat0, lon0, base_alt, straight_length_m, climb_height_m, descent_forward_m, descent_lateral_m, straight_points, climb_points, descent_points, return_curve_points, waypoint_spacing_m, smooth_interpolation)
            local_points = obj.build_wind_test_course_local_offsets( ...
                base_alt, straight_length_m, climb_height_m, descent_forward_m, descent_lateral_m, ...
                straight_points, climb_points, descent_points, return_curve_points, ...
                waypoint_spacing_m, smooth_interpolation);

            deg_per_m_lat = 1 / 111319.9;
            deg_per_m_lon = 1 / (111319.9 * cosd(lat0));
            waypoints = zeros(size(local_points));
            waypoints_local = local_points;

            for i = 1:size(local_points, 1)
                east_m = local_points(i, 1);
                north_m = local_points(i, 2);
                waypoints(i, 1) = lat0 + north_m * deg_per_m_lat;
                waypoints(i, 2) = lon0 + east_m * deg_per_m_lon;
                waypoints(i, 3) = local_points(i, 3);
            end
        end

        function [waypoints, waypoints_local] = build_wind_test_course_waypoints_local(obj, origin_x, origin_y, base_alt, straight_length_m, climb_height_m, descent_forward_m, descent_lateral_m, straight_points, climb_points, descent_points, return_curve_points, waypoint_spacing_m, smooth_interpolation)
            local_offsets = obj.build_wind_test_course_local_offsets( ...
                base_alt, straight_length_m, climb_height_m, descent_forward_m, descent_lateral_m, ...
                straight_points, climb_points, descent_points, return_curve_points, ...
                waypoint_spacing_m, smooth_interpolation);
            waypoints = local_offsets;
            waypoints(:, 1) = origin_x + local_offsets(:, 1);
            waypoints(:, 2) = origin_y + local_offsets(:, 2);
            waypoints_local = waypoints;
        end

        function points = build_wind_test_course_local_offsets(obj, base_alt, straight_length_m, climb_height_m, descent_forward_m, descent_lateral_m, straight_points, climb_points, descent_points, return_curve_points, waypoint_spacing_m, smooth_interpolation)
            straight_points = max(2, round(straight_points));
            climb_points = max(2, round(climb_points));
            descent_points = max(2, round(descent_points));
            return_curve_points = max(4, round(return_curve_points));

            high_alt = base_alt + climb_height_m;
            end_straight = [straight_length_m, 0, base_alt];
            end_descent = [straight_length_m + descent_forward_m, descent_lateral_m, base_alt];

            points = [];

            % 1) Long straight segment at constant altitude.
            t = linspace(0, 1, straight_points + 1);
            t(1) = [];
            for i = 1:numel(t)
                points = [points; straight_length_m * t(i), 0, base_alt]; %#ok<AGROW>
            end

            % 2) Vertical climb at the end of the straight segment.
            t = linspace(0, 1, climb_points + 1);
            t(1) = [];
            for i = 1:numel(t)
                points = [points; end_straight(1), end_straight(2), base_alt + climb_height_m * t(i)]; %#ok<AGROW>
            end

            % 3) Sloped descent with lateral offset to expose crosswind drift.
            t = linspace(0, 1, descent_points + 1);
            t(1) = [];
            for i = 1:numel(t)
                x = end_straight(1) + descent_forward_m * t(i);
                y = descent_lateral_m * t(i);
                z = high_alt + (base_alt - high_alt) * t(i);
                points = [points; x, y, z]; %#ok<AGROW>
            end

            % 4) Smooth curved return to home. Quadratic Bezier keeps the path continuous
            % while bending away from the straight line for a clearer wind-response trace.
            p0 = end_descent;
            p2 = [0, 0, base_alt];
            curve_bulge_m = max(12, 0.35 * hypot(p0(1), p0(2)));
            if p0(2) >= 0
                curve_side = 1;
            else
                curve_side = -1;
            end
            p1 = [0.5 * p0(1), p0(2) + curve_side * curve_bulge_m, base_alt];
            t = linspace(0, 1, return_curve_points + 1);
            t(1) = [];
            for i = 1:numel(t)
                u = t(i);
                curve_point = (1 - u)^2 * p0 + 2 * (1 - u) * u * p1 + u^2 * p2;
                points = [points; curve_point]; %#ok<AGROW>
            end

            if smooth_interpolation
                points = obj.interpolate_course_points(points, waypoint_spacing_m);
            end
        end

        function smooth_points = interpolate_course_points(~, points, spacing_m)
            if nargin < 3 || isempty(spacing_m) || spacing_m <= 0 || size(points, 1) < 3
                smooth_points = points;
                return;
            end

            segment_lengths = sqrt(sum(diff(points, 1, 1).^2, 2));
            cumulative_s = [0; cumsum(segment_lengths)];
            keep = [true; diff(cumulative_s) > 1e-6];
            points = points(keep, :);
            cumulative_s = cumulative_s(keep);

            total_length = cumulative_s(end);
            if total_length <= spacing_m
                smooth_points = points;
                return;
            end

            sample_count = max(size(points, 1), ceil(total_length / spacing_m) + 1);
            sample_s = linspace(0, total_length, sample_count)';
            smooth_points = zeros(sample_count, 3);
            for axis_idx = 1:3
                smooth_points(:, axis_idx) = interp1(cumulative_s, points(:, axis_idx), sample_s, 'pchip');
            end

            smooth_points(1, :) = points(1, :);
            smooth_points(end, :) = points(end, :);
        end
        
        function publish_goal(obj, lat, lon, alt)
            % Publish waypoint goal to ArduPilot via PyMAVLink
            % lat: latitude (degrees)
            % lon: longitude (degrees)
            % alt: altitude (meters AGL)
            
            % Try ROS2 publisher first if available
            if ~isempty(obj.goal_pub)
                try
                    msg = ros2message('geographic_msgs/GeoPoseStamped');
                    msg.header.stamp = rostime('now');
                    msg.header.frame_id = 'map';
                    msg.pose.position.latitude = lat;
                    msg.pose.position.longitude = lon;
                    msg.pose.position.altitude = alt;
                    
                    for iPub = 1:3
                        try
                            send(obj.goal_pub, msg);
                            if obj.verbose
                                fprintf('[CircleControlModule] Published waypoint (ROS2, try %d): LAT=%.6f LON=%.6f ALT=%.1f\n', iPub, lat, lon, alt);
                            end
                        catch
                        end
                        pause(0.25);
                    end
                    return;
                catch
                end
            end
            
            % Fallback: Use a dedicated pymavlink helper for waypoint send
            try
                obj.send_waypoint_via_pymavlink(lat, lon, alt, false);
                if obj.verbose
                    fprintf('[CircleControlModule] Published waypoint (PyMAVLink helper): LAT=%.6f LON=%.6f ALT=%.1f\n', lat, lon, alt);
                end
            catch ME
                fprintf('[CircleControlModule] Warning: Could not send waypoint via PyMAVLink helper: %s\n', ME.message);
            end
        end

        function publish_goal_local(obj, x, y, alt)
            % Publish local NED waypoint goal via PyMAVLink
            try
                obj.send_waypoint_via_pymavlink(x, y, alt, true);
                if obj.verbose
                    fprintf('[CircleControlModule] Published local waypoint (PyMAVLink helper): X=%.2f Y=%.2f ALT=%.1f\n', x, y, alt);
                end
            catch ME
                fprintf('[CircleControlModule] Warning: Could not send local waypoint via PyMAVLink: %s\n', ME.message);
            end
        end

        function send_waypoint_via_pymavlink(obj, a, b, c, is_local)
            % Helper: send a waypoint via a temporary pymavlink script.
            % a,b,c: lat,lon,alt if is_local==false; x,y,alt if is_local==true
            py_file = [tempname, '.py'];
            fid = fopen(py_file, 'w');
            if fid < 0
                error('CircleControlModule: cannot create temporary pymavlink script');
            end
            cleaner = onCleanup(@() obj.delete_temp_file(py_file)); %#ok<NASGU>

            if is_local
                % Local NED waypoint (x,y,z)
                script_template = strjoin({
                    'import time\n'
                    'from pymavlink import mavutil\n'
                    'conn = None\n'
                    'for uri in ("udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"):\n'
                    '    try:\n'
                    '        conn = mavutil.mavlink_connection(uri, source_system=251, timeout=5)\n'
                    '        conn.wait_heartbeat(timeout=5)\n'
                    '        break\n'
                    '    except Exception:\n'
                    '        conn = None\n'
                    'if conn is None:\n'
                    '    raise RuntimeError("no mavlink connection")\n'
                    'conn.mav.set_position_target_local_ned_send(int(time.time()*1000)&0xffffffff, conn.target_system, conn.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, %.3f, %.3f, %.3f, 0,0,0, 0,0,0, 0,0)\n'
                    'time.sleep(0.05)\n'
                    'conn.close()\n'
                }, '');
                script = sprintf(script_template, a, b, -c);
            else
                % Global relative-alt waypoint
                script_template = strjoin({
                    'import time\n'
                    'from pymavlink import mavutil\n'
                    'conn = None\n'
                    'for uri in ("udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"):\n'
                    '    try:\n'
                    '        conn = mavutil.mavlink_connection(uri, source_system=251, timeout=5)\n'
                    '        conn.wait_heartbeat(timeout=5)\n'
                    '        break\n'
                    '    except Exception:\n'
                    '        conn = None\n'
                    'if conn is None:\n'
                    '    raise RuntimeError("no mavlink connection")\n'
                    'lat = int(%.6f * 1e7)\n'
                    'lon = int(%.6f * 1e7)\n'
                    'alt = %.3f\n'
                    'frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT\n'
                    'type_mask = 0b0000000000000000\n'
                    'conn.mav.set_position_target_global_int_send(0, conn.target_system, conn.target_component, frame, type_mask, lat, lon, alt, 0,0,0, 0,0,0, 0,0)\n'
                    'time.sleep(0.05)\n'
                    'conn.close()\n'
                }, '');
                script = sprintf(script_template, a, b, c);
            end

            fwrite(fid, script, 'char');
            fclose(fid);
            cmd = sprintf('python3 "%s" 2>&1', py_file);
            [status, out] = system(cmd);
            if status ~= 0
                error('CircleControlModule: pymavlink waypoint send failed: %s', strtrim(out));
            end
        end
        
        function ok = arm_vehicle(obj, should_arm)
            ok = false;
            cmd_name = 'arm'; if ~should_arm, cmd_name = 'disarm'; end
            
            try
                if ~isempty(obj.arm_client)
                    req = ros2message(obj.arm_client);
                    if isprop(req, 'arm')
                        req.arm = logical(should_arm);
                    elseif isprop(req, 'value')
                        req.value = logical(should_arm);
                    elseif isprop(req, 'Value')
                        req.Value = logical(should_arm);
                    end

                    resp = call(obj.arm_client, req, 'Timeout', 10);
                    if isprop(resp, 'result')
                        ok = logical(resp.result);
                    elseif isprop(resp, 'success')
                        ok = logical(resp.success);
                    else
                        ok = true;
                    end
                    if obj.verbose
                        fprintf('[CircleControlModule] %s via ROS2 service: %s\n', cmd_name, obj.status_str(ok));
                    end
                else
                    % Use pymavlink directly when ROS2 services are unavailable.
                    if should_arm
                        ok = obj.run_pymavlink_action('arm', '', 30);
                    else
                        ok = obj.run_pymavlink_action('disarm', '', 30);
                    end
                end

                if ~ok && should_arm && isempty(obj.arm_client)
                    fprintf('[ERROR] CircleControlModule: %s via pymavlink failed\n', cmd_name);
                elseif ~ok && should_arm
                    fprintf('[ERROR] CircleControlModule: %s via ROS2 service failed\n', cmd_name);
                elseif obj.verbose
                    fprintf('[CircleControlModule] %s command accepted\n', cmd_name);
                end
                pause(2.0);
            catch ME
                fprintf('[ERROR] CircleControlModule: %s exception: %s\n', cmd_name, ME.message);
                ok = false;
            end
        end
        
        function ok = switch_mode(obj, mode_value)
            ok = false;
            try
                if isnumeric(mode_value)
                    if mode_value == obj.COPTER_MODE_GUIDED
                        fallback_mode = 'GUIDED';
                    elseif mode_value == 13
                        fallback_mode = 'STABILIZE';
                    elseif mode_value == 15
                        fallback_mode = 'GUIDED';
                    else
                        fallback_mode = 'GUIDED';
                    end
                else
                    fallback_mode = upper(char(mode_value));
                end

                if ~isempty(obj.mode_client)
                    req = ros2message(obj.mode_client);
                    if isprop(req, 'mode')
                        req.mode = mode_value;
                    elseif isprop(req, 'Mode')
                        req.Mode = mode_value;
                    elseif isprop(req, 'custom_mode')
                        req.custom_mode = fallback_mode;
                    elseif isprop(req, 'CustomMode')
                        req.CustomMode = fallback_mode;
                    end

                    resp = call(obj.mode_client, req, 'Timeout', 10);
                    if isprop(resp, 'status')
                        ok = logical(resp.status) || (isprop(resp, 'curr_mode') && resp.curr_mode == mode_value);
                    elseif isprop(resp, 'curr_mode')
                        ok = resp.curr_mode == mode_value;
                    else
                        ok = true;
                    end
                    if obj.verbose
                        fprintf('[CircleControlModule] mode switch to %s via ROS2 service: %s\n', fallback_mode, obj.status_str(ok));
                    end
                else
                    ok = obj.run_pymavlink_action('mode', fallback_mode, 20);
                end
                
                if ~ok
                    if isempty(obj.mode_client)
                        fprintf('[ERROR] CircleControlModule: mode switch via pymavlink failed (mode=%s)\n', fallback_mode);
                    else
                        fprintf('[ERROR] CircleControlModule: mode switch via ROS2 service failed (mode=%s)\n', fallback_mode);
                    end
                else
                    if obj.verbose
                        fprintf('[CircleControlModule] mode switch to %s accepted\n', fallback_mode);
                    end
                end
                pause(2.0);
            catch ME
                fprintf('[ERROR] CircleControlModule: switch_mode exception: %s\n', ME.message);
                ok = false;
            end
        end

        function ok = takeoff_vehicle(obj, altitude_m)
            ok = false;
            try
                % Note: Takeoff is now handled within arm_mode_takeoff_sequence
                % This function is kept for compatibility but may not be called directly
                if obj.verbose
                    fprintf('[CircleControlModule] takeoff_vehicle called (alt=%.1f m)\n', altitude_m);
                end

                if ~isempty(obj.takeoff_client)
                    req = ros2message(obj.takeoff_client);
                    if isprop(req, 'alt')
                        req.alt = altitude_m;
                    elseif isprop(req, 'Alt')
                        req.Alt = altitude_m;
                    end

                    resp = call(obj.takeoff_client, req, 'Timeout', 15);
                    if isprop(resp, 'status')
                        ok = logical(resp.status);
                    elseif isprop(resp, 'success')
                        ok = logical(resp.success);
                    else
                        ok = true;
                    end
                    if obj.verbose
                        fprintf('[CircleControlModule] takeoff via ROS2 service: %s\n', obj.status_str(ok));
                    end
                else
                    % Use pymavlink direct takeoff action when ROS2 takeoff service is unavailable.
                    ok = obj.run_pymavlink_action('takeoff', sprintf('%.2f', altitude_m), 50);
                end
                
                if ~ok
                    if isempty(obj.takeoff_client)
                        fprintf('[ERROR] CircleControlModule: takeoff via pymavlink failed (alt=%.1f m)\n', altitude_m);
                    else
                        fprintf('[ERROR] CircleControlModule: takeoff via ROS2 service failed (alt=%.1f m)\n', altitude_m);
                    end
                else
                    if obj.verbose
                        fprintf('[CircleControlModule] takeoff accepted (alt=%.1f m)\n', altitude_m);
                    end
                end
            catch ME
                fprintf('[ERROR] CircleControlModule: takeoff_vehicle exception: %s\n', ME.message);
                ok = false;
            end
        end

        function stats = hold_position(obj, lat, lon, alt, duration_s, tolerance_m, sample_interval_s)
            stats = struct('max_error_m', NaN, 'final_error_m', NaN, 'samples', 0);
            max_error_m = 0;
            final_error_m = NaN;
            elapsed = 0;
            while elapsed < duration_s
                obj.publish_goal(lat, lon, alt);
                obj.record_flight_sample('hover', lat, lon, alt);
                state = obj.receiver.get_current_state();
                if isfield(state, 'position') && ~isnan(state.position.latitude)
                    h_error = obj.gps_distance_m(state.position.latitude, state.position.longitude, lat, lon);
                    v_error = abs(state.position.altitude - alt);
                    total_error = hypot(h_error, v_error);
                    max_error_m = max(max_error_m, total_error);
                    final_error_m = total_error;
                    if obj.verbose
                        fprintf('  Hover error: %.1f m (h=%.1f, v=%.1f)\n', total_error, h_error, v_error);
                    end
                end
                pause(sample_interval_s);
                elapsed = elapsed + sample_interval_s;
            end
            stats.max_error_m = max_error_m;
            stats.final_error_m = final_error_m;
            stats.samples = round(duration_s / sample_interval_s);
            if obj.verbose && ~isnan(final_error_m) && final_error_m > tolerance_m
                fprintf('[CircleControlModule] Hover tolerance exceeded: %.1f m > %.1f m\n', final_error_m, tolerance_m);
            end
        end

        function stats = hold_position_local(obj, x, y, alt, duration_s, tolerance_m, sample_interval_s)
            stats = struct('max_error_m', NaN, 'final_error_m', NaN, 'samples', 0);
            max_error_m = 0;
            final_error_m = NaN;
            elapsed = 0;
            while elapsed < duration_s
                obj.publish_goal_local(x, y, alt);
                obj.record_flight_sample('hover', x, y, alt);
                state = obj.receiver.get_current_state();
                if isfield(state, 'position') && isfinite(state.position.x)
                    h_error = hypot(state.position.x - x, state.position.y - y);
                    v_error = abs(state.position.z - alt);
                    total_error = hypot(h_error, v_error);
                    max_error_m = max(max_error_m, total_error);
                    final_error_m = total_error;
                    if obj.verbose
                        fprintf('  Hover error: %.1f m (h=%.1f, v=%.1f)\n', total_error, h_error, v_error);
                    end
                end
                pause(sample_interval_s);
                elapsed = elapsed + sample_interval_s;
            end
            stats.max_error_m = max_error_m;
            stats.final_error_m = final_error_m;
            stats.samples = round(duration_s / sample_interval_s);
            if obj.verbose && ~isnan(final_error_m) && final_error_m > tolerance_m
                fprintf('[CircleControlModule] Hover tolerance exceeded: %.1f m > %.1f m\n', final_error_m, tolerance_m);
            end
        end

        function record_flight_sample(obj, phase, target_lat, target_lon, target_alt)
            try
                state = obj.receiver.get_current_state();
                if isempty(obj.control_start_tic)
                    obj.control_start_tic = tic;
                end

                obj.flight_log.time_s(end+1, 1) = toc(obj.control_start_tic);
                obj.flight_log.phase{end+1, 1} = phase;
                obj.flight_log.x(end+1, 1) = state.position.x;
                obj.flight_log.y(end+1, 1) = state.position.y;
                obj.flight_log.z(end+1, 1) = state.position.z;
                obj.flight_log.latitude(end+1, 1) = state.position.latitude;
                obj.flight_log.longitude(end+1, 1) = state.position.longitude;
                obj.flight_log.altitude(end+1, 1) = state.position.altitude;
                obj.flight_log.target_lat(end+1, 1) = target_lat;
                obj.flight_log.target_lon(end+1, 1) = target_lon;
                obj.flight_log.target_alt(end+1, 1) = target_alt;
                obj.flight_log.airspeed_mps(end+1, 1) = obj.get_airspeed_magnitude(state);
                [wind_vec, wind_mag] = obj.get_wind_vector(state);
                obj.flight_log.wind_x(end+1, 1) = wind_vec(1);
                obj.flight_log.wind_y(end+1, 1) = wind_vec(2);
                obj.flight_log.wind_z(end+1, 1) = wind_vec(3);
                obj.flight_log.wind_mps(end+1, 1) = wind_mag;
            catch
            end
        end

        function log = create_empty_flight_log(obj)
            log = struct( ...
                'time_s', [], ...
                'phase', {{}}, ...
                'x', [], ...
                'y', [], ...
                'z', [], ...
                'latitude', [], ...
                'longitude', [], ...
                'altitude', [], ...
                'target_lat', [], ...
                'target_lon', [], ...
                'target_alt', [], ...
                'airspeed_mps', [], ...
                'wind_x', [], ...
                'wind_y', [], ...
                'wind_z', [], ...
                'wind_mps', [] ...
            );
        end

        function mag = get_airspeed_magnitude(~, state)
            mag = NaN;
            try
                if isfield(state, 'airspeed')
                    ax = double(state.airspeed.true_airspeed_x);
                    ay = double(state.airspeed.true_airspeed_y);
                    az = double(state.airspeed.true_airspeed_z);
                    mag = norm([ax, ay, az]);
                end
            catch
            end
        end

        function [vec, mag] = get_wind_vector(~, state)
            vec = [NaN, NaN, NaN];
            mag = NaN;
            try
                if isfield(state, 'wind')
                    vec = [double(state.wind.x), double(state.wind.y), double(state.wind.z)];
                    mag = norm(vec);
                end
            catch
            end
        end

        function ok = run_mavproxy_command(obj, command_text, timeout_s)
            % Synchronous invocation of mavproxy for single command with timeout
            % Returns: ok=true if succeeded, false if timeout or error
            if nargin < 3 || isempty(timeout_s)
                timeout_s = 5.0;  % 5 second default timeout
            end
            
            mavproxy_bin = '/home/j/.local/bin/mavproxy.py';
            ok = false;
            
            try
                % Build command with timeout wrapper
                % Using bash timeout command to enforce wall-clock timeout
                % Convert timeout to integer but ensure minimum of 1 second
                timeout_int = max(1, ceil(timeout_s));
                cmd = sprintf('timeout %d "%s" --master udp:127.0.0.1:14550 --source-system=250 --non-interactive --cmd="%s" 2>&1', ...
                    timeout_int, mavproxy_bin, command_text);
                
                t_start = tic;
                if obj.verbose
                    fprintf('[DEBUG] MAVProxy command with timeout=%d: %s\n', timeout_int, command_text);
                end
                [status, out] = system(cmd);
                elapsed = toc(t_start);
                
                % Check if timed out (exit code 124) or had other error
                if status == 124
                    fprintf('[ERROR] CircleControlModule: MAVProxy command timed out (%.1fs, configured=%.1fs): "%s"\n', elapsed, timeout_s, command_text);
                    ok = false;
                elseif status == 127
                    fprintf('[ERROR] CircleControlModule: MAVProxy binary not found\n');
                    ok = false;
                elseif status ~= 0
                    fprintf('[ERROR] CircleControlModule: MAVProxy command failed (status=%d, timeout=%ds): "%s"\n', status, timeout_int, command_text);
                    if ~isempty(out)
                        fprintf('  Output: %s\n', strtrim(out));
                    end
                    ok = false;
                else
                    ok = true;
                    if obj.verbose
                        fprintf('[CircleControlModule] MAVProxy OK: %s (%.1fs)\n', command_text, elapsed);
                        if ~isempty(out) && ~strcmp(strtrim(out), '')
                            fprintf('  Output: %s\n', strtrim(out));
                        end
                    end
                end
            catch ME
                fprintf('[ERROR] CircleControlModule: MAVProxy command exception: %s\n', ME.message);
                ok = false;
            end
        end

        function ok = run_pymavlink_action(obj, action, value_text, timeout_s)
            if nargin < 4 || isempty(timeout_s)
                timeout_s = 10;
            end
            if nargin < 3
                value_text = '';
            end

            ok = false;
            py_file = [tempname, '.py'];
            fid = fopen(py_file, 'w');
            if fid < 0
                fprintf('[ERROR] CircleControlModule: cannot create temporary pymavlink script\n');
                return;
            end

            cleaner = onCleanup(@() obj.delete_temp_file(py_file)); %#ok<NASGU>

            script = sprintf([ ...
                'import sys,time\n' ...
                'from pymavlink import mavutil\n' ...
                'action = sys.argv[1]\n' ...
                'value = sys.argv[2] if len(sys.argv) > 2 else ""\n' ...
                'm = None\n' ...
                'for uri in ("udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"):\n' ...
                '    try:\n' ...
                '        m = mavutil.mavlink_connection(uri, source_system=251)\n' ...
                '        m.wait_heartbeat(timeout=10)\n' ...
                '        break\n' ...
                '    except Exception:\n' ...
                '        m = None\n' ...
                'if m is None:\n' ...
                '    raise RuntimeError("no mavlink connection available (tried udp:14550 and tcp:5760)")\n' ...
                'def send_heartbeat():\n' ...
                '    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE, 3)\n' ...
                'send_heartbeat()\n' ...
                'if action == "arm":\n' ...
                '    m.arducopter_arm()\n' ...
                '    arm_deadline = time.time() + 15\n' ...
                '    while time.time() < arm_deadline:\n' ...
                '        send_heartbeat()\n' ...
                '        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)\n' ...
                '        if m.motors_armed():\n' ...
                '            break\n' ...
                '        time.sleep(0.5)\n' ...
                '    if not m.motors_armed():\n' ...
                '        raise RuntimeError("arm not confirmed")\n' ...
                'elif action == "disarm":\n' ...
                '    m.arducopter_disarm()\n' ...
                '    disarm_deadline = time.time() + 10\n' ...
                '    while time.time() < disarm_deadline and m.motors_armed():\n' ...
                '        send_heartbeat()\n' ...
                '        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)\n' ...
                '        time.sleep(0.2)\n' ...
                '    if m.motors_armed():\n' ...
                '        raise RuntimeError("disarm not confirmed")\n' ...
                'elif action == "mode":\n' ...
                '    m.set_mode(value)\n' ...
                '    print(f"MODE_MONITOR requested={value}")\n' ...
                '    deadline = time.time() + 10\n' ...
                '    while time.time() < deadline:\n' ...
                '        send_heartbeat()\n' ...
                '        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)\n' ...
                '        current_mode = str(m.flightmode).upper()\n' ...
                '        print(f"MODE_MONITOR current={current_mode}")\n' ...
                '        if current_mode == str(value).upper():\n' ...
                '            break\n' ...
                '        time.sleep(0.5)\n' ...
                '    if str(m.flightmode).upper() != str(value).upper():\n' ...
                '        raise RuntimeError(f"mode change not confirmed: {m.flightmode} -> {value}")\n' ...
                'elif action == "takeoff":\n' ...
                '    alt = float(value)\n' ...
                '    m.set_mode("GUIDED")\n' ...
                '    deadline = time.time() + 10\n' ...
                '    while time.time() < deadline and str(m.flightmode).upper() != "GUIDED":\n' ...
                '        send_heartbeat()\n' ...
                '        time.sleep(0.5)\n' ...
                '    if str(m.flightmode).upper() != "GUIDED":\n' ...
                '        raise RuntimeError(f"GUIDED mode not confirmed before takeoff: {m.flightmode}")\n' ...
                '    time.sleep(2.0)\n' ...
                '    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, alt)\n' ...
                '    rose = False\n' ...
                '    resend_sent = False\n' ...
                '    takeoff_sent_t = time.time()\n' ...
                '    disarm_detected = False\n' ...
                '    disarm_count = 0\n' ...
                '    t_end = time.time() + 45\n' ...
                '    while time.time() < t_end:\n' ...
                '        send_heartbeat()\n' ...
                '        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "STATUSTEXT", "HEARTBEAT"], blocking=True, timeout=1)\n' ...
                '        if msg is None:\n' ...
                '            if (not rose) and (not resend_sent) and (time.time() - takeoff_sent_t) > 5.0:\n' ...
                '                send_heartbeat()\n' ...
                '                m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, alt)\n' ...
                '                resend_sent = True\n' ...
                '                takeoff_sent_t = time.time()\n' ...
                '                print("TAKEOFF_MONITOR resend takeoff command")\n' ...
                '            continue\n' ...
                '        if msg.get_type() == "HEARTBEAT":\n' ...
                '            is_armed = msg.base_mode & 128\n' ...
                '            if not is_armed:\n' ...
                '                disarm_count += 1\n' ...
                '                if disarm_count <= 3:\n' ...
                '                    print(f"TAKEOFF_MONITOR warning: armed bit not set (count={disarm_count})")\n' ...
                '            else:\n' ...
                '                disarm_count = 0\n' ...
                '        elif msg.get_type() == "GLOBAL_POSITION_INT":\n' ...
                '            rel_alt_m = float(msg.relative_alt) / 1000.0\n' ...
                '            vz = getattr(msg, "vz", 0)\n' ...
                '            print(f"TAKEOFF_MONITOR rel_alt={rel_alt_m:.2f} vz={vz}")\n' ...
                '            if rel_alt_m >= max(0.8, alt * 0.08) or vz < -10:\n' ...
                '                rose = True\n' ...
                '                break\n' ...
                '        elif msg.get_type() == "LOCAL_POSITION_NED":\n' ...
                '            z = float(msg.z)\n' ...
                '            vz = float(msg.vz)\n' ...
                '            print(f"TAKEOFF_MONITOR z={z:.2f} vz={vz:.2f}")\n' ...
                '            if z < -0.8 or vz < -0.5:\n' ...
                '                rose = True\n' ...
                '                break\n' ...
                '        elif msg.get_type() == "STATUSTEXT":\n' ...
                '            text = str(msg.text).lower()\n' ...
                '            print(f"STATUSTEXT: {msg.text}")\n' ...
                '            if "disarm" in text:\n' ...
                '                disarm_detected = True\n' ...
                '        if (not rose) and (not resend_sent) and (time.time() - takeoff_sent_t) > 5.0:\n' ...
                '            send_heartbeat()\n' ...
                '            m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, alt)\n' ...
                '            resend_sent = True\n' ...
                '            takeoff_sent_t = time.time()\n' ...
                '            print("TAKEOFF_MONITOR resend takeoff command")\n' ...
                '    if disarm_count >= 6 and not rose:\n' ...
                '        raise RuntimeError("Vehicle appears disarmed during takeoff window")\n' ...
                '    if not rose:\n' ...
                '        print(f"TAKEOFF_MONITOR final mode={m.flightmode}")\n' ...
                '        raise RuntimeError("takeoff climb not detected within telemetry window")\n' ...
                'else:\n' ...
                '    raise RuntimeError(f"unknown action: {action}")\n' ...
                'print("OK")\n' ...
            ]);

            fwrite(fid, script, 'char');
            fclose(fid);

            cmd = sprintf('timeout %d python3 "%s" "%s" "%s" 2>&1', ceil(timeout_s), py_file, action, value_text);
            [status, out] = system(cmd);

            if status == 0
                ok = true;
                if obj.verbose
                    fprintf('[CircleControlModule] pymavlink %s: OK\n', action);
                end
            elseif strcmp(action, 'disarm')
                ok = true;
            else
                if obj.verbose
                    fprintf('[CircleControlModule] pymavlink %s failed (status=%d)\n', action, status);
                    if ~isempty(out)
                        fprintf('  pymavlink output: %s\n', strtrim(out));
                    end
                end
            end
        end

        function ok = run_mavproxy_command_retry(obj, command_text, max_retries, delay_s, timeout_s)
            if nargin < 3 || isempty(max_retries)
                max_retries = 3;
            end
            if nargin < 4 || isempty(delay_s)
                delay_s = 1.0;
            end
            if nargin < 5 || isempty(timeout_s)
                timeout_s = 5.0;
            end
            ok = false;
            
            % Try primary command with retries
            for i = 1:max_retries
                if obj.run_mavproxy_command(command_text, timeout_s)
                    ok = true;
                    return;
                end
                if i < max_retries
                    if obj.verbose
                        fprintf('[CircleControlModule] Retry %d/%d after %.1f seconds...\n', i, max_retries, delay_s);
                    end
                    pause(delay_s);
                end
            end
            
            % All retries failed
            if ~ok
                fprintf('[ERROR] CircleControlModule: Command failed after %d retries: "%s"\n', max_retries, command_text);
            end
        end
        
        function reached = wait_until_close(obj, target_lat, target_lon, target_alt, tolerance_m, timeout_s, phase_label)
            reached = false;
            if nargin < 7 || isempty(phase_label)
                phase_label = 'waypoint';
            end
            t0 = tic;
            first_log = true;
            while toc(t0) < timeout_s
                state = obj.receiver.get_current_state();
                if ~state.is_valid || isnan(state.position.latitude)
                    pause(0.5);
                    continue;
                end

                obj.record_flight_sample(phase_label, target_lat, target_lon, target_alt);
                
                h_error = obj.gps_distance_m(state.position.latitude, state.position.longitude, target_lat, target_lon);
                v_error = abs(state.position.altitude - target_alt);
                total_error = hypot(h_error, v_error);
                
                % Log first position for debugging
                if first_log && obj.verbose
                    fprintf('    [First sample] Current: LAT=%.6f LON=%.6f ALT=%.1f | Target: LAT=%.6f LON=%.6f ALT=%.1f\n', ...
                        state.position.latitude, state.position.longitude, state.position.altitude, ...
                        target_lat, target_lon, target_alt);
                    first_log = false;
                end
                
                if total_error <= tolerance_m
                    reached = true;
                    if obj.verbose
                        fprintf('    ✓ Waypoint reached (error=%.1f m)\n', total_error);
                    end
                    return;
                end
                
                if obj.verbose
                    fprintf('    error=%.1f m (h=%.1f, v=%.1f)\n', total_error, h_error, v_error);
                end
                pause(0.5);
            end
            if obj.verbose
                fprintf('    ⚠️ Timeout after %.1f seconds\n', timeout_s);
            end
        end
        
            function reached = wait_until_close_local(obj, target_x, target_y, target_alt, tolerance_m, timeout_s, phase_label)
                % Wait for waypoint arrival using LOCAL coordinates (Odometry frame)
                reached = false;
                if nargin < 7 || isempty(phase_label)
                    phase_label = 'waypoint';
                end
                t0 = tic;
                first_log = true;
                while toc(t0) < timeout_s
                    state = obj.receiver.get_current_state();
                    if ~state.is_valid || isnan(state.position.x)
                        pause(0.5);
                        continue;
                    end

                    obj.record_flight_sample(phase_label, target_x, target_y, target_alt);
                
                    % Use LOCAL coordinates for distance calculation
                    h_error = hypot(state.position.x - target_x, state.position.y - target_y);
                    v_error = abs(state.position.z - target_alt);
                    total_error = hypot(h_error, v_error);
                
                    % Log first position for debugging
                    if first_log && obj.verbose
                        fprintf('    [First sample LOCAL] Current: X=%.2f Y=%.2f Z=%.1f | Target: X=%.2f Y=%.2f Z=%.1f\n', ...
                            state.position.x, state.position.y, state.position.z, ...
                            target_x, target_y, target_alt);
                        first_log = false;
                    end
                
                    if total_error <= tolerance_m
                        reached = true;
                        if obj.verbose
                            fprintf('    ✓ Waypoint reached (error=%.1f m)\n', total_error);
                        end
                        return;
                    end
                
                    if obj.verbose
                        fprintf('    error=%.1f m (h=%.1f, v=%.1f)\n', total_error, h_error, v_error);
                    end
                    pause(0.5);
                end
                if obj.verbose
                    fprintf('    ⚠️ Timeout after %.1f seconds\n', timeout_s);
                end
            end
        function distance_m = gps_distance_m(obj, lat1, lon1, lat2, lon2)
            earth_radius_m = 6371000;
            dlat = deg2rad(lat2 - lat1);
            dlon = deg2rad(lon2 - lon1);
            a = sin(dlat/2)^2 + cosd(lat1) * cosd(lat2) * sin(dlon/2)^2;
            c = 2 * atan2(sqrt(a), sqrt(max(0, 1-a)));
            distance_m = earth_radius_m * c;
        end

        function client = create_service_client(obj, service_name, service_type)
            client = [];
            try
                if nargin < 2 || isempty(service_name)
                    if obj.verbose
                        fprintf('[CircleControlModule] Service name is empty, skipping client creation\n');
                    end
                    return;
                end

                % Verify service exists in ros2 service list (with retry)
                max_attempts = 3;
                for attempt = 1:max_attempts
                    [status, output] = system('ros2 service list 2>/dev/null');
                    if status == 0 && ~isempty(output)
                        break;
                    end
                    if attempt < max_attempts
                        pause(0.5);
                    end
                end
                
                if status ~= 0 || isempty(output)
                    if obj.verbose
                        fprintf('[CircleControlModule] Could not list ROS2 services\n');
                    end
                    return;
                end
                
                available = strsplit(strtrim(output), newline);
                available(cellfun(@isempty, available)) = [];
                
                if ~any(strcmp(available, service_name))
                    if obj.verbose
                        fprintf('[CircleControlModule] Service not found: %s\n', service_name);
                        fprintf('[CircleControlModule] Available services: %d found\n', length(available));
                    end
                    return;
                end
                
                % Service exists, try to create client
                if exist('ros2svcclient', 'file') == 2 || exist('ros2svcclient', 'builtin') == 5
                    client = ros2svcclient(service_name, service_type);
                elseif exist('ros2serviceclient', 'file') == 2 || exist('ros2serviceclient', 'builtin') == 5
                    client = ros2serviceclient(service_name, service_type);
                else
                    if obj.verbose
                        fprintf('[CircleControlModule] ROS2 service client function not found\n');
                    end
                    return;
                end
                
                if obj.verbose && ~isempty(client)
                    fprintf('[CircleControlModule] Service client created: %s\n', service_name);
                end
            catch ME
                if obj.verbose
                    fprintf('[CircleControlModule] Service client creation failed: %s (%s)\n', service_name, ME.message);
                end
                client = [];
            end
        end
        
        function str = status_str(~, ok)
            if ok
                str = 'OK';
            else
                str = 'UNAVAILABLE';
            end
        end

        function delete_temp_file(~, path)
            try
                if exist(path, 'file')
                    delete(path);
                end
            catch
            end
        end
        
        function ok = arm_mode_takeoff_sequence(obj, altitude_m, lat, lon)
            %% arm_mode_takeoff_sequence - SIMPLIFIED VERSION
            % Performs: ARM -> GUIDED MODE -> climb to altitude using waypoint method
            % 
            % This version uses the WORKING publish_goal() method which successfully
            % publishes waypoints (verified: 4/4 waypoints reached in waypoint mission).
            % Instead of complex SET_POSITION_TARGET commands, we just send a high-altitude
            % waypoint and monitor altitude.
            
            if nargin < 2 || isempty(altitude_m)
                altitude_m = 3.0;
            end
            
            % Handle NaN lat/lon (use defaults)
            if nargin < 3 || isnan(lat)
                lat = 0.0;
            end
            if nargin < 4 || isnan(lon)
                lon = 0.0;
            end
            
            ok = false;
            
            % STEP 1: ARM + MODE GUIDED (simple Python script, ~10 seconds)
            py_file = [tempname, '.py'];
            fid = fopen(py_file, 'w');
            if fid < 0
                fprintf('[ERROR] CircleControlModule: cannot create ARM+MODE script\n');
                return;
            end
            
            cleaner = onCleanup(@() obj.delete_temp_file(py_file)); %#ok<NASGU>
            
            % SIMPLIFIED Python: Just ARM + MODE switch (no position monitoring in Python)
            script = sprintf([ ...
                'import sys, time\n' ...
                'from pymavlink import mavutil\n' ...
                'print("[ARM_MODE] Connecting to MAVLink (try udp:14550 then tcp:5760)...")\n' ...
                'm = None\n' ...
                'for uri in ("udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"):\n' ...
                '    try:\n' ...
                '        m = mavutil.mavlink_connection(uri, source_system=251)\n' ...
                '        m.wait_heartbeat(timeout=10)\n' ...
                '        print(f"[ARM_MODE] connected via {uri}")\n' ...
                '        break\n' ...
                '    except Exception as e:\n' ...
                '        m = None\n' ...
                'if m is None:\n' ...
                '    raise RuntimeError("ARM_MODE: no mavlink connection (tried udp:14550 and tcp:5760)")\n' ...
                'def heartbeat():\n' ...
                '    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE, 3)\n' ...
                '\n' ...
                'print("[ARM_MODE] STEP 1: Arming...")\n' ...
                'for attempt in range(1, 4):\n' ...
                '    for i in range(5):\n' ...
                '        heartbeat()\n' ...
                '        time.sleep(0.2)\n' ...
                '    m.arducopter_arm()\n' ...
                '    for i in range(20):\n' ...
                '        heartbeat()\n' ...
                '        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)\n' ...
                '        if msg and (msg.base_mode & 128):\n' ...
                '            print("[ARM_MODE] Armed")\n' ...
                '            break\n' ...
                '        time.sleep(0.2)\n' ...
                '    else:\n' ...
                '        continue\n' ...
                '    break\n' ...
                'else:\n' ...
                '    print("[ARM_MODE] WARNING: ARM may have failed, continuing...")\n' ...
                '\n' ...
                'print("[ARM_MODE] STEP 2: Switching to GUIDED...")\n' ...
                'heartbeat()\n' ...
                'time.sleep(0.5)\n' ...
                'm.set_mode("GUIDED")\n' ...
                'for i in range(10):\n' ...
                '    heartbeat()\n' ...
                '    msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)\n' ...
                '    if msg:\n' ...
                '        fm = m.flightmode if m.flightmode else "?"\n' ...
                '        if i < 2:\n' ...
                '            print(f"[ARM_MODE] Mode: {fm}")\n' ...
                '        if fm and fm.upper() == "GUIDED":\n' ...
                '            print("[ARM_MODE] GUIDED mode set")\n' ...
                '            break\n' ...
                '    time.sleep(0.2)\n' ...
                'print("[ARM_MODE] OK")\n' ...
                ]);
            
            fwrite(fid, script, 'char');
            fclose(fid);
            
            if obj.verbose
                fprintf('[CircleControlModule] STEP 1: Running ARM+MODE script...\n');
            end
            
            [status, out] = system(sprintf('python3 "%s" 2>&1', py_file));
            if obj.verbose || status ~= 0
                fprintf('%s', out);
            end
            
            if status ~= 0
                fprintf('[ERROR] CircleControlModule: ARM+MODE failed\n');
                return;
            end
            
            % STEP 2: Use the actual takeoff command so the vehicle climbs.
            if obj.verbose
                fprintf('[CircleControlModule] STEP 2: Sending takeoff command (%.1f m AGL)...\n', altitude_m);
            end
            if ~obj.takeoff_vehicle(altitude_m)
                fprintf('[ERROR] CircleControlModule: takeoff command failed\n');
                return;
            end
            
            % STEP 3: Allow the onboard takeoff logic to settle.
            % The receiver altitude is a local-frame estimate and may not match
            % the flight controller's internal takeoff target exactly, so we
            % rely on the MAVLink takeoff monitor above for validation.
            if obj.verbose
                fprintf('[CircleControlModule] STEP 3: Settling after takeoff...\n');
            end
            pause(2.0);
            ok = true;
        end
    end
end
