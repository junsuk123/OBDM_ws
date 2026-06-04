%% Integrated Main - Simple Orchestration
% 각 기능 모듈을 조정하는 최상위 오케스트레이션 스크립트
%
% ✅ 업데이트 (2026-05-11):
%   - 공식 ArduPilot Gazebo 패키지 사용 (iris_runway.launch.py)
%   - 포트 정리 및 환경 변수 자동 설정 (ROS_DOMAIN_ID, GZ_VERSION)
%   - /iris 네임스페이스 자동 감지 및 모든 센서 토픽 지원
%   - 이륙/호버 인터페이스 확인 및 제어 중 실제 이동 궤적 3D 플로팅
%
% 실행 단계:
%   1. 시뮬레이션 초기화 및 시작
%   2. 센서 데이터 수집 (10초)
%   3. 제어 테스트: 원형 비행 (필수 실행)
%
% Usage:
%   기본 실행 (데이터 수집 + 제어 테스트):
%     matlab -batch "run('matlab/main/integrated_main.m');"
%
%   제어 테스트 건너뛰기:
%     export OBDM_SKIP_CONTROL=1
%     matlab -batch "run('matlab/main/integrated_main.m');"
%
%   실행 후 시뮬레이터 유지 (자동 종료 비활성화):
%     export OBDM_SKIP_CLEANUP=1
%     matlab -batch "run('matlab/main/integrated_main.m');"
%
%   또는 MATLAB에서 직접:
%     cd ~/OBDM_ws/matlab
%     addpath(genpath(pwd))
%     integrated_main

close all force;
try
    bdclose('all');
catch
end
clearvars;
clc;

fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║  Integrated Flight Control (v2)   ║\n');
fprintf('║  Module-Based Architecture         ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

% 환경 설정
fprintf('Setting up environment...\n');
matlab_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(matlab_root));
sanitize_runtime_environment();

% 커스텀 MATLAB 메시지 경로 추가
try
    ws_root = fileparts(matlab_root);
    custom_msgs_dir = fullfile(ws_root, 'ardu_ws', 'src', 'ardupilot', 'Tools', 'ros2', 'matlab_msg_gen');
    if isfolder(custom_msgs_dir)
        addpath(genpath(custom_msgs_dir));
        fprintf('  ✓ Added custom ROS2 messages\n');
    end
catch
end

setenv('ROS_DOMAIN_ID', '0');
if isempty(getenv('OBDM_ENABLE_ROVER_WHEEL_BRIDGE'))
    setenv('OBDM_ENABLE_ROVER_WHEEL_BRIDGE', '1');
end
cleanup_obj = onCleanup(@cleanup_on_exit); %#ok<NASGU>

try
    %% 1. 시뮬레이션 초기화 및 시작
    fprintf('\n═════════════════════════════════════\n');
    fprintf('1️⃣ Initializing Simulation\n');
    fprintf('═════════════════════════════════════\n\n');
    
    run_mode = getenv('OBDM_RUN_MODE');
    if isempty(run_mode)
        run_mode = 'gazebo_bridge';
    end

    ap_dds_control_config = struct( ...
        'navsat', true, ...
        'imu', true, ...
        'battery', true, ...
        'airspeed', true, ...
        'pose', true, ...
        'twist', true, ...
        'geopose', true, ...
        'static_tf', true, ...
        'status', true, ...
        'rc', true, ...
        'cmd_gps_pose', true, ...
        'arm_motors', true, ...
        'mode_switch', true, ...
        'takeoff', true, ...
        'prearm_check', true);
    
    sim = SimulationManager('verbose', true);
    sim = sim.initialize();
    if strcmpi(strtrim(run_mode), 'ap_dds_plane')
        sim = sim.start_simulation( ...
            'auto_launch', true, ...
            'wait_for_topics', true, ...
            'launch_mode', run_mode, ...
            'ap_dds_sensor_config', ap_dds_control_config);
    else
        sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true, 'launch_mode', run_mode);
    end
    sim.ros2_manager = sim.ros2_manager.auto_detect_namespace();
    
    ns = sim.get_namespace();
    fprintf('✓ Simulation ready. Namespace: %s\n', ns);

    % Extra wait for full simulation initialization
    % ArduPilot and Gazebo bridge need time to establish JSON sensor communication
    fprintf('\n⏳ Waiting for complete simulation initialization (10 seconds)...\n');
    for wait_idx = 1:10
        fprintf('  %d/10\n', wait_idx);
        pause(1.0);
    end
    fprintf('✓ Simulation initialization complete\n\n');

    % Start background helpers before data collection. Wind estimates are
    % recorded from the airspeed sensor, not from Gazebo wind commands.
    logger_mod = [];
    vis_mod = [];
    collector_mod = [];
    wind_mod = [];
    adverse_mod = [];
    target_mod = [];
    airspeed_bridge_pid = '';
    virtual_wind_pid = '';
    wind_visualizer_pid = '';
    world_name = getenv('OBDM_WORLD_NAME');
    if isempty(world_name)
        world_name = 'runway';
    end
    adverse_preset = getenv('OBDM_ADVERSE_PRESET');
    if isempty(adverse_preset)
        adverse_preset = getenv('OBDM_WIND_PRESET');
    end
    if isempty(adverse_preset)
        adverse_preset = 'boundary';
    end
    cfg = struct('namespace', ns, 'verbose', true, 'base_dir', fullfile(matlab_root, 'data'), ...
        'worldName', world_name, 'adversePreset', adverse_preset, ...
        'targetModelName', 'r1_rover', 'targetMarkerZ', 0.20, 'targetModelZ', 0.05);

    try
        airspeed_bridge_pid = start_airspeed_bridge(ns, world_name);
    catch ME
        fprintf('[integrated_main] Warning: Airspeed relay failed to start: %s\n', ME.message);
    end

    try
        virtual_wind_pid = start_virtual_wind_sensor(ns, world_name);
    catch ME
        fprintf('[integrated_main] Warning: Virtual wind sensor failed to start: %s\n', ME.message);
    end

    try
        wind_visualizer_pid = start_wind_visualizer(ns);
    catch ME
        fprintf('[integrated_main] Warning: Wind visualizer failed to start: %s\n', ME.message);
    end

    try
        wind_mod = WindControlModuleAdapter();
        wind_mod.init(cfg);
        wind_preset = getenv('OBDM_WIND_PRESET');
        if isempty(wind_preset)
            wind_preset = adverse_preset;
        end
        wind_mod.publish_once(wind_preset);
        fprintf('[integrated_main] Gazebo wind preset published: %s\n', wind_preset);
        verify_gazebo_wind(world_name);
    catch ME
        fprintf('[integrated_main] Warning: WindControlModule failed: %s\n', ME.message);
    end

    try
        target_mod = MovingLandingTargetModule();
        target_mod.init(cfg);
        target_mod.start();
    catch ME
        fprintf('[integrated_main] Warning: MovingLandingTargetModule failed: %s\n', ME.message);
    end

    enable_adverse_env = strcmpi(getenv('OBDM_ENABLE_ADVERSE_ENV'), '1') ...
        || has_wind_effect_preset(adverse_preset) ...
        || has_wind_effect_preset(wind_preset);
    if enable_adverse_env
        try
            adverse_mod = AdverseEnvironmentModule();
            adverse_mod.init(cfg);
            adverse_mod.start();
        catch ME
            fprintf('[integrated_main] Warning: AdverseEnvironmentModule failed: %s\n', ME.message);
        end
    else
        fprintf('[integrated_main] AdverseEnvironmentModule disabled (set OBDM_ENABLE_ADVERSE_ENV=1 to enable periodic wind gusts)\n');
    end

    try
        logger_mod = DataLoggerModule();
        logger_mod.init(cfg);
        logger_mod.start();
        fprintf('[integrated_main] Data logger module started (saving to matlab/data/test/<timestamp>/)\n');
    catch ME
        fprintf('[integrated_main] Warning: DataLoggerModule failed to start: %s\n', ME.message);
    end

    % Start the visualizer only in interactive desktop sessions. In batch
    % mode the timer can block progression, so keep the workflow non-blocking.
    enable_visualizer = usejava('desktop') && ~strcmpi(getenv('OBDM_DISABLE_VISUALIZER'), '1');
    if enable_visualizer
        try
            vis_mod = RealtimeVisualizerModule();
            vis_mod.init(cfg);
            vis_mod.start();
        catch ME
            fprintf('[integrated_main] Warning: RealtimeVisualizerModule failed to start: %s\n', ME.message);
            vis_mod = [];
        end
    else
        fprintf('[integrated_main] RealtimeVisualizer skipped in batch/headless mode\n');
    end

    collector_mod = DataCollectorModuleAdapter();
    collector_mod.init(cfg);
    collector_mod.start();
    
    %% 2. 센서 데이터 수집
    fprintf('═════════════════════════════════════\n');
    fprintf('2️⃣ Data Collection Phase\n');
    fprintf('═════════════════════════════════════\n\n');
    
    fprintf('  Collecting sensor data for 5 seconds...\n\n');
    data = collector_mod.collect_data(5);
    if exist('collector_mod','var') && ~isempty(collector_mod) && ~isempty(collector_mod.collector)
        collector_mod.collector.print_summary(data);
    end

    % Persist the collected virtual wind stream into the wind log.
    try
        if exist('logger_mod', 'var') && ~isempty(logger_mod) && isfield(data, 'wind') && ~isempty(data.wind)
            for idx = 1:size(data.wind, 1)
                row = data.wind(idx, :);
                if numel(row) >= 3 && all(isfinite(row(1:3)))
                    logger_mod.logger.log_wind(max(0, idx - 1), row(1), row(2), row(3), 'virtual_wind_sensor');
                end
            end
            fprintf('[integrated_main] Virtual wind samples recorded: %d\n', size(data.wind, 1));
        end
    catch ME
        fprintf('[integrated_main] Warning: Could not persist virtual wind samples: %s\n', ME.message);
    end
    
    %% 3. 제어 테스트 (필수 실행)
    % 제어 테스트는 항상 실행됨
    % OBDM_SKIP_CONTROL=1로 설정하면 건너뜀
    skip_control = getenv('OBDM_SKIP_CONTROL');
    control_test_ran = false;
    if isempty(skip_control) || ~strcmpi(skip_control, '1')
        control_test_ran = true;
        fprintf('═════════════════════════════════════\n');
        fprintf('3️⃣ Control Test: ArUco Search / Track / Landing\n');
        fprintf('═════════════════════════════════════\n');
        fprintf('⏳ Pre-flight checks...\n\n');
        
        % Verify ArduPilot is responding before control test
        fprintf('[Pre-flight] Checking ArduPilot connectivity...\n');
        port_ok = false;
        for attempt = 1:30
            [status, output] = system('timeout 2 nc -zv 127.0.0.1 5760 2>&1');
            if status == 0 || contains(output, 'succeeded')
                port_ok = true;
                break;
            end
            pause(1.0);
        end
        if port_ok
            fprintf('[Pre-flight] ✓ ArduPilot port 5760 is responsive\n');
        else
            fprintf('[Pre-flight] ArduPilot port 5760 not ready; MAVLink helper will retry fallback endpoints\n');
        end
        
        % Check available ROS2 services
        fprintf('[Pre-flight] Checking ROS2 services...\n');
        [status, output] = system('ros2 service list 2>/dev/null | grep -E "arm_motors|mode_switch|takeoff"');
        service_lines = strsplit(strtrim(output), newline);
        service_lines(cellfun(@isempty, service_lines)) = [];
        service_count = length(service_lines);
        if service_count >= 2
            fprintf('[Pre-flight] ✓ ROS2 services available (%d found)\n', service_count);
        else
            fprintf('[Pre-flight] ROS2 control services not used in this launch mode\n');
        end

        if service_count == 0
            fprintf('[Pre-flight] No ROS2 control services available in current launch mode\n');
            fprintf('[Pre-flight] Proceeding with MAVLink fallback control path\n');
        end

        fprintf('\n');
            
            try
                mission_type = getenv('OBDM_MISSION');
                if isempty(mission_type)
                    mission_type = 'aruco_landing';
                end
                fprintf('\n');

                if strcmpi(mission_type, 'aruco_landing')
                    controller = ArucoLandingControlModule('namespace', ns, 'verbose', true);
                    controller = controller.initialize();
                    success = controller.execute(...
                        'takeoff_alt_m', 5, ...
                        'search_step_m', 2, ...
                        'search_timeout_s', 180, ...
                        'search_forward_step_m', 4, ...
                        'search_max_forward_m', 16, ...
                        'search_lateral_step_m', 3, ...
                        'min_marker_samples', 2, ...
                        'approach_points', 6, ...
                        'tracking_timeout_s', 120, ...
                        'landing_timeout_s', 45);
                    flight_log = controller.get_flight_log();
                    if success
                        fprintf('\n✓ ArUco landing mission completed successfully\n');
                        if ~isempty(flight_log) && isfield(flight_log, 'time_s') && ~isempty(flight_log.time_s)
                            if exist('data_logger', 'var') && ~isempty(data_logger)
                                data_logger.stop_background();
                            end
                            plot_aruco_landing_trajectory(flight_log, ns);
                        end
                    else
                        fprintf('\n✗ ArUco landing mission failed to complete (success=false)\n');
                    end
                else
                    controller = CircleControlModule('namespace', ns, 'verbose', true);
                    controller = controller.initialize();
                    success = controller.execute_circle_flight(...
                        'course_type', 'wind_test', ...
                        'altitude_agl_m', 5, ...
                        'waypoint_spacing_m', 3, ...
                        'smooth_interpolation', true, ...
                        'land_at_end', true);
                    flight_log = controller.get_flight_log();
                    waypoints_local = controller.get_last_waypoints_local();
                    if success && ~isempty(flight_log) && isfield(flight_log, 'time_s') && ~isempty(flight_log.time_s)
                        plot_control_trajectory(flight_log, waypoints_local, ns);
                    end
                end
                
                controller.cleanup();
            catch ME
                fprintf('\n╔════════════════════════════════════╗\n');
                fprintf('║ CONTROL TEST FAILED WITH EXCEPTION ║\n');
                fprintf('╚════════════════════════════════════╝\n');
                fprintf('[ERROR] Exception during flight control:\n');
                fprintf('  Message: %s\n', ME.message);
                fprintf('  File: %s\n', ME.stack(1).file);
                fprintf('  Line: %d\n', ME.stack(1).line);
                fprintf('\nFlight control aborted. Attempting cleanup...\n');
                try
                    controller.cleanup();
                catch
                end
            end
    else
        fprintf('═════════════════════════════════════\n');
        fprintf('3️⃣ Control Test: SKIPPED\n');
        fprintf('═════════════════════════════════════\n\n');
        fprintf('(제어 테스트를 실행하려면 OBDM_SKIP_CONTROL를 설정 해제)\n\n');
    end

    try
        fprintf('[integrated_main] Final sensor summary after control phase\n');
        if exist('collector_mod','var') && ~isempty(collector_mod) && ~isempty(collector_mod.collector)
            collector_mod.collector.print_summary(collector_mod.collector.collected_data);
        end
    catch
    end

    try
        if exist('collector_mod','var') && ~isempty(collector_mod)
            collector_mod.cleanup();
        end
    catch
    end
    
    %% 4. 완료
    % Ensure vehicle is landed at the end of the control test only.
    if exist('control_test_ran', 'var') && control_test_ran
        try
            fprintf('\n[integrated_main] Attempting final landing (best-effort)\n');
            perform_landing(ns);
        catch ME
            fprintf('[integrated_main] Landing helper failed: %s\n', ME.message);
        end
    else
        fprintf('\n[integrated_main] Final landing skipped because control test was skipped\n');
    end

    % Clean up visualizer and data logger
    try
        if exist('vis_mod', 'var') && ~isempty(vis_mod)
            vis_mod.stop();
            fprintf('[integrated_main] Visualizer stopped\n');
        end
    catch ME
        fprintf('[integrated_main] Visualizer cleanup warning: %s\n', ME.message);
    end
    
    try
        if exist('logger_mod', 'var') && ~isempty(logger_mod)
            logger_mod.cleanup();
            fprintf('[integrated_main] Data logger cleaned up\n');
        end
    catch ME
        fprintf('[integrated_main] Data logger cleanup warning: %s\n', ME.message);
    end

    try
        if exist('wind_mod', 'var') && ~isempty(wind_mod)
            wind_mod.cleanup();
        end
    catch
    end
    try
        if exist('adverse_mod', 'var') && ~isempty(adverse_mod)
            adverse_mod.cleanup();
        end
    catch
    end
    try
        if exist('target_mod', 'var') && ~isempty(target_mod)
            target_mod.cleanup();
        end
    catch
    end
    if strcmpi(getenv('OBDM_SKIP_CLEANUP'), '1')
        fprintf('[integrated_main] Airspeed relay left running with simulation (PID: %s)\n', strtrim(airspeed_bridge_pid));
        fprintf('[integrated_main] Virtual wind sensor left running with simulation (PID: %s)\n', strtrim(virtual_wind_pid));
    else
        stop_background_pid(airspeed_bridge_pid, 'Airspeed relay');
        stop_background_pid(virtual_wind_pid, 'Virtual wind sensor');
        stop_background_pid(wind_visualizer_pid, 'Wind visualizer');
    end

    fprintf('\n═════════════════════════════════════\n');
    fprintf('✓ Integration test complete\n');
    fprintf('═════════════════════════════════════\n\n');
    
    sim.print_status();
    
catch ME
    fprintf('\n✗ Error: %s\n', ME.message);
    fprintf('Stack trace:\n');
    disp(ME.stack);
    
    % Emergency cleanup on exception
    try
        if exist('vis_mod', 'var') && ~isempty(vis_mod)
            vis_mod.stop();
        end
    catch
    end
    try
        if exist('logger_mod', 'var') && ~isempty(logger_mod)
            logger_mod.cleanup();
        end
    catch
    end
    try
        if exist('wind_mod', 'var') && ~isempty(wind_mod)
            wind_mod.cleanup();
        end
    catch
    end
    try
        if exist('adverse_mod', 'var') && ~isempty(adverse_mod)
            adverse_mod.cleanup();
        end
        if exist('target_mod', 'var') && ~isempty(target_mod)
            target_mod.cleanup();
        end
    catch
    end
    try
        if exist('airspeed_bridge_pid', 'var') && ~strcmpi(getenv('OBDM_SKIP_CLEANUP'), '1')
            stop_background_pid(airspeed_bridge_pid, 'Airspeed relay');
        end
        if exist('virtual_wind_pid', 'var') && ~strcmpi(getenv('OBDM_SKIP_CLEANUP'), '1')
            stop_background_pid(virtual_wind_pid, 'Virtual wind sensor');
        end
        if exist('wind_visualizer_pid', 'var') && ~strcmpi(getenv('OBDM_SKIP_CLEANUP'), '1')
            stop_background_pid(wind_visualizer_pid, 'Wind visualizer');
        end
    catch
    end
end

function pid = start_airspeed_bridge(ns, world_name)
    pid = '';
    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    script_path = fullfile(repo_root, 'bridges', 'ros_gz', 'gz_airspeed_to_ros2.py');
    ardu_ws = SimulationConfig.get_ardu_ws_path();
    robot_name = regexprep(ns, '^/', '');
    if isempty(robot_name)
        robot_name = 'iris';
    end

    log_file = '/tmp/obdm_airspeed_bridge.log';
    cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
        'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
        'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
        'setsid python3 "%s" --namespace "%s" --world "%s" --robot "%s" --rate 5 > "%s" 2>&1 < /dev/null & echo $!'''], ...
        ardu_ws, ardu_ws, script_path, ns, world_name, robot_name, log_file);
    [status, output] = system(cmd);
    if status == 0
        pid = strtrim(output);
        fprintf('[integrated_main] Airspeed relay started (PID: %s, log: %s)\n', pid, log_file);
    else
        fprintf('[integrated_main] Airspeed relay start command failed: %s\n', strtrim(output));
    end
end

function pid = start_virtual_wind_sensor(ns, world_name)
    pid = '';
    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    script_path = fullfile(repo_root, 'bridges', 'ros_gz', 'gz_virtual_wind_sensor_to_ros2.py');
    ardu_ws = SimulationConfig.get_ardu_ws_path();
    robot_name = regexprep(ns, '^/', '');
    if isempty(robot_name)
        robot_name = 'iris';
    end

    log_file = '/tmp/obdm_virtual_wind_sensor.log';
    cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
        'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
        'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
        'setsid python3 "%s" --namespace "%s" --world "%s" --robot "%s" --rate 5 > "%s" 2>&1 < /dev/null & echo $!'''], ...
        ardu_ws, ardu_ws, script_path, ns, world_name, robot_name, log_file);
    [status, output] = system(cmd);
    if status == 0
        pid = strtrim(output);
        fprintf('[integrated_main] Virtual wind sensor started (PID: %s, log: %s)\n', pid, log_file);
    else
        fprintf('[integrated_main] Virtual wind sensor start command failed: %s\n', strtrim(output));
    end
end

function pid = start_wind_visualizer(ns)
    pid = '';
    repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
    script_path = fullfile(repo_root, 'simulator', 'visualization', 'wind_arrow_marker_node.py');
    ardu_ws = SimulationConfig.get_ardu_ws_path();

    log_file = '/tmp/obdm_wind_visualizer.log';
    cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
        'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
        'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
        'setsid python3 "%s" --namespace "%s" --rate 10 > "%s" 2>&1 < /dev/null & echo $!'''], ...
        ardu_ws, ardu_ws, script_path, ns, log_file);
    [status, output] = system(cmd);
    if status == 0
        pid = strtrim(output);
        fprintf('[integrated_main] Wind visualizer started (PID: %s, log: %s)\n', pid, log_file);
    else
        fprintf('[integrated_main] Wind visualizer start command failed: %s\n', strtrim(output));
    end
end

function stop_background_pid(pid, label)
    if nargin < 2 || isempty(label)
        label = 'Background process';
    end
    try
        if ~isempty(pid)
            system(sprintf('kill %s 2>/dev/null || true', strtrim(pid)));
            pause(0.2);
            system(sprintf('kill -9 %s 2>/dev/null || true', strtrim(pid)));
            fprintf('[integrated_main] %s stopped (PID: %s)\n', label, strtrim(pid));
        end
    catch
    end
end

function verify_gazebo_wind(world_name)
    try
        cmd = sprintf(['GZ_VERSION=harmonic gz service -s /world/%s/wind_info ' ...
            '--reqtype gz.msgs.Empty --reptype gz.msgs.Wind --timeout 2000 --req "" 2>/dev/null'], world_name);
        [status, output] = system(cmd);
        if status == 0 && contains(output, 'linear_velocity')
            one_line = regexprep(strtrim(output), '\s+', ' ');
            fprintf('[integrated_main] Gazebo wind_info: %s\n', one_line);
        else
            fprintf('[integrated_main] Warning: Gazebo wind_info service did not respond\n');
        end
    catch ME
        fprintf('[integrated_main] Warning: Gazebo wind verification failed: %s\n', ME.message);
    end
end

function cleanup_on_exit()
    skip_cleanup = getenv('OBDM_SKIP_CLEANUP');
    if strcmpi(skip_cleanup, '1')
        fprintf('\n╔════════════════════════════════════╗\n');
        fprintf('║ Cleanup: SKIPPED (OBDM_SKIP_CLEANUP=1) ║\n');
        fprintf('╚════════════════════════════════════╝\n\n');
        return;
    end

    fprintf('\n╔════════════════════════════════════╗\n');
    fprintf('║ Cleanup: Stopping Simulation       ║\n');
    fprintf('╚════════════════════════════════════╝\n\n');

    % Stop any orphan MATLAB timers first so callbacks don't keep running
    % after user interruption.
    try
        t = timerfindall;
        if ~isempty(t)
            stop(t);
            delete(t);
        end
    catch
    end
    
    try
        pm = ProcessManager('verbose', true);
        pm = pm.cleanup_all_simulation();
        pm.print_cleanup_log();
    catch ME
        fprintf('Cleanup error: %s\n', ME.message);
    end

    try
        close all force;
        bdclose('all');
    catch
    end
    
    fprintf('✓ Cleanup complete\n\n');
end

function tf = has_wind_effect_preset(preset)
    tf = false;
    if nargin < 1 || isempty(preset)
        return;
    end

    preset = lower(string(preset));
    tf = ~any(preset == ["", "none", "calm"]);
end

function plot_control_trajectory(flight_log, waypoints_local, ns)
    if nargin < 3 || isempty(ns)
        ns = '/iris';
    end

    if isempty(flight_log) || ~isfield(flight_log, 'x') || isempty(flight_log.x)
        return;
    end

    figure('Name', sprintf('OBDM Control Trajectory (%s)', ns), 'NumberTitle', 'off', 'Color', 'w');
    hold on;
    grid on;
    axis equal;
    view(3);

    actual_x = flight_log.x;
    actual_y = flight_log.y;
    actual_z = flight_log.z;

    wind_mag = [];
    wind_vec = [];
    if isfield(flight_log, 'wind_mps') && numel(flight_log.wind_mps) == numel(actual_x)
        wind_mag = flight_log.wind_mps;
    end
    if isfield(flight_log, 'wind_x') && numel(flight_log.wind_x) == numel(actual_x)
        wind_vec = [flight_log.wind_x(:), flight_log.wind_y(:), flight_log.wind_z(:)];
    end

    if ~isempty(wind_mag) && any(isfinite(wind_mag))
        scatter3(actual_x, actual_y, actual_z, 28, wind_mag, 'filled');
        colormap(gca, turbo);
        cb = colorbar;
        cb.Label.String = 'Relative wind magnitude (m/s)';
        add_wind_arrows(gca, actual_x, actual_y, actual_z, wind_vec, wind_mag);
        legend_entries = {'Actual trajectory', 'Relative wind direction/scale'};
    else
        scatter3(actual_x, actual_y, actual_z, 18, 'b', 'filled');
        legend_entries = {'Actual trajectory'};
    end
    scatter3(actual_x(1), actual_y(1), actual_z(1), 70, 'g', 'filled');
    scatter3(actual_x(end), actual_y(end), actual_z(end), 70, 'r', 'filled');

    legend_entries{end+1} = 'Start';
    legend_entries{end+1} = 'End';
    if nargin >= 2 && ~isempty(waypoints_local)
        scatter3(waypoints_local(:,1), waypoints_local(:,2), waypoints_local(:,3), 24, 'm', 'filled');
        legend_entries{end+1} = 'Planned waypoints';
    end

    xlabel('East / X (m)');
    ylabel('North / Y (m)');
    zlabel('Altitude / Z (m)');
    title(sprintf('Control Trajectory and Hover Verification: %s', ns), 'Interpreter', 'none');
    legend(legend_entries, 'Location', 'best');

    hover_mask = false(size(flight_log.phase));
    for i = 1:numel(flight_log.phase)
        if contains(string(flight_log.phase{i}), 'hover')
            hover_mask(i) = true;
        end
    end
    if any(hover_mask)
        scatter3(actual_x(hover_mask), actual_y(hover_mask), actual_z(hover_mask), 18, [0.2 0.6 0.9], 'filled');
    end

    hold off;
end

function add_wind_arrows(ax, x, y, z, wind_vec, wind_mag)
    try
        n = numel(x);
        if n < 1 || isempty(wind_vec) || size(wind_vec, 1) ~= n
            return;
        end

        max_arrows = 32;
        step = max(1, floor(n / max_arrows));
        idx = 1:step:n;
        wind_norm = sqrt(sum(wind_vec(idx, :).^2, 2));
        valid = wind_norm > 1e-6;
        idx = idx(valid);
        if isempty(idx)
            return;
        end

        speed_vals = wind_mag(idx);
        speed_vals(~isfinite(speed_vals)) = 0;
        max_speed = max(speed_vals);
        if max_speed <= 0
            max_speed = 1;
        end

        span = max([range(x), range(y), range(z), 1]);
        arrow_len = span * 0.08 * (0.25 + 0.75 * speed_vals ./ max_speed);
        wind_norm = sqrt(sum(wind_vec(idx, :).^2, 2));
        ux = wind_vec(idx, 1) ./ wind_norm .* arrow_len(:);
        uy = wind_vec(idx, 2) ./ wind_norm .* arrow_len(:);
        uz = wind_vec(idx, 3) ./ wind_norm .* arrow_len(:);

        quiver3(ax, x(idx), y(idx), z(idx), ux, uy, uz, 0, ...
            'Color', [0.05 0.2 0.45], 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
    catch
    end
end

function plot_aruco_landing_trajectory(flight_log, ns)
    if nargin < 2 || isempty(ns)
        ns = '/iris';
    end
    try
        n_traj = min([numel(flight_log.x), numel(flight_log.y), numel(flight_log.z)]);
        if n_traj < 1
            return;
        end
        x = flight_log.x(1:n_traj);
        y = flight_log.y(1:n_traj);
        z = flight_log.z(1:n_traj);

        figure('Name', sprintf('OBDM ArUco Landing (%s)', ns), 'NumberTitle', 'off', 'Color', 'w');
        hold on; grid on; axis equal; view(3);
        plot3(x, y, z, '-', 'Color', [0.10 0.33 0.85], 'LineWidth', 1.4);
        scatter3(x(end), y(end), z(end), 40, [0.10 0.33 0.85], 'filled');

        n_target = min([numel(flight_log.target_x), numel(flight_log.target_y), numel(flight_log.target_z)]);
        if n_target > 0
            target_x = flight_log.target_x(1:n_target);
            target_y = flight_log.target_y(1:n_target);
            target_z = flight_log.target_z(1:n_target);
            target_step = max(1, floor(n_target / 150));
            target_idx = 1:target_step:n_target;
            plot3(target_x, target_y, target_z, '--', 'Color', [0.85 0.25 0.55], 'LineWidth', 1.0);
            scatter3(target_x(target_idx), target_y(target_idx), target_z(target_idx), ...
                14, [0.85 0.25 0.55], 'filled');
        end

        n_marker = min([numel(flight_log.marker_x), numel(flight_log.marker_y), numel(flight_log.marker_z), ...
            numel(flight_log.marker_vx), numel(flight_log.marker_vy), numel(flight_log.marker_vz)]);
        if n_marker > 0
            marker_x = flight_log.marker_x(1:n_marker);
            marker_y = flight_log.marker_y(1:n_marker);
            marker_z = flight_log.marker_z(1:n_marker);
            marker_vx = flight_log.marker_vx(1:n_marker);
            marker_vy = flight_log.marker_vy(1:n_marker);
            marker_vz = flight_log.marker_vz(1:n_marker);
        else
            marker_x = [];
            marker_y = [];
            marker_z = [];
            marker_vx = [];
            marker_vy = [];
            marker_vz = [];
        end
        valid_marker = isfinite(marker_x) & isfinite(marker_y) & isfinite(marker_z);
        if any(valid_marker)
            scatter3(marker_x(valid_marker), marker_y(valid_marker), marker_z(valid_marker), ...
                28, 'r', 'filled');
            quiver3(marker_x(valid_marker), marker_y(valid_marker), marker_z(valid_marker), ...
                marker_vx(valid_marker), marker_vy(valid_marker), marker_vz(valid_marker), ...
                0.8, 'Color', [0.65 0.1 0.1], 'LineWidth', 1.0);
        end
        xlabel('World X / east-like (m)');
        ylabel('World Y / north-forward (m)');
        zlabel('Z / altitude (m)');
        title(sprintf('Moving ArUco Landing Target Tracking: %s', ns), 'Interpreter', 'none');
        legend({'Drone trajectory', 'Drone final point', 'Commanded waypoints', 'Marker position', 'Marker velocity'}, 'Location', 'best');
        hold off;
    catch ME
        fprintf('[plot_aruco_landing_trajectory] skipped: %s\n', ME.message);
    end
end

function perform_landing(~)
    % Best-effort landing helper using pymavlink. Attempts to set mode LAND and disarm.
    try
        py_file = [tempname, '.py'];
        fid = fopen(py_file, 'w');
        if fid < 0
            error('Cannot create temporary pymavlink script');
        end
        cleaner = onCleanup(@() delete_temp_file(py_file)); %#ok<NASGU>
        % Try common URIs
        uris = {'tcp:127.0.0.1:5760', 'udp:127.0.0.1:14550', 'tcp:127.0.0.1:5770', 'udp:127.0.0.1:14650'};
        py_uris = ['[', strjoin(cellfun(@(u)sprintf('"%s"', u), uris, 'UniformOutput', false), ', '), ']'];
        script_lines = { ...
            'import time, sys', ...
            'from pymavlink import mavutil', ...
            sprintf('uris = %s', py_uris), ...
            'm = None', ...
            'for uri in uris:', ...
            '    try:', ...
            '        m = mavutil.mavlink_connection(uri, source_system=251, timeout=8)', ...
            '        m.wait_heartbeat(timeout=8)', ...
            '        break', ...
            '    except Exception:', ...
            '        m = None', ...
            'if m is None:', ...
            '    print("NO_MAVLINK")', ...
            '    sys.exit(2)', ...
            'try:', ...
            '    m.set_mode("LAND")', ...
            '    t0 = time.time()', ...
            '    while time.time() - t0 < 15:', ...
            '        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)', ...
            '        if str(m.flightmode).upper() == "LAND":', ...
            '            break', ...
            '        time.sleep(0.5)', ...
            '    low_alt_count = 0', ...
            '    t1 = time.time()', ...
            '    while time.time() - t1 < 45:', ...
            '        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "HEARTBEAT"], blocking=True, timeout=1)', ...
            '        if msg is None:', ...
            '            continue', ...
            '        if msg.get_type() == "GLOBAL_POSITION_INT":', ...
            '            rel_alt_m = float(msg.relative_alt) / 1000.0', ...
            '            print(f"LAND_MONITOR rel_alt={rel_alt_m:.2f}")', ...
            '            if rel_alt_m <= 0.35:', ...
            '                low_alt_count += 1', ...
            '            else:', ...
            '                low_alt_count = 0', ...
            '        elif msg.get_type() == "LOCAL_POSITION_NED":', ...
            '            z_m = abs(float(msg.z))', ...
            '            print(f"LAND_MONITOR z_abs={z_m:.2f}")', ...
            '            if z_m <= 0.35:', ...
            '                low_alt_count += 1', ...
            '            else:', ...
            '                low_alt_count = 0', ...
            '        if low_alt_count >= 3:', ...
            '            break', ...
            '    m.arducopter_disarm()', ...
            '    print("LAND_OK")', ...
            'except Exception as e:', ...
            '    print("LAND_FAIL", e)', ...
            '    sys.exit(3)' ...
        };
        script = [strjoin(script_lines, newline), newline];
        fwrite(fid, script, 'char'); fclose(fid);
        cmd = sprintf('python3 "%s" 2>&1', py_file);
        [status, out] = system(cmd);
        if status == 0 && contains(out, 'LAND_OK')
            fprintf('[perform_landing] LAND command accepted\n');
        else
            fprintf('[perform_landing] LAND helper exit=%d output=%s\n', status, strtrim(out));
        end
    catch ME
        fprintf('[perform_landing] Exception: %s\n', ME.message);
    end
end

function delete_temp_file(file_path)
    try
        if exist(file_path, 'file')
            delete(file_path);
        end
    catch
    end
end

function sanitize_runtime_environment()
    % Snap-packaged IDEs export paths that break ROS 2 GUI/subscriber
    % processes by making them load /snap/core* system libraries.
    snap_vars = { ...
        'SNAP', 'SNAP_ARCH', 'SNAP_COMMON', 'SNAP_CONTEXT', 'SNAP_COOKIE', ...
        'SNAP_DATA', 'SNAP_EUID', 'SNAP_INSTANCE_NAME', 'SNAP_LAUNCHER_ARCH_TRIPLET', ...
        'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_REAL_HOME', 'SNAP_REVISION', ...
        'SNAP_UID', 'SNAP_USER_COMMON', 'SNAP_USER_DATA', 'SNAP_VERSION', ...
        'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GTK_PATH', 'GTK_MODULES', ...
        'LOCPATH', 'QT_IM_MODULE', 'XDG_DATA_HOME'};
    for idx = 1:numel(snap_vars)
        setenv(snap_vars{idx}, '');
    end
    setenv('XDG_DATA_DIRS', '/usr/local/share:/usr/share');
    setenv('XDG_CONFIG_DIRS', '/etc/xdg');
    setenv('QT_ACCESSIBILITY', '0');
    setenv('NO_AT_BRIDGE', '1');
end
