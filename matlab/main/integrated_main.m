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

clear; clc;

fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║  Integrated Flight Control (v2)   ║\n');
fprintf('║  Module-Based Architecture         ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

% 환경 설정
fprintf('Setting up environment...\n');
matlab_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(matlab_root));

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
cleanup_obj = onCleanup(@cleanup_on_exit);

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
    
    %% 2. 센서 데이터 수집
    fprintf('═════════════════════════════════════\n');
    fprintf('2️⃣ Data Collection Phase\n');
    fprintf('═════════════════════════════════════\n\n');
    
    collector = DataCollectionModule('namespace', ns, 'verbose', true);
    collector = collector.subscribe_to_topics();
    
    fprintf('  Collecting sensor data for 5 seconds...\n\n');
    data = collector.collect_data(5);
    collector.print_summary(data);
    collector.unsubscribe_from_topics();

    % Start realtime visualizer in background to monitor odom and wind
    try
        vis = RealtimeVisualizer('namespace', ns, 'verbose', true);
        vis.init();
        vis.start_background(0.5);
    catch
        fprintf('[integrated_main] Warning: RealtimeVisualizer failed to start\n');
    end
    
    %% 3. 제어 테스트 (필수 실행)
    % 제어 테스트는 항상 실행됨
    % OBDM_SKIP_CONTROL=1로 설정하면 건너뜀
    skip_control = getenv('OBDM_SKIP_CONTROL');
    if isempty(skip_control) || ~strcmpi(skip_control, '1')
        fprintf('═════════════════════════════════════\n');
        fprintf('3️⃣ Control Test: Takeoff / Hover / Circle Flight\n');
        fprintf('═════════════════════════════════════\n');
        fprintf('⏳ Pre-flight checks...\n\n');
        
        % Verify ArduPilot is responding before control test
        fprintf('[Pre-flight] Checking ArduPilot connectivity...\n');
        [status, output] = system('timeout 2 nc -zv 127.0.0.1 5760 2>&1');
        if status == 0 || contains(output, 'succeeded')
            fprintf('[Pre-flight] ✓ ArduPilot port 5760 is responsive\n');
        else
            fprintf('[Pre-flight] ⚠ ArduPilot port 5760 may not be responsive - will use ROS2 services\n');
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
            fprintf('[Pre-flight] ⚠ Limited ROS2 services available\n');
        end

        if service_count == 0
            fprintf('[Pre-flight] ⚠ No control services available in current launch mode\n');
            fprintf('[Pre-flight] Proceeding with MAVLink fallback control path\n');
        end

        fprintf('\n');
            
            try
                controller = CircleControlModule('namespace', ns, 'verbose', true);
                controller = controller.initialize();
                fprintf('\n');
                
                % Execute flight control with explicit error handling
                success = controller.execute_circle_flight(...
                    'radius_m', 10, ...
                    'altitude_agl_m', 3, ...
                    'num_points', 8, ...
                    'tolerance_m', 3, ...
                    'waypoint_timeout_s', 10, ...
                    'hover_duration_s', 2, ...
                    'hover_tolerance_m', 4, ...
                    'sample_interval_s', 0.5);
                
                % Check if flight control completed or failed
                flight_log = controller.get_flight_log();
                waypoints_local = controller.get_last_waypoints_local();
                
                if success
                    fprintf('\n✓ Circle flight completed successfully\n');
                    if ~isempty(flight_log) && isfield(flight_log, 'time_s') && ~isempty(flight_log.time_s)
                        plot_control_trajectory(flight_log, waypoints_local, ns);
                    end
                else
                    fprintf('\n✗ Circle flight failed to complete (success=false)\n');
                    fprintf('   Check error messages above for details\n');
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
    
    %% 4. 완료
    % Ensure vehicle is landed at the end of the test (best-effort)
    try
        fprintf('\n[integrated_main] Attempting final landing (best-effort)\n');
        perform_landing(ns);
    catch ME
        fprintf('[integrated_main] Landing helper failed: %s\n', ME.message);
    end

    fprintf('\n═════════════════════════════════════\n');
    fprintf('✓ Integration test complete\n');
    fprintf('═════════════════════════════════════\n\n');
    
    sim.print_status();
    
catch ME
    fprintf('\n✗ Error: %s\n', ME.message);
    fprintf('Stack trace:\n');
    disp(ME.stack);
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
    
    try
        pm = ProcessManager('verbose', true);
        pm = pm.cleanup_all_simulation();
        pm.print_cleanup_log();
    catch ME
        fprintf('Cleanup error: %s\n', ME.message);
    end
    
    fprintf('✓ Cleanup complete\n\n');
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

    scatter3(actual_x, actual_y, actual_z, 18, 'b', 'filled');
    scatter3(actual_x(1), actual_y(1), actual_z(1), 70, 'g', 'filled');
    scatter3(actual_x(end), actual_y(end), actual_z(end), 70, 'r', 'filled');

    legend_entries = {'Actual trajectory', 'Start', 'End'};
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

function perform_landing(ns)
    % Best-effort landing helper using pymavlink. Attempts to set mode LAND and disarm.
    if nargin < 1 || isempty(ns), ns = '/iris'; end
    try
        py_file = [tempname, '.py'];
        fid = fopen(py_file, 'w');
        if fid < 0
            error('Cannot create temporary pymavlink script');
        end
        % Try common URIs
        uris = ["tcp:127.0.0.1:5760", "udp:127.0.0.1:14550", "tcp:127.0.0.1:5770", "udp:127.0.0.1:14650"];
        py_uris = '[' + string(join(uris, ', ')) + ']';
        script = sprintf([ ...
            'import time, sys\n'
            'from pymavlink import mavutil\n'
            'uris = %s\n'
            'm = None\n'
            'for uri in uris:\n'
            '    try:\n'
            '        m = mavutil.mavlink_connection(uri, source_system=251, timeout=8)\n'
            '        m.wait_heartbeat(timeout=8)\n'
            '        break\n'
            '    except Exception:\n'
            '        m = None\n'
            'if m is None:\n'
            '    print("NO_MAVLINK")\n'
            '    sys.exit(2)\n'
            'try:\n'
            '    m.set_mode("LAND")\n'
            '    t0 = time.time()\n'
            '    while time.time() - t0 < 15:\n'
            '        m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)\n'
            '        if str(m.flightmode).upper() == "LAND":\n'
            '            break\n'
            '        time.sleep(0.5)\n'
            '    m.arducopter_disarm()\n'
            '    print("LAND_OK")\n'
            'except Exception as e:\n'
            '    print("LAND_FAIL", e)\n'
            '    sys.exit(3)\n'], py_uris);
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
