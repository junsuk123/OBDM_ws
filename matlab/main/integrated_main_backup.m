%% Integrated Main - Complete MATLAB Flight Control
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   run('main/integrated_main.m')

clear; clc;

fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║    Integrated Flight Control       ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

fprintf('Setting up environment variables...\n');

% Simulation mode selection:
%   gazebo_bridge : Gazebo + ArduPilot SITL + ROS2 bridge (default)
%   ap_dds_plane  : ArduPilot native ROS2 DDS (no Gazebo)
run_mode = getenv('OBDM_RUN_MODE');
if isempty(run_mode)
    run_mode = 'gazebo_bridge';
end

preferred_ns = getenv('OBDM_PREFERRED_NS');

setenv('ROS_DOMAIN_ID', '0');
fprintf('  ROS_DOMAIN_ID: %s\n', getenv('ROS_DOMAIN_ID'));

fprintf('  RUN_MODE: %s\n', run_mode);
if isempty(preferred_ns)
    fprintf('  Preferred Namespace: (auto)\n');
else
    fprintf('  Preferred Namespace: %s\n', preferred_ns);
end

if isempty(getenv('DISPLAY'))
    setenv('DISPLAY', ':0');
end
fprintf('  DISPLAY: %s\n', getenv('DISPLAY'));

xauth_value = getenv('XAUTHORITY');
if ~isempty(xauth_value)
    setenv('XAUTHORITY', xauth_value);
    fprintf('  XAUTHORITY: %s\n', getenv('XAUTHORITY'));
else
    fprintf('  XAUTHORITY: (not set)\n');
end

fprintf('\n');

cleanup_obj = onCleanup(@cleanup_on_exit);

try
    %% 1. Initialize
    fprintf('═════════════════════════════════════\n');
    fprintf('1️⃣ Initializing Managers\n');
    fprintf('═════════════════════════════════════\n\n');

    matlab_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(matlab_root));
    % If ros2genmsg was used to generate custom messages, add them to path
    try
        ws_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        custom_msgs_dir = fullfile(ws_root, 'ardu_ws', 'src', 'ardupilot', 'Tools', 'ros2', 'matlab_msg_gen');
        if isfolder(custom_msgs_dir)
            addpath(genpath(custom_msgs_dir));
            fprintf('  Added generated MATLAB ROS2 messages from: %s\n', custom_msgs_dir);
        end
    catch
        % non-fatal if path add fails
    end

    sim = SimulationManager('verbose', true);
    pm = ProcessManager('verbose', true);

    if ~sim.verify_workspace()
        fprintf('\n⚠️  Warning: Workspace configuration incomplete\n');
        fprintf('Attempting to continue anyway...\n\n');
    end

    fprintf('Preparing simulation runtime...\n');
    fprintf('Cleaning up existing processes...\n');
    pm = pm.cleanup_all_simulation();
    pause(1);

    fprintf('\nInitializing SimulationManager...\n');
    sim = sim.initialize();

    %% 2. Start Simulation
    fprintf('\n═════════════════════════════════════\n');
    fprintf('2️⃣ Starting Simulation\n');
    fprintf('═════════════════════════════════════\n\n');

    sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true, 'launch_mode', run_mode);

    % Re-detect namespace after launch
    sim.ros2_manager = sim.ros2_manager.auto_detect_namespace();

    %% 3. ROS2 Connection Check
    fprintf('\n═════════════════════════════════════\n');
    fprintf('3️⃣ ROS2 Connection Information\n');
    fprintf('═════════════════════════════════════\n\n');

    sim.ros2_manager.print_info();

    %% 4. Data Collection
    fprintf('═════════════════════════════════════\n');
    fprintf('4️⃣ Collecting Data (10 seconds)\n');
    fprintf('═════════════════════════════════════\n\n');

    effective_ns = preferred_ns;
    if isempty(effective_ns)
        effective_ns = sim.get_namespace();
    end

    fprintf('Using namespace for topic resolution: %s\n', effective_ns);

    resolved_tf       = SimulationConfig.resolve_topic('tf', effective_ns);
    resolved_imu      = SimulationConfig.resolve_topic('imu', effective_ns);
    resolved_odom     = SimulationConfig.resolve_topic('odometry', effective_ns);
    resolved_gps      = SimulationConfig.resolve_topic_if_available('gps', effective_ns);
    resolved_battery  = SimulationConfig.resolve_topic('battery', effective_ns);
    resolved_airspeed = SimulationConfig.resolve_topic('airspeed', effective_ns);

    try
        node = ros2node("/matlab_data_collector");

        sub_imu  = [];
        sub_gps  = [];
        sub_odom = [];
        sub_tf   = [];
        sub_battery = [];
        sub_airspeed = [];

        last_imu_stamp  = NaN;
        last_gps_stamp  = NaN;
        last_odom_stamp = NaN;
        last_tf_stamp   = NaN;
        last_battery_stamp = NaN;
        last_airspeed_stamp = NaN;

        % IMU subscriber
        try
            sub_imu = ros2subscriber(node, resolved_imu, "sensor_msgs/Imu");
            fprintf('  ✓ %s\n', resolved_imu);
        catch ME
            fprintf('  ✗ %s (subscriber create failed)\n', resolved_imu);
            fprintf('    reason: %s\n', ME.message);
        end

        % GPS subscriber
        if ~isempty(resolved_gps)
            try
                sub_gps = ros2subscriber(node, resolved_gps, "sensor_msgs/NavSatFix");
                fprintf('  ✓ %s\n', resolved_gps);
            catch ME
                fprintf('  ✗ %s (subscriber create failed)\n', resolved_gps);
                fprintf('    reason: %s\n', ME.message);
            end
        else
            fprintf('  - GPS topic not published; skipping GPS subscription\n');
        end

        % Odometry subscriber
        try
            sub_odom = ros2subscriber(node, resolved_odom, "nav_msgs/Odometry");
            fprintf('  ✓ %s\n', resolved_odom);
        catch ME
            fprintf('  ✗ %s (subscriber create failed)\n', resolved_odom);
            fprintf('    reason: %s\n', ME.message);
        end

        % TF subscriber
        try
            sub_tf = ros2subscriber(node, resolved_tf, "tf2_msgs/TFMessage");
            fprintf('  ✓ %s\n', resolved_tf);
        catch ME
            fprintf('  ✗ %s (subscriber create failed)\n', resolved_tf);
            fprintf('    reason: %s\n', ME.message);
        end

        % Battery subscriber
        try
            sub_battery = ros2subscriber(node, resolved_battery, "sensor_msgs/BatteryState");
            fprintf('  ✓ %s\n', resolved_battery);
        catch ME
            fprintf('  ✗ %s (subscriber create failed)\n', resolved_battery);
            fprintf('    reason: %s\n', ME.message);
        end

        % Airspeed subscriber
        try
            sub_airspeed = ros2subscriber(node, resolved_airspeed, "ardupilot_msgs/Airspeed");
            fprintf('  ✓ %s\n', resolved_airspeed);
        catch ME
            fprintf('  ✗ %s (subscriber create failed)\n', resolved_airspeed);
            fprintf('    reason: %s\n', ME.message);
        end

        fprintf('\n');

        data_log = struct();
        data_log.imu  = [];
        data_log.gps  = [];
        data_log.odom = [];
        data_log.tf   = [];
        data_log.battery = [];
        data_log.airspeed = [];

        for t = 1:10

            % IMU receive
            if ~isempty(sub_imu)
                try
                    [msg, status] = receive(sub_imu, 1.0);

                    if status && ~isempty(msg)
                        stamp = get_msg_stamp(msg);

                        if isnan(last_imu_stamp) || stamp ~= last_imu_stamp
                            data_log.imu = [data_log.imu; ...
                                msg.linear_acceleration.x, ...
                                msg.linear_acceleration.y, ...
                                msg.linear_acceleration.z, ...
                                msg.angular_velocity.x, ...
                                msg.angular_velocity.y, ...
                                msg.angular_velocity.z];

                            last_imu_stamp = stamp;
                        end
                    end
                catch ME
                    fprintf('  ✗ IMU receive error: %s\n', ME.message);
                    try
                        fprintf('    IMU fields:\n');
                        disp(fieldnames(msg));
                    catch
                    end
                end
            end

            % GPS receive
            if ~isempty(sub_gps)
                try
                    [msg, status] = receive(sub_gps, 1.0);

                    if status && ~isempty(msg)
                        stamp = get_msg_stamp(msg);

                        if isnan(last_gps_stamp) || stamp ~= last_gps_stamp
                            data_log.gps = [data_log.gps; ...
                                msg.latitude, ...
                                msg.longitude, ...
                                msg.altitude];

                            last_gps_stamp = stamp;
                        end
                    end
                catch ME
                    fprintf('  ✗ GPS receive error: %s\n', ME.message);
                    try
                        fprintf('    GPS fields:\n');
                        disp(fieldnames(msg));
                    catch
                    end
                end
            end

            % ODOM receive
            if ~isempty(sub_odom)
                try
                    [msg, status] = receive(sub_odom, 1.0);

                    if status && ~isempty(msg)
                        stamp = get_msg_stamp(msg);

                        if isnan(last_odom_stamp) || stamp ~= last_odom_stamp
                            data_log.odom = [data_log.odom; ...
                                msg.pose.pose.position.x, ...
                                msg.pose.pose.position.y, ...
                                msg.pose.pose.position.z, ...
                                msg.twist.twist.linear.x, ...
                                msg.twist.twist.linear.y, ...
                                msg.twist.twist.linear.z];

                            last_odom_stamp = stamp;
                        end
                    end
                catch ME
                    fprintf('  ✗ ODOM receive error: %s\n', ME.message);
                    try
                        fprintf('    ODOM fields:\n');
                        disp(fieldnames(msg));
                    catch
                    end
                end
            end

            % TF receive
            if ~isempty(sub_tf)
                try
                    [msg, status] = receive(sub_tf, 1.0);

                    if status && ~isempty(msg) && isfield(msg, 'transforms') && ~isempty(msg.transforms)
                        tf = msg.transforms(1);
                        stamp = get_tf_stamp(tf);

                        if isnan(last_tf_stamp) || stamp ~= last_tf_stamp
                            data_log.tf = [data_log.tf; ...
                                tf.transform.translation.x, ...
                                tf.transform.translation.y, ...
                                tf.transform.translation.z, ...
                                tf.transform.rotation.x, ...
                                tf.transform.rotation.y, ...
                                tf.transform.rotation.z, ...
                                tf.transform.rotation.w];

                            last_tf_stamp = stamp;
                        end
                    end
                catch ME
                    fprintf('  ✗ TF receive error: %s\n', ME.message);
                    try
                        fprintf('    TF fields:\n');
                        disp(fieldnames(msg));
                    catch
                    end
                end
            end

            % Battery receive
            if ~isempty(sub_battery)
                try
                    [msg, status] = receive(sub_battery, 1.0);

                    if status && ~isempty(msg)
                        stamp = get_msg_stamp(msg);

                        if isnan(last_battery_stamp) || stamp ~= last_battery_stamp
                            data_log.battery = [data_log.battery; ...
                                msg.percentage, ...
                                msg.voltage, ...
                                msg.current, ...
                                msg.charge, ...
                                msg.capacity, ...
                                msg.design_capacity];

                            last_battery_stamp = stamp;
                        end
                    end
                catch ME
                    fprintf('  ✗ BATTERY receive error: %s\n', ME.message);
                    try
                        fprintf('    BATTERY fields:\n');
                        disp(fieldnames(msg));
                    catch
                    end
                end
            end

            % Airspeed receive
            if ~isempty(sub_airspeed)
                try
                    [msg, status] = receive(sub_airspeed, 1.0);

                    if status && ~isempty(msg)
                        stamp = get_msg_stamp(msg);

                        if isnan(last_airspeed_stamp) || stamp ~= last_airspeed_stamp
                            data_log.airspeed = [data_log.airspeed; ...
                                msg.true_airspeed.x, ...
                                msg.true_airspeed.y, ...
                                msg.true_airspeed.z, ...
                                msg.eas_2_tas];

                            last_airspeed_stamp = stamp;
                        end
                    end
                catch ME
                    fprintf('  ✗ AIRSPEED receive error: %s\n', ME.message);
                    try
                        fprintf('    AIRSPEED fields:\n');
                        disp(fieldnames(msg));
                    catch
                    end
                end
            end

            fprintf('[%2d/10] IMU: %d, GPS: %d, ODOM: %d, TF: %d, BATTERY: %d, AIRSPEED: %d\n', ...
                t, ...
                size(data_log.imu, 1), ...
                size(data_log.gps, 1), ...
                size(data_log.odom, 1), ...
                size(data_log.tf, 1), ...
                size(data_log.battery, 1), ...
                size(data_log.airspeed, 1));

            pause(1);
        end

        fprintf('\nData Collection Summary:\n');
        fprintf('  IMU messages:  %d\n', size(data_log.imu, 1));
        fprintf('  GPS messages:  %d\n', size(data_log.gps, 1));
        fprintf('  ODOM messages: %d\n', size(data_log.odom, 1));
        fprintf('  TF messages:   %d\n', size(data_log.tf, 1));
        fprintf('  BATTERY msgs:  %d\n', size(data_log.battery, 1));
        fprintf('  AIRSPEED msgs: %d\n', size(data_log.airspeed, 1));

        clear sub_imu sub_gps sub_odom sub_tf sub_battery sub_airspeed node;

    catch ME
        fprintf('Data collection error: %s\n', ME.message);
        try
            clear sub_imu sub_gps sub_odom sub_tf sub_battery sub_airspeed node;
        catch
        end
    end

    % Optional: run control test (circle track) if requested
    try
        control_test = getenv('OBDM_CONTROL_TEST');
        if ~isempty(control_test) && strcmpi(control_test, 'circle')
            fprintf('\nRunning control test: circle flight\n');
            try
                receiver_ctl = AircraftStateReceiver('verbose', true, 'enable_logg', false);
                receiver_ctl.subscribe_to_topics();

                ns = sim.get_namespace();
                goal_topic = SimulationConfig.resolve_topic('cmd_gps_pose', ns);
                arm_service = SimulationConfig.resolve_service('arm_motors', ns);
                mode_service = SimulationConfig.resolve_service('mode_switch', ns);

                if isempty(goal_topic)
                    warning('Control test: cmd_gps_pose topic not found for namespace %s', ns);
                else
                    goal_pub = ros2publisher(goal_topic, 'ardupilot_msgs/msg/GlobalPosition');
                    arm_client = ros2serviceclient(arm_service, 'ardupilot_msgs/srv/ArmMotors');
                    mode_client = ros2serviceclient(mode_service, 'ardupilot_msgs/srv/ModeSwitch');

                    state = wait_for_valid_state_simple(receiver_ctl, 30);
                    lat0 = state.position.latitude;
                    lon0 = state.position.longitude;
                    alt0 = state.position.altitude;
                    target_alt = alt0 + 30; % 30 m AGL

                    arm_vehicle_simple(arm_client, true);
                    switch_mode_simple(mode_client, 13);
                    pause(1.0);
                    switch_mode_simple(mode_client, 15);

                    wait_until_altitude_simple(receiver_ctl, target_alt, 60);

                    track_waypoints = build_track_waypoints_simple(lat0, lon0, target_alt, 'circle', 100, 36, 1);
                    for i = 1:size(track_waypoints, 1)
                        publish_global_goal_simple(goal_pub, track_waypoints(i,1), track_waypoints(i,2), track_waypoints(i,3));
                        fprintf('  [%03d/%03d] LAT=%.6f LON=%.6f ALT=%.1f\n', i, size(track_waypoints,1), track_waypoints(i,1), track_waypoints(i,2), track_waypoints(i,3));
                        reached = wait_until_close_simple(receiver_ctl, track_waypoints(i,1), track_waypoints(i,2), track_waypoints(i,3), 30, 20);
                        if ~reached
                            warning('Control test: waypoint %d not reached in time', i);
                        end
                        pause(1.0);
                    end

                    % Return to home and disarm
                    publish_global_goal_simple(goal_pub, lat0, lon0, target_alt);
                    pause(3.0);
                    arm_vehicle_simple(arm_client, false);
                end

                receiver_ctl.unsubscribe_from_topics();
            catch ME2
                fprintf('Control test failed: %s\n', ME2.message);
            end
        end
    catch
    end

    sim.print_status();

    %% 5. Completion
    fprintf('═════════════════════════════════════\n');
    fprintf('✓ Integrated test complete\n');
    fprintf('═════════════════════════════════════\n\n');

catch ME
    fprintf('\n✗ Error during execution!\n');
    fprintf('Message: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    disp(ME.stack);
end

%% Local helper functions

function stamp = get_msg_stamp(msg)
    try
        stamp = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec) * 1e-9;
    catch
        stamp = now;
    end
end

function stamp = get_tf_stamp(tf)
    try
        stamp = double(tf.header.stamp.sec) + double(tf.header.stamp.nanosec) * 1e-9;
    catch
        stamp = now;
    end
end

function cleanup_on_exit()
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

%% Control test helper functions
function state = wait_for_valid_state_simple(receiver, timeout_s)
    t0 = tic;
    state = receiver.get_current_state();
    while toc(t0) < timeout_s
        state = receiver.get_current_state();
        if state.is_valid && isfield(state, 'position') && isfield(state.position, 'latitude') && ~isnan(state.position.latitude)
            return;
        end
        pause(0.5);
    end
    error('Timed out waiting for valid aircraft state.');
end

function wait_until_altitude_simple(receiver, target_altitude_m, timeout_s)
    t0 = tic;
    while toc(t0) < timeout_s
        state = receiver.get_current_state();
        current_alt = state.position.altitude;
        fprintf('  Altitude: %.1f m\n', current_alt);
        if current_alt >= target_altitude_m - 2
            return;
        end
        pause(1.0);
    end
    warning('Altitude target not reached before timeout.');
end

function waypoints = build_track_waypoints_simple(center_lat, center_lon, altitude_m, style, radius_m, num_points, loops)
    if nargin < 7 || isempty(loops)
        loops = 1;
    end

    waypoints = [];
    deg_per_m_lat = 1 / 111319.9;
    deg_per_m_lon = 1 / (111319.9 * cosd(center_lat));

    base_t = linspace(0, 2 * pi, num_points + 1);
    base_t(end) = [];

    for loop_index = 1:loops
        t = base_t + 2 * pi * (loop_index - 1);
        [east_m, north_m] = local_track_points_simple(style, radius_m, t);
        lat = center_lat + north_m(:) * deg_per_m_lat;
        lon = center_lon + east_m(:) * deg_per_m_lon;
        alt = altitude_m * ones(numel(lat), 1);
        waypoints = [waypoints; lat, lon, alt]; %#ok<AGROW>
    end
end

function [east_m, north_m] = local_track_points_simple(style, radius_m, t)
    switch lower(style)
        case 'square'
            corners = [
                radius_m, 0;
                0, radius_m;
                -radius_m, 0;
                0, -radius_m;
                radius_m, 0];
            segment_count = numel(t);
            segment_edges = round(linspace(1, size(corners, 1), segment_count));
            east_m = corners(segment_edges, 1);
            north_m = corners(segment_edges, 2);
        otherwise
            east_m = radius_m * cos(t);
            north_m = radius_m * sin(t);
    end
end

function publish_global_goal_simple(goal_pub, latitude, longitude, altitude)
    msg = ros2message(goal_pub);
    if isprop(msg, 'latitude')
        msg.latitude = latitude;
    end
    if isprop(msg, 'longitude')
        msg.longitude = longitude;
    end
    if isprop(msg, 'altitude')
        msg.altitude = altitude;
    end
    if isprop(msg, 'coordinate_frame')
        msg.coordinate_frame = uint8(5);
    end
    if isprop(msg, 'type_mask')
        msg.type_mask = uint8(0);
    end
    if isprop(msg, 'header')
        try
            msg.header.frame_id = 'map';
        catch
        end
    end
    send(goal_pub, msg);
end

function arm_vehicle_simple(arm_client, should_arm)
    req = ros2message(arm_client);
    if isprop(req, 'arm')
        req.arm = logical(should_arm);
    elseif isprop(req, 'value')
        req.value = logical(should_arm);
    elseif isprop(req, 'Value')
        req.Value = logical(should_arm);
    end
    try
        resp = call(arm_client, req, 'Timeout', 5);
        if isprop(resp, 'result')
            fprintf('  Arm response: %d\n', resp.result);
        end
    catch
    end
end

function switch_mode_simple(mode_client, mode_value)
    req = ros2message(mode_client);
    if isprop(req, 'mode')
        req.mode = mode_value;
    elseif isprop(req, 'Mode')
        req.Mode = mode_value;
    elseif isprop(req, 'custom_mode')
        req.custom_mode = mode_value;
    elseif isprop(req, 'CustomMode')
        req.CustomMode = mode_value;
    end
    try
        resp = call(mode_client, req, 'Timeout', 5);
        if isprop(resp, 'status')
            fprintf('  Mode switch status: %d\n', resp.status);
        end
    catch
    end
end

function reached = wait_until_close_simple(receiver, target_lat, target_lon, target_alt, tolerance_m, timeout_s)
    reached = false;
    t0 = tic;
    while toc(t0) < timeout_s
        state = receiver.get_current_state();
        if ~state.is_valid
            pause(0.5);
            continue;
        end

        if ~isfield(state.position, 'latitude') || isnan(state.position.latitude)
            pause(0.5);
            continue;
        end

        horizontal_error = gps_distance_m_simple(state.position.latitude, state.position.longitude, target_lat, target_lon);
        altitude_error = abs(state.position.altitude - target_alt);
        total_error = hypot(horizontal_error, altitude_error);

        if total_error <= tolerance_m
            reached = true;
            return;
        end

        fprintf('    error=%.1f m (horizontal=%.1f, alt=%.1f)\n', total_error, horizontal_error, altitude_error);
        pause(0.5);
    end
end

function distance_m = gps_distance_m_simple(lat1, lon1, lat2, lon2)
    earth_radius_m = 6371000;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat / 2)^2 + cosd(lat1) * cosd(lat2) * sin(dlon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(max(0, 1 - a)));
    distance_m = earth_radius_m * c;
end