%% Circle Flight Test for ArduPilot ROS 2 Waypoint Goal Interface
% Generates a circular track in MATLAB and publishes waypoints via /ap/cmd_gps_pose.
%
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   run('matlab/algorithms/circle_flight/circle_flight.m')

clear; clc;
fprintf('\n=== MATLAB Circle Flight Test ===\n');

matlab_root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(matlab_root));

TRACK_STYLE = 'circle';
TRACK_RADIUS_M = 100;
TRACK_POINTS = 36;
TRACK_LOOPS = 1;
TRACK_TOLERANCE_M = 30;
WAYPOINT_TIMEOUT_S = 20;
WAYPOINT_HOLD_S = 1.0;
ALTITUDE_AGL_M = 30;

sensor_config = struct( ...
    'navsat', true, ...
    'imu', true, ...
    'battery', true, ...
    'airspeed', false, ...
    'pose', true, ...
    'twist', true, ...
    'geopose', true, ...
    'static_tf', true, ...
    'status', true, ...
    'rc', true);

sim = [];
receiver = [];
arm_client = [];
mode_client = [];
goal_pub = [];
cleanup_obj = onCleanup(@cleanup_on_exit);

try
    fprintf('1) Initializing simulation...\n');
    sim = SimulationManager('verbose', true);
    sim = sim.initialize();
    sim = sim.start_simulation( ...
        'auto_launch', true, ...
        'wait_for_topics', true, ...
        'launch_mode', 'gazebo_bridge', ...
        'ap_dds_sensor_config', sensor_config);

    ns = sim.get_namespace();
    fprintf('  Namespace: %s\n', ns);

    goal_topic = SimulationConfig.resolve_topic('cmd_gps_pose', ns);
    arm_service = SimulationConfig.resolve_service('arm_motors', ns);
    mode_service = SimulationConfig.resolve_service('mode_switch', ns);
    prearm_service = SimulationConfig.resolve_service('prearm_check', ns);

    if isempty(goal_topic)
        error('cmd_gps_pose topic could not be resolved for namespace %s', ns);
    end

    fprintf('2) Subscribing to aircraft state...\n');
    receiver = AircraftStateReceiver('verbose', true, 'enable_logg', false);
    receiver.subscribe_to_topics();
    state = wait_for_valid_state(receiver, 30);

    lat0 = state.position.latitude;
    lon0 = state.position.longitude;
    alt0 = state.position.altitude;
    target_alt = alt0 + ALTITUDE_AGL_M;

    fprintf('  Home position: LAT=%.6f LON=%.6f ALT=%.1f m\n', lat0, lon0, alt0);
    fprintf('  Target altitude: %.1f m AGL\n', ALTITUDE_AGL_M);

    fprintf('3) Connecting ROS 2 interfaces...\n');
    arm_client = ros2serviceclient(arm_service, 'ardupilot_msgs/srv/ArmMotors');
    mode_client = ros2serviceclient(mode_service, 'ardupilot_msgs/srv/ModeSwitch');
    goal_pub = ros2publisher(goal_topic, 'ardupilot_msgs/msg/GlobalPosition');

    if ~isempty(prearm_service)
        try
            prearm_client = ros2serviceclient(prearm_service, 'std_srvs/srv/Trigger');
            prearm_req = ros2message(prearm_client);
            prearm_resp = call(prearm_client, prearm_req, 'Timeout', 5);
            fprintf('  Prearm check: success=%d | %s\n', prearm_resp.success, prearm_resp.message);
        catch ME
            fprintf('  Prearm check skipped: %s\n', ME.message);
        end
    end

    fprintf('4) Arming and switching to guided control...\n');
    arm_vehicle(arm_client, true);
    switch_mode(mode_client, 13);
    pause(1.0);
    switch_mode(mode_client, 15);

    fprintf('5) Waiting for climb...\n');
    wait_until_altitude(receiver, target_alt, 60);

    fprintf('6) Building circle trajectory...\n');
    track_waypoints = build_track_waypoints(lat0, lon0, target_alt, TRACK_STYLE, TRACK_RADIUS_M, TRACK_POINTS, TRACK_LOOPS);
    fprintf('  Waypoints: %d\n', size(track_waypoints, 1));

    fprintf('7) Following circle...\n');
    for i = 1:size(track_waypoints, 1)
        lat_cmd = track_waypoints(i, 1);
        lon_cmd = track_waypoints(i, 2);
        alt_cmd = track_waypoints(i, 3);

        publish_global_goal(goal_pub, lat_cmd, lon_cmd, alt_cmd);
        fprintf('  [%03d/%03d] LAT=%.6f LON=%.6f ALT=%.1f\n', i, size(track_waypoints, 1), lat_cmd, lon_cmd, alt_cmd);
        reached = wait_until_close(receiver, lat_cmd, lon_cmd, alt_cmd, TRACK_TOLERANCE_M, WAYPOINT_TIMEOUT_S);
        if ~reached
            warning('circle_flight:WaypointTimeout', 'Waypoint %d/%d not reached before timeout.', i, size(track_waypoints, 1));
        end
        pause(WAYPOINT_HOLD_S);
    end

    fprintf('8) Sending final loiter/home point...\n');
    publish_global_goal(goal_pub, lat0, lon0, target_alt);
    pause(3.0);

    fprintf('9) Disarming...\n');
    arm_vehicle(arm_client, false);

    fprintf('\nCircle flight test complete.\n');

catch ME
    fprintf('\n✗ Circle flight test failed: %s\n', ME.message);
    rethrow(ME);
end

function waypoints = build_track_waypoints(center_lat, center_lon, altitude_m, style, radius_m, num_points, loops)
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
        [east_m, north_m] = local_track_points(style, radius_m, t);
        lat = center_lat + north_m(:) * deg_per_m_lat;
        lon = center_lon + east_m(:) * deg_per_m_lon;
        alt = altitude_m * ones(numel(lat), 1);
        waypoints = [waypoints; lat, lon, alt]; %#ok<AGROW>
    end
end

function [east_m, north_m] = local_track_points(style, radius_m, t)
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

function publish_global_goal(goal_pub, latitude, longitude, altitude)
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
    if isprop(msg, 'Header')
        try
            msg.Header.FrameId = 'map';
        catch
        end
    end
    send(goal_pub, msg);
end

function arm_vehicle(arm_client, should_arm)
    req = ros2message(arm_client);
    if isprop(req, 'arm')
        req.arm = logical(should_arm);
    elseif isprop(req, 'value')
        req.value = logical(should_arm);
    elseif isprop(req, 'Value')
        req.Value = logical(should_arm);
    end
    resp = call(arm_client, req, 'Timeout', 5);
    if isprop(resp, 'result')
        fprintf('  Arm response: %d\n', resp.result);
    end
end

function switch_mode(mode_client, mode_value)
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
    resp = call(mode_client, req, 'Timeout', 5);
    if isprop(resp, 'status')
        fprintf('  Mode switch status: %d\n', resp.status);
    end
end

function state = wait_for_valid_state(receiver, timeout_s)
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

function wait_until_altitude(receiver, target_altitude_m, timeout_s)
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

function reached = wait_until_close(receiver, target_lat, target_lon, target_alt, tolerance_m, timeout_s)
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

        horizontal_error = gps_distance_m(state.position.latitude, state.position.longitude, target_lat, target_lon);
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

function distance_m = gps_distance_m(lat1, lon1, lat2, lon2)
    earth_radius_m = 6371000;
    dlat = deg2rad(lat2 - lat1);
    dlon = deg2rad(lon2 - lon1);
    a = sin(dlat / 2)^2 + cosd(lat1) * cosd(lat2) * sin(dlon / 2)^2;
    c = 2 * atan2(sqrt(a), sqrt(max(0, 1 - a)));
    distance_m = earth_radius_m * c;
end

function cleanup_on_exit()
    try
        if evalin('caller', 'exist(''receiver'', ''var'')')
            caller_receiver = evalin('caller', 'receiver');
            if ~isempty(caller_receiver)
                caller_receiver.unsubscribe_from_topics();
            end
        end
    catch
    end

    try
        if evalin('caller', 'exist(''sim'', ''var'')')
            caller_sim = evalin('caller', 'sim');
            if ~isempty(caller_sim)
                caller_sim.stop_simulation();
            end
        end
    catch
    end
end
