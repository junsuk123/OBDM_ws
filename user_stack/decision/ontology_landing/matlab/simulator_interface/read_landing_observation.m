function [observation, sim] = read_landing_observation(sim, uavState, windState, cfg)
%READ_LANDING_OBSERVATION Generate noisy pad observation and visual stability.

if should_read_marker_measurement(sim, cfg)
    [rosObservation, sim, ok] = try_read_marker_observation(sim, uavState, windState, cfg);
    if ok
        observation = rosObservation;
        return;
    end
end

positionError = norm(uavState.position(1:2));
windPenalty = min(1.0, windState.speed / max(cfg.ontology.windSpeedReference, eps));
alignmentPenalty = min(1.0, positionError / max(cfg.ontology.positionErrorReference, eps));

tagDetectedProb = 0.95 * exp(-0.45 * positionError) * exp(-0.20 * max(0.0, uavState.altitude - cfg.ontology.tagVisibleAltitude));
tagDetectedProb = max(0.0, min(1.0, tagDetectedProb));
tagDetected = rand() < tagDetectedProb;

tagCenterError = positionError + generate_sensor_noise(cfg, 'tag_center');
tagCenterError = max(0.0, tagCenterError);

visualStability = 1.0 - 0.45 * alignmentPenalty - 0.35 * windPenalty + generate_sensor_noise(cfg, 'stability');
visualStability = max(0.0, min(1.0, visualStability));

observation = struct();
observation.timestamp = uavState.timestamp;
observation.tagDetected = tagDetected;
observation.tagDetectionProbability = tagDetectedProb;
observation.tagCenterError = tagCenterError;
observation.positionError = positionError;
observation.visualStability = visualStability;
observation.altitude = uavState.altitude;
observation.verticalSpeed = uavState.velocity(3);
observation.horizontalSpeed = norm(uavState.velocity(1:2));
observation.attitude = uavState.attitude;
observation.windSpeed = windState.speed;
observation.windAcceleration = windState.acceleration;
observation.windDirectionChange = windState.directionChange;
observation.continuous = [ ...
    uavState.altitude, ...
    uavState.velocity(3), ...
    norm(uavState.velocity(1:2)), ...
    positionError, ...
    windState.speed, ...
    windState.acceleration, ...
    tagCenterError, ...
    visualStability, ...
    double(tagDetected)];
observation.isMock = true;

sim.truth.observation = observation;

end

function tf = should_read_marker_measurement(sim, cfg)
tf = isfield(sim, 'isConnected') && sim.isConnected && ...
    isfield(cfg.simulator, 'useRosMeasurements') && cfg.simulator.useRosMeasurements;
end

function [observation, sim, ok] = try_read_marker_observation(sim, uavState, windState, cfg)
ok = false;
observation = struct();

[visibleOk, tagDetected] = read_ros_bool_field('/landing_marker/visible', 'data');
if ~visibleOk
    return;
end

markerPosition = [nan, nan, nan];
markerVelocity = [0.0, 0.0, 0.0];
markerOk = false;
if tagDetected
    [markerOk, markerPosition] = read_ros_vector_field('/landing_marker/odom', 'pose.pose.position');
    [velOk, markerVelocityRead] = read_ros_vector_field('/landing_marker/odom', 'twist.twist.linear');
    if velOk
        markerVelocity = markerVelocityRead;
    end
end

if markerOk
    positionError = norm(uavState.position(1:2) - markerPosition(1:2));
else
    positionError = norm(uavState.position(1:2));
end

windPenalty = min(1.0, windState.speed / max(cfg.ontology.windSpeedReference, eps));
alignmentPenalty = min(1.0, positionError / max(cfg.ontology.positionErrorReference, eps));
tagCenterError = positionError;
visualStability = 1.0 - 0.45 * alignmentPenalty - 0.35 * windPenalty;
if ~tagDetected
    visualStability = min(visualStability, 0.25);
end
visualStability = max(0.0, min(1.0, visualStability));

observation = struct();
observation.timestamp = uavState.timestamp;
observation.tagDetected = tagDetected;
observation.tagDetectionProbability = double(tagDetected);
observation.tagCenterError = tagCenterError;
observation.positionError = positionError;
observation.visualStability = visualStability;
observation.altitude = uavState.altitude;
observation.verticalSpeed = uavState.velocity(3);
observation.horizontalSpeed = norm(uavState.velocity(1:2));
observation.attitude = uavState.attitude;
observation.windSpeed = windState.speed;
observation.windAcceleration = windState.acceleration;
observation.windDirectionChange = windState.directionChange;
observation.markerPosition = markerPosition;
observation.markerVelocity = markerVelocity;
observation.continuous = [ ...
    uavState.altitude, ...
    uavState.velocity(3), ...
    norm(uavState.velocity(1:2)), ...
    positionError, ...
    windState.speed, ...
    windState.acceleration, ...
    tagCenterError, ...
    visualStability, ...
    double(tagDetected)];
observation.isMock = false;
observation.source = '/landing_marker/visible';

sim.truth.observation = observation;
ok = true;
end

function [ok, value] = read_ros_bool_field(topic, fieldName)
ok = false;
value = false;
cmd = sprintf(['timeout 1 %s bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; timeout 0.7 ros2 topic echo "%s" std_msgs/msg/Bool --once --field "%s" 2>/dev/null'''], ...
    clean_env_command_prefix(), topic, fieldName);
[status, output] = system(cmd);
if status ~= 0 || isempty(strtrim(output))
    return;
end
text = lower(strtrim(output));
value = contains(text, 'true') || strcmp(text, '1');
ok = true;
end

function [ok, vector] = read_ros_vector_field(topic, fieldName)
ok = false;
vector = [0.0, 0.0, 0.0];
cmd = sprintf(['timeout 1 %s bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; timeout 0.7 ros2 topic echo "%s" nav_msgs/msg/Odometry --once --field "%s" 2>/dev/null'''], ...
    clean_env_command_prefix(), topic, fieldName);
[status, output] = system(cmd);
if status ~= 0 || isempty(strtrim(output))
    return;
end

values = nan(1, 3);
names = {'x', 'y', 'z'};
for idx = 1:3
    token = regexp(output, sprintf('(?m)^\\s*%s:\\s*([-+0-9.eE]+)', names{idx}), 'tokens', 'once');
    if isempty(token)
        return;
    end
    values(idx) = str2double(token{1});
end
if any(isnan(values))
    return;
end
vector = values;
ok = true;
end

function prefix = clean_env_command_prefix()
keys = {'HOME', 'USER', 'LOGNAME', 'SHELL', 'LANG', 'DISPLAY', ...
    'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', ...
    'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY'};
defaults = struct('SHELL', '/bin/bash', 'LANG', 'C.UTF-8');
parts = {'env -i'};
for i = 1:numel(keys)
    key = keys{i};
    value = getenv(key);
    if isempty(value) && isfield(defaults, key)
        value = defaults.(key);
    end
    if ~isempty(value)
        parts{end+1} = sprintf('%s="%s"', key, regexprep(value, '"', '\\"')); %#ok<AGROW>
    end
end
parts{end+1} = 'PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"';
if isempty(getenv('RMW_IMPLEMENTATION'))
    parts{end+1} = 'RMW_IMPLEMENTATION="rmw_fastrtps_cpp"';
end
if isempty(getenv('ROS_LOCALHOST_ONLY'))
    parts{end+1} = 'ROS_LOCALHOST_ONLY="0"';
end
prefix = strjoin(parts, ' ');
end
