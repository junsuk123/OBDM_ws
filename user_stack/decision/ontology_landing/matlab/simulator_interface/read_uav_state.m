function [uavState, sim] = read_uav_state(sim, timestamp, windVector, cfg)
%READ_UAV_STATE Read or synthesize UAV state measurements.

dt = cfg.simulation.dt;
sim.currentStep = sim.currentStep + 1;

if isempty(windVector)
    windVector = cfg.wind.meanVelocity;
end

if should_read_ros_measurement(sim, cfg)
    [rosState, sim, ok] = try_read_ros_odometry(sim, timestamp, windVector, cfg);
    if ok
        uavState = rosState;
        return;
    end
end

action = upper(sim.lastDecision.action);
if strcmp(action, 'LAND')
    descentRate = cfg.simulator.nominalDescentRate;
else
    descentRate = cfg.simulator.hoverDescentRate;
end

windHorizontal = windVector(1:2);
windSpeed = norm(windVector);

previousPosition = sim.truth.position;
previousVelocity = sim.truth.velocity;
previousAttitude = sim.truth.attitude;

horizontalDrift = cfg.simulator.windDriftGain * windHorizontal;
positionXY = previousPosition(1:2) + dt * (previousVelocity(1:2) + horizontalDrift);
altitude = max(0.0, previousPosition(3) - dt * descentRate - 0.01 * windSpeed + generate_sensor_noise(cfg, 'altitude'));
verticalVelocity = (altitude - previousPosition(3)) / dt;

velocityXY = (positionXY - previousPosition(1:2)) / dt;
rollRad = deg2rad(cfg.simulator.rollGain * windVector(2)) + generate_sensor_noise(cfg, 'attitude');
pitchRad = deg2rad(cfg.simulator.pitchGain * windVector(1)) + generate_sensor_noise(cfg, 'attitude');
yawRad = previousAttitude(3) + cfg.simulator.yawDriftGain * dt * windVector(2) + generate_sensor_noise(cfg, 'attitude');

uavState = struct();
uavState.timestamp = timestamp;
uavState.position = [positionXY, altitude];
uavState.velocity = [velocityXY, verticalVelocity];
uavState.attitude = [rollRad, pitchRad, yawRad];
uavState.altitude = altitude;
uavState.windVector = windVector;
uavState.windSpeed = windSpeed;
uavState.isMock = true;

sim.truth.position = uavState.position;
sim.truth.velocity = uavState.velocity;
sim.truth.attitude = uavState.attitude;
sim.truth.altitude = altitude;
sim.truth.timestamp = timestamp;
sim.state = sim.truth;
sim.lastKnownState = uavState;

end

function tf = should_read_ros_measurement(sim, cfg)
tf = isfield(sim, 'isConnected') && sim.isConnected && ...
    isfield(cfg.simulator, 'useRosMeasurements') && cfg.simulator.useRosMeasurements;
if ~tf
    return;
end

readEvery = 1;
if isfield(cfg.simulator, 'rosReadEveryNSteps')
    readEvery = max(1, round(cfg.simulator.rosReadEveryNSteps));
end
tf = mod(max(sim.currentStep - 1, 0), readEvery) == 0;
end

function [uavState, sim, ok] = try_read_ros_odometry(sim, timestamp, windVector, cfg)
ok = false;
uavState = struct();

ns = cfg.simulator.namespace;
if isfield(sim, 'namespace') && ~isempty(sim.namespace)
    ns = sim.namespace;
end
topic = SimulationConfig.resolve_topic('odometry', ns);

[posOk, position] = read_ros_vector_field(topic, 'pose.pose.position');
[velOk, velocity] = read_ros_vector_field(topic, 'twist.twist.linear');
if ~posOk
    return;
end
if ~velOk
    if isfield(sim, 'truth') && isfield(sim.truth, 'position') && isfield(sim.truth, 'timestamp')
        dt = max(timestamp - sim.truth.timestamp, cfg.simulation.dt);
        velocity = (position - sim.truth.position) ./ max(dt, eps);
    else
        velocity = [0.0, 0.0, 0.0];
    end
end

attitude = [0.0, 0.0, 0.0];
if isfield(sim, 'truth') && isfield(sim.truth, 'attitude')
    attitude = sim.truth.attitude;
end

uavState = struct();
uavState.timestamp = timestamp;
uavState.position = position;
uavState.velocity = velocity;
uavState.attitude = attitude;
uavState.altitude = position(3);
uavState.windVector = windVector;
uavState.windSpeed = norm(windVector);
uavState.isMock = false;
uavState.source = topic;

sim.truth.position = uavState.position;
sim.truth.velocity = uavState.velocity;
sim.truth.attitude = uavState.attitude;
sim.truth.altitude = uavState.altitude;
sim.truth.timestamp = timestamp;
sim.state = sim.truth;
sim.lastKnownState = uavState;
ok = true;
end

function [ok, vector] = read_ros_vector_field(topic, fieldName)
ok = false;
vector = [nan, nan, nan];
qosProfiles = {'reliable', 'best_effort', 'system_default'};
output = '';
for idx = 1:numel(qosProfiles)
    cmd = sprintf(['timeout 2 %s bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
        'export ROS_DOMAIN_ID=0; timeout 1.4 ros2 topic echo --no-daemon --spin-time 0.4 ' ...
        '--qos-reliability %s --once --field "%s" "%s" nav_msgs/msg/Odometry 2>/dev/null'''], ...
        clean_env_command_prefix(), qosProfiles{idx}, fieldName, topic);
    [status, candidate] = system(cmd);
    if status == 0 && ~isempty(strtrim(candidate))
        output = candidate;
        break;
    end
end
if isempty(strtrim(output))
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
    'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI', ...
    'FASTRTPS_DEFAULT_PROFILES_FILE'};
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
