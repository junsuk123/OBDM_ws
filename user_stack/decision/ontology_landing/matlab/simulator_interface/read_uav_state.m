function [uavState, sim] = read_uav_state(sim, timestamp, windVector, cfg)
%READ_UAV_STATE Read or synthesize UAV state measurements.

dt = cfg.simulation.dt;
sim.currentStep = sim.currentStep + 1;

if isempty(windVector)
    windVector = cfg.wind.meanVelocity;
end

action = upper(sim.lastDecision.action);

if should_read_ros_measurement(sim, cfg)
    [rosState, sim, ok] = try_read_ros_odometry(sim, timestamp, windVector, cfg);
    if ok
        [implausibleMeasurement, sim] = detect_implausible_ros_state(sim, rosState, timestamp, cfg);
        if ~implausibleMeasurement
            [staleMeasurement, sim] = detect_stale_ros_state(sim, rosState, action, cfg);
            if ~staleMeasurement
                uavState = rosState;
                return;
            end
            if ~sim.rosStateMonitor.warned
                fprintf('[simulator_interface] Warning: stale odometry detected during LAND; using fallback motion model.\n');
                sim.rosStateMonitor.warned = true;
            end
        end
    end
end

if should_hold_last_ros_state(sim, cfg)
    uavState = sim.lastKnownState;
    uavState.timestamp = timestamp;
    uavState.velocity = [0.0, 0.0, 0.0];
    uavState.isMock = false;
    uavState.source = 'held_last_ros_odometry';
    sim.truth.position = uavState.position;
    sim.truth.velocity = uavState.velocity;
    sim.truth.attitude = uavState.attitude;
    sim.truth.altitude = uavState.altitude;
    sim.truth.timestamp = timestamp;
    sim.state = sim.truth;
    sim.lastKnownState = uavState;
    return;
end

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
uavState.localAltitude = altitude;
uavState.worldAltitude = altitude;
uavState.altitudeOffset = 0.0;
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

function [isImplausible, sim] = detect_implausible_ros_state(sim, rosState, timestamp, cfg)
isImplausible = false;

if ~isfield(sim, 'rosStateMonitor') || isempty(sim.rosStateMonitor)
    sim.rosStateMonitor = struct('lastPosition', [], 'staleCount', 0, 'warned', false, ...
        'jumpWarned', false);
end
if ~isfield(sim.rosStateMonitor, 'jumpWarned')
    sim.rosStateMonitor.jumpWarned = false;
end

lastPosition = sim.rosStateMonitor.lastPosition;
if isempty(lastPosition) || numel(lastPosition) < 3 || any(~isfinite(lastPosition))
    sim.rosStateMonitor.lastPosition = rosState.position;
    return;
end

lastTimestamp = timestamp - cfg.simulation.dt;
if isfield(sim, 'truth') && isfield(sim.truth, 'timestamp') && isfinite(sim.truth.timestamp)
    lastTimestamp = sim.truth.timestamp;
end
dt = max(timestamp - lastTimestamp, cfg.simulation.dt);
delta = rosState.position - lastPosition;
horizontalJump = norm(delta(1:2));
verticalJump = abs(delta(3));

maxPositionJump = get_simulator_value(cfg, 'rosMaxPositionJump', 8.0);
maxHorizontalSpeed = get_simulator_value(cfg, 'rosMaxHorizontalSpeed', 25.0);
maxVerticalSpeed = get_simulator_value(cfg, 'rosMaxVerticalSpeed', 8.0);
maxHorizontalJump = max(maxPositionJump, maxHorizontalSpeed * dt);
maxVerticalJump = max(max(1.5, maxVerticalSpeed * dt), 0.5 * maxPositionJump);

if horizontalJump > maxHorizontalJump || verticalJump > maxVerticalJump
    isImplausible = true;
    if ~sim.rosStateMonitor.jumpWarned
        fprintf(['[simulator_interface] Warning: rejected implausible odometry jump ' ...
            '(dxy=%.2f m, dz=%.2f m, dt=%.2f s; limits dxy<=%.2f, dz<=%.2f). Using fallback motion model.\n'], ...
            horizontalJump, verticalJump, dt, maxHorizontalJump, maxVerticalJump);
        sim.rosStateMonitor.jumpWarned = true;
    end
else
    sim.rosStateMonitor.lastPosition = rosState.position;
    sim.rosStateMonitor.jumpWarned = false;
end
end

function [isStale, sim] = detect_stale_ros_state(sim, rosState, action, cfg)
isStale = false;

if ~isfield(sim, 'rosStateMonitor') || isempty(sim.rosStateMonitor)
    sim.rosStateMonitor = struct('lastPosition', rosState.position, 'staleCount', 0, 'warned', false);
    return;
end

positionEpsilon = 0.02;
if isfield(cfg, 'simulator') && isfield(cfg.simulator, 'rosStalePositionEpsilon') && ~isempty(cfg.simulator.rosStalePositionEpsilon)
    positionEpsilon = cfg.simulator.rosStalePositionEpsilon;
end

triggerCount = 3;
if isfield(cfg, 'simulator') && isfield(cfg.simulator, 'rosStaleTriggerCount') && ~isempty(cfg.simulator.rosStaleTriggerCount)
    triggerCount = max(2, round(cfg.simulator.rosStaleTriggerCount));
end

positionDelta = norm(rosState.position - sim.rosStateMonitor.lastPosition);
sim.rosStateMonitor.lastPosition = rosState.position;

if strcmp(action, 'LAND') && positionDelta <= positionEpsilon
    sim.rosStateMonitor.staleCount = sim.rosStateMonitor.staleCount + 1;
else
    sim.rosStateMonitor.staleCount = 0;
    sim.rosStateMonitor.warned = false;
end

isStale = sim.rosStateMonitor.staleCount >= triggerCount;
end

function tf = should_hold_last_ros_state(sim, cfg)
tf = isfield(sim, 'isConnected') && sim.isConnected && ...
    isfield(cfg, 'simulator') && isfield(cfg.simulator, 'holdLastRosStateOnDropout') && ...
    cfg.simulator.holdLastRosStateOnDropout && ...
    isfield(sim, 'lastKnownState') && isstruct(sim.lastKnownState) && ...
    isfield(sim.lastKnownState, 'position') && numel(sim.lastKnownState.position) >= 3 && ...
    isfield(sim.lastKnownState, 'isMock') && ~sim.lastKnownState.isMock;
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
if ~posOk
    return;
end
if isfield(sim, 'truth') && isfield(sim.truth, 'position') && isfield(sim.truth, 'timestamp')
    dt = max(timestamp - sim.truth.timestamp, cfg.simulation.dt);
    velocity = (position - sim.truth.position) ./ max(dt, eps);
else
    velocity = [0.0, 0.0, 0.0];
end

attitude = [0.0, 0.0, 0.0];
if isfield(sim, 'truth') && isfield(sim.truth, 'attitude')
    attitude = sim.truth.attitude;
end

sim = update_ros_altitude_offset(sim, position, cfg);
localAltitude = max(0.0, position(3) - sim.rosAltitudeOffset);

uavState = struct();
uavState.timestamp = timestamp;
uavState.position = position;
uavState.velocity = velocity;
uavState.attitude = attitude;
uavState.altitude = localAltitude;
uavState.localAltitude = localAltitude;
uavState.worldAltitude = position(3);
uavState.altitudeOffset = sim.rosAltitudeOffset;
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

function sim = update_ros_altitude_offset(sim, position, cfg)
if isfield(sim, 'rosAltitudeOffset') && isfinite(sim.rosAltitudeOffset)
    return;
end
if isfield(sim, 'guidance') && isfield(sim.guidance, 'odomAltitudeOffset') && ...
        ~isempty(sim.guidance.odomAltitudeOffset) && isfinite(sim.guidance.odomAltitudeOffset)
    sim.rosAltitudeOffset = max(0.0, sim.guidance.odomAltitudeOffset);
    return;
end
takeoffAltitude = get_simulator_value(cfg, 'takeoffAltitude', 5.0);
if numel(position) >= 3 && isfinite(position(3)) && position(3) > takeoffAltitude + 0.5
    sim.rosAltitudeOffset = max(0.0, position(3) - takeoffAltitude);
else
    sim.rosAltitudeOffset = 0.0;
end
end

function [ok, vector] = read_ros_vector_field(topic, fieldName)
ok = false;
vector = [nan, nan, nan];
qosProfiles = {'best_effort', 'reliable'};
output = '';
for idx = 1:numel(qosProfiles)
    cmd = sprintf(['timeout 0.6 %s bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
        'export ROS_DOMAIN_ID=0; timeout 0.35 ros2 topic echo --no-daemon --spin-time 0.12 ' ...
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

function value = get_simulator_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'simulator') && isfield(cfg.simulator, fieldName) && ~isempty(cfg.simulator.(fieldName))
    value = double(cfg.simulator.(fieldName));
end
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
