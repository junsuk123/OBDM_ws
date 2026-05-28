function sim = send_decision_command(sim, action, classProbability, confidence, cfg, uavState, observation, semantic)
%SEND_DECISION_COMMAND Generate and stream ontology-conditioned trajectory commands.

if nargin < 3 || isempty(classProbability)
    classProbability = [1.0, 0.0];
end
if nargin < 4 || isempty(confidence)
    confidence = max(classProbability);
end
if nargin < 6 || isempty(uavState)
    uavState = sim.lastKnownState;
end
if nargin < 7
    observation = struct();
end
if nargin < 8 || isempty(semantic)
    semantic = default_semantic_state();
end
semantic = complete_semantic_state(semantic);

action = upper(char(action));
if ~ismember(action, {'LAND', 'HOVER'})
    action = 'HOVER';
end

timestamp = sim.truth.timestamp;
if isfield(uavState, 'timestamp')
    timestamp = uavState.timestamp;
end

sim.lastDecision = struct();
sim.lastDecision.action = action;
sim.lastDecision.confidence = confidence;
sim.lastDecision.probability = classProbability;
sim.lastDecision.timestamp = timestamp;
sim.lastDecision.backend = cfg.simulation.backend;
sim.lastDecision.semantic = semantic;

if ~isfield(sim, 'guidance') || isempty(sim.guidance)
    sim.guidance = struct('lastCommandTimestamp', -inf, 'landModeSent', false, ...
        'lastTarget', [nan, nan, nan], 'mode', 'HOVER', 'odomOrigin', [nan, nan, nan]);
end
if ~isfield(sim.guidance, 'odomOrigin') || any(isnan(sim.guidance.odomOrigin))
    sim.guidance.odomOrigin = uavState.position;
end
if ~isfield(sim.guidance, 'odomAltitudeOffset') || isempty(sim.guidance.odomAltitudeOffset) || ...
        isnan(sim.guidance.odomAltitudeOffset)
    sim.guidance.odomAltitudeOffset = max(0.0, uavState.position(3) - cfg.simulator.takeoffAltitude);
end
sim = update_pad_ekf(sim, observation, timestamp, cfg);

if ~should_send_guidance_command(sim, timestamp, cfg)
    return;
end

trajectory = generate_ontology_trajectory(sim, classProbability, confidence, cfg, uavState, observation, semantic, timestamp);
sim.guidance.mode = trajectory.mode;
sim.guidance.trajectory = trajectory;

if strcmpi(trajectory.command, 'land')
    if ~sim.guidance.landModeSent
        if run_guidance_command('land', 0, 0, 0, 45, sim, cfg)
            sim.guidance.landModeSent = true;
        end
    end
else
    sim.guidance.lastTarget = smooth_guidance_target(sim, trajectory.target, cfg);
    localTarget = odom_target_to_local_setpoint(sim.guidance.lastTarget, sim.guidance.odomOrigin, ...
        sim.guidance.odomAltitudeOffset);
    sim.guidance.localTarget = localTarget;
    run_guidance_command('local_wp', localTarget(1), localTarget(2), localTarget(3), 25, sim, cfg);
end

sim.guidance.lastCommandTimestamp = timestamp;
end

function semantic = default_semantic_state()
semantic = struct();
semantic.windRisk = 0.5;
semantic.visualStability = 0.5;
semantic.alignmentState = 0.5;
semantic.attitudeStability = 0.5;
semantic.descentStability = 0.5;
semantic.dragRisk = 0.5;
semantic.timeEfficiency = 0.5;
semantic.energyEfficiency = 0.5;
semantic.pathLengthEstimate = 0.0;
semantic.timeToTargetEstimate = 0.0;
semantic.energyCostEstimate = 0.0;
semantic.dragForce = 0.0;
semantic.dragPower = 0.0;
semantic.relativeAirSpeed = 0.0;
semantic.vector = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5];
end

function semantic = complete_semantic_state(semantic)
defaults = default_semantic_state();
fields = fieldnames(defaults);
for idx = 1:numel(fields)
    name = fields{idx};
    if ~isfield(semantic, name) || isempty(semantic.(name))
        semantic.(name) = defaults.(name);
    end
end
end

function holdTarget = get_hold_target(sim, cfg)
if isfield(sim, 'lastKnownState') && ~isempty(sim.lastKnownState) && isfield(sim.lastKnownState, 'position')
    holdTarget = sim.lastKnownState.position;
else
    holdTarget = cfg.simulator.initialPosition;
end
end

function ok = run_guidance_command(action, x, y, z, timeout_s, sim, cfg)
ok = false;
if isfield(sim, 'isConnected') && ~sim.isConnected
    return;
end
if strcmpi(action, 'local_wp') && publish_guidance_setpoint(x, y, z)
    ok = true;
elseif strcmpi(action, 'land') && publish_guidance_land()
    ok = true;
elseif mavlink_fallback_enabled()
    ok = run_mavlink_command(action, x, y, z, timeout_s, sim, cfg);
end
end

function tf = mavlink_fallback_enabled()
value = getenv('OBDM_ENABLE_MAVLINK_FALLBACK');
tf = strcmpi(strtrim(value), '1') || strcmpi(strtrim(value), 'true');
end

function target = smooth_guidance_target(sim, target, cfg)
if ~isfield(sim, 'guidance') || ~isfield(sim.guidance, 'lastTarget') || any(isnan(sim.guidance.lastTarget))
    return;
end
alpha = cfg.guidance.smoothingAlpha;
target = (1.0 - alpha) * sim.guidance.lastTarget + alpha * target;
end

function localTarget = odom_target_to_local_setpoint(odomTarget, odomOrigin, odomAltitudeOffset)
worldDelta = odomTarget(1:2) - odomOrigin(1:2);
localTarget = [0.0, 0.0, 0.0];
% MATLAB / ROS / Gazebo trajectory generation uses world x/y/z-up. ArduPilot
% MAVLink LOCAL_NED expects x=north, y=east, z=down. The Gazebo runway
% convention used in this project maps world Y to north and world X to east.
localTarget(1) = worldDelta(2);
localTarget(2) = worldDelta(1);
% Convert Gazebo/ROS world z-up into local altitude above ArduPilot home.
% Without this offset, an odom world z such as 13.7 m can be misread as a
% local altitude command and cause an unwanted climb.
localTarget(3) = max(0.0, odomTarget(3) - odomAltitudeOffset);
end

function ok = publish_guidance_setpoint(x, y, z)
payload = sprintf(['"{header: {frame_id: local_ned}, pose: {position: {x: %.6f, y: %.6f, z: %.6f}, ' ...
    'orientation: {w: 1.0}}}"'], x, y, z);
cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; timeout 2 ros2 topic pub --once /obdm/guidance_setpoint ' ...
    'geometry_msgs/msg/PoseStamped %s >/dev/null 2>&1'''], clean_env_command_prefix(), payload);
[status, ~] = system(cmd);
ok = status == 0 || status == 124;
end

function ok = publish_guidance_land()
cmd = [clean_env_command_prefix(), ' bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; timeout 2 ros2 topic pub --once /obdm/guidance_land ' ...
    'std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1'''];
[status, ~] = system(cmd);
ok = status == 0 || status == 124;
end

function tf = should_send_guidance_command(sim, timestamp, cfg)
tf = true;
if ~isfield(cfg, 'guidance') || ~isfield(cfg.guidance, 'commandPeriod')
    return;
end
if ~isfield(sim, 'guidance') || ~isfield(sim.guidance, 'lastCommandTimestamp')
    return;
end
tf = (timestamp - sim.guidance.lastCommandTimestamp) >= cfg.guidance.commandPeriod;
end

function trajectory = generate_ontology_trajectory(sim, classProbability, confidence, cfg, uavState, observation, semantic, timestamp)
trajectory = initial_trajectory(sim, classProbability, confidence, cfg, uavState, semantic);

hasMarker = isfield(sim.guidance, 'padEkf') && sim.guidance.padEkf.initialized;
if ~hasMarker
    return;
end

trajectory.hasMarker = true;
trajectory.ekfTracking = true;
trajectory.rawTagDetected = isfield(observation, 'tagDetected') && observation.tagDetected;
trajectory.padEkfAge = max(0.0, timestamp - sim.guidance.padEkf.lastMeasurementTimestamp);
if trajectory.rawTagDetected
    trajectory.padEkfSource = 'measurement_corrected';
else
    trajectory.padEkfSource = 'prediction_from_previous_marker';
end
markerPosition = sim.guidance.padEkf.state(1:3)';
markerVelocity = sim.guidance.padEkf.state(4:6)';
uavVelocity = [0.0, 0.0, 0.0];
if isfield(uavState, 'velocity') && numel(uavState.velocity) >= 3
    uavVelocity = uavState.velocity;
end
uavAltitude = uavState.position(3);
if isfield(uavState, 'altitude')
    uavAltitude = uavState.altitude;
end

rel = markerPosition(1:2) - uavState.position(1:2);
relVel = markerVelocity(1:2) - uavVelocity(1:2);
range = norm(rel);
closingSpeed = max(0.2, -dot(rel, relVel) / max(range, eps));

landingPosterior = classProbability(min(2, numel(classProbability)));
intent = max(cfg.guidance.ontologyLandingIntentFloor, ...
    clamp01(landingPosterior * max(0.0, min(1.0, confidence))));
risk = clamp01(0.30 * semantic.windRisk + 0.25 * semantic.dragRisk + ...
    0.20 * (1.0 - semantic.energyEfficiency) + 0.15 * (1.0 - semantic.attitudeStability) + ...
    0.10 * (1.0 - semantic.visualStability));
trackQuality = clamp01(0.45 * semantic.alignmentState + 0.35 * semantic.visualStability + ...
    0.20 * semantic.attitudeStability);
efficiency = clamp01(0.55 * semantic.energyEfficiency + 0.45 * semantic.timeEfficiency);

leadTimeMax = lerp(cfg.guidance.ontologyMinLeadTime, cfg.guidance.ontologyMaxLeadTime, ...
    clamp01(trackQuality * efficiency * (1.0 - 0.5 * risk)));
leadDistanceGain = get_guidance_value(cfg, 'ontologyLeadDistanceGain', 0.0);
leadTime = min(leadTimeMax, max(cfg.guidance.ontologyMinLeadTime, range / closingSpeed + leadDistanceGain * range));
leadPoint = markerPosition(1:2) + markerVelocity(1:2) * leadTime;

navGain = lerp(cfg.guidance.ontologyMinNavigationConstant, cfg.guidance.ontologyMaxNavigationConstant, ...
    clamp01(intent * (0.30 + 0.55 * trackQuality + 0.15 * semantic.timeEfficiency) * ...
    (1.0 - 0.45 * risk)));
pnStep = navGain * (leadPoint - uavState.position(1:2));
stepNorm = norm(pnStep);
maxStep = lerp(cfg.guidance.ontologyMinHorizontalStep, cfg.guidance.ontologyMaxHorizontalStep, ...
    clamp01((0.25 + 0.55 * trackQuality + 0.20 * efficiency) * (1.0 - 0.35 * risk)));
if stepNorm > maxStep
    pnStep = pnStep / stepNorm * maxStep;
end
directStep = marker_closure_step(rel, range, maxStep, cfg);
pnStep = (1.0 - cfg.guidance.ontologyDistancePriority) * pnStep + ...
    cfg.guidance.ontologyDistancePriority * directStep;
pnStep = enforce_marker_closure_step(pnStep, rel, range, cfg);

targetXY = uavState.position(1:2) + pnStep;
descentAuthority = clamp01(intent * semantic.alignmentState * semantic.visualStability * ...
    semantic.attitudeStability * semantic.energyEfficiency * (1.0 - risk));
descentRate = lerp(cfg.guidance.ontologyMinDescentRate, cfg.guidance.ontologyMaxDescentRate, descentAuthority);
alignRadius = cfg.guidance.alignRadius * lerp(0.75, 1.25, semantic.alignmentState);
descentStartRadius = max(alignRadius, cfg.guidance.ontologyDescentStartRadius * ...
    lerp(0.65, 1.15, trackQuality) * lerp(1.05, 0.70, risk));

descentReferenceAltitude = uavAltitude;
if isfield(sim.guidance, 'lastTarget') && numel(sim.guidance.lastTarget) >= 3 && ...
        ~isnan(sim.guidance.lastTarget(3))
    descentReferenceAltitude = min(uavAltitude, sim.guidance.lastTarget(3));
end
descentDt = guidance_descent_dt(sim, timestamp, cfg);
if range <= descentStartRadius
    targetAltitude = max(cfg.labeling.touchdownAltitude, ...
        descentReferenceAltitude - descentRate * descentDt);
else
    farDescentRate = descentRate * cfg.guidance.ontologyFarDescentRateScale;
    targetAltitude = max(cfg.labeling.touchdownAltitude, ...
        descentReferenceAltitude - farDescentRate * descentDt);
end

trajectory.mode = 'ONTOLOGY_ARUCO_TRAJECTORY';
trajectory.target = [targetXY, targetAltitude];
trajectory.markerWorldPosition = markerPosition;
trajectory.markerWorldVelocity = markerVelocity;
trajectory.descentStartRadius = descentStartRadius;
trajectory.landingIntent = intent;
trajectory.leadTime = leadTime;
trajectory.navigationGain = navGain;
trajectory.maxHorizontalStep = maxStep;
trajectory.approachStep = norm(pnStep);
trajectory.distanceToMarker = range;
trajectory.descentRate = descentRate;
trajectory.trackQuality = trackQuality;
trajectory.efficiency = efficiency;
trajectory.risk = risk;
trajectory.pathLengthEstimate = norm([trajectory.target(1:2) - uavState.position(1:2), ...
    max(uavAltitude - trajectory.target(3), 0.0)]);
trajectory.timeToTargetEstimate = trajectory.pathLengthEstimate / max(norm(uavVelocity(1:2)), 0.25);
trajectory.dragForce = semantic.dragForce;
trajectory.dragPower = semantic.dragPower;
trajectory.energyCostEstimate = semantic.energyCostEstimate;
trajectory.finalLand = range <= cfg.guidance.finalLandRadius && ...
    uavAltitude <= cfg.guidance.finalLandAltitude && ...
    semantic.alignmentState >= cfg.guidance.ontologyFinalLandMinAlignment && ...
    semantic.visualStability >= cfg.guidance.ontologyFinalLandMinVisualStability && ...
    semantic.windRisk <= cfg.guidance.ontologyFinalLandMaxWindRisk && ...
    semantic.energyEfficiency >= 0.25;
if trajectory.finalLand
    trajectory.command = 'land';
end
end

function dt = guidance_descent_dt(sim, timestamp, cfg)
dt = max(cfg.simulation.dt, eps);
if isfield(sim, 'guidance') && isfield(sim.guidance, 'lastCommandTimestamp') && ...
        isfinite(sim.guidance.lastCommandTimestamp)
    dt = max(dt, timestamp - sim.guidance.lastCommandTimestamp);
end
dt = min(max(dt, eps), 1.0);
end

function trajectory = initial_trajectory(sim, classProbability, confidence, cfg, uavState, semantic)
holdTarget = get_hold_target(sim, cfg);
landingPosterior = classProbability(min(2, numel(classProbability)));
intent = max(cfg.guidance.ontologyLandingIntentFloor, ...
    clamp01(landingPosterior * max(0.0, min(1.0, confidence))));
risk = clamp01(0.35 * semantic.windRisk + 0.25 * semantic.dragRisk + ...
    0.20 * (1.0 - semantic.energyEfficiency) + 0.10 * (1.0 - semantic.attitudeStability) + ...
    0.10 * (1.0 - semantic.visualStability));
targetAltitude = uavState.position(3);
if isfield(sim, 'guidance') && isfield(sim.guidance, 'lastTarget') && ~any(isnan(sim.guidance.lastTarget))
    holdTarget = sim.guidance.lastTarget;
end
searchAltitude = get_guidance_value(cfg, 'searchAltitude', cfg.simulator.takeoffAltitude);
climbRate = get_guidance_value(cfg, 'markerLostClimbRate', 0.6);
if targetAltitude < searchAltitude
    timestamp = sim.truth.timestamp;
    if isfield(uavState, 'timestamp')
        timestamp = uavState.timestamp;
    end
    dt = guidance_descent_dt(sim, timestamp, cfg);
    targetAltitude = min(searchAltitude, max(targetAltitude, holdTarget(3)) + climbRate * dt);
end
mode = 'ONTOLOGY_SEARCH_TRAJECTORY';
trajectory = struct();
trajectory.command = 'local_wp';
trajectory.mode = mode;
trajectory.target = [holdTarget(1), holdTarget(2), targetAltitude];
trajectory.finalLand = false;
trajectory.hasMarker = false;
trajectory.ekfTracking = false;
trajectory.rawTagDetected = false;
trajectory.landingIntent = intent;
trajectory.leadTime = 0.0;
trajectory.navigationGain = 0.0;
trajectory.maxHorizontalStep = 0.0;
trajectory.descentRate = 0.0;
trajectory.trackQuality = clamp01(0.45 * semantic.alignmentState + 0.35 * semantic.visualStability + ...
    0.20 * semantic.attitudeStability);
trajectory.efficiency = clamp01(0.55 * semantic.energyEfficiency + 0.45 * semantic.timeEfficiency);
trajectory.risk = risk;
trajectory.pathLengthEstimate = semantic.pathLengthEstimate;
trajectory.timeToTargetEstimate = semantic.timeToTargetEstimate;
trajectory.dragForce = semantic.dragForce;
trajectory.dragPower = semantic.dragPower;
trajectory.energyCostEstimate = semantic.energyCostEstimate;
end

function sim = update_pad_ekf(sim, observation, timestamp, cfg)
if ~isfield(sim, 'guidance') || isempty(sim.guidance)
    return;
end
if ~isfield(sim.guidance, 'padEkf') || isempty(sim.guidance.padEkf)
    sim.guidance.padEkf = struct('initialized', false, 'state', zeros(6, 1), ...
        'covariance', eye(6), 'timestamp', timestamp, 'lastMeasurementTimestamp', -inf);
elseif ~isfield(sim.guidance.padEkf, 'lastMeasurementTimestamp')
    sim.guidance.padEkf.lastMeasurementTimestamp = sim.guidance.padEkf.timestamp;
end

ekf = sim.guidance.padEkf;
dt = max(0.0, timestamp - ekf.timestamp);
if ekf.initialized && dt > 0.0
    [state, covariance] = predict_pad_ekf(ekf.state, ekf.covariance, dt, cfg);
    ekf.state = state;
    ekf.covariance = covariance;
    ekf.timestamp = timestamp;
end

hasMeasurement = isfield(observation, 'tagDetected') && observation.tagDetected && ...
    isfield(observation, 'markerPosition') && numel(observation.markerPosition) >= 3 && ...
    ~any(isnan(observation.markerPosition(1:2)));
if ~hasMeasurement
    sim.guidance.padEkf = ekf;
    return;
end

measurement = observation.markerPosition(:);
if ~ekf.initialized
    velocity = zeros(3, 1);
    if isfield(observation, 'markerVelocity') && numel(observation.markerVelocity) >= 3
        velocity = observation.markerVelocity(:);
    end
    ekf.state = [measurement(1:3); velocity(1:3)];
    ekf.covariance = diag([0.15, 0.15, 0.08, 0.35, 0.35, 0.15].^2);
    ekf.timestamp = timestamp;
    ekf.lastMeasurementTimestamp = timestamp;
    ekf.initialized = true;
else
    [state, covariance] = correct_pad_ekf(ekf.state, ekf.covariance, measurement(1:3), cfg);
    ekf.state = state;
    ekf.covariance = covariance;
    ekf.lastMeasurementTimestamp = timestamp;
end
sim.guidance.padEkf = ekf;
end

function [state, covariance] = predict_pad_ekf(state, covariance, dt, cfg)
F = eye(6);
F(1, 4) = dt;
F(2, 5) = dt;
F(3, 6) = dt;
qPos = cfg.guidance.padEkfProcessNoisePosition;
qVel = cfg.guidance.padEkfProcessNoiseVelocity;
Q = diag([qPos, qPos, 0.5 * qPos, qVel, qVel, 0.5 * qVel].^2) * max(dt, eps);
state = F * state;
covariance = F * covariance * F' + Q;
end

function [state, covariance] = correct_pad_ekf(state, covariance, measurement, cfg)
H = [eye(3), zeros(3)];
r = cfg.guidance.padEkfMeasurementNoise;
R = diag([r, r, 0.5 * r].^2);
innovation = measurement(:) - H * state;
S = H * covariance * H' + R;
K = covariance * H' / S;
state = state + K * innovation;
covariance = (eye(6) - K * H) * covariance;
end

function value = lerp(a, b, t)
value = a + (b - a) * clamp01(t);
end

function value = clamp01(value)
value = max(0.0, min(1.0, value));
end

function step = enforce_marker_closure_step(step, rel, range, cfg)
if range <= cfg.guidance.finalLandRadius
    step = rel;
    return;
end
direction = rel ./ max(range, eps);
closingComponent = dot(step, direction);
minStep = min(cfg.guidance.ontologyMinApproachStep, range);
if closingComponent < minStep
    lateral = step - closingComponent * direction;
    step = lateral + minStep * direction;
end
maxStep = cfg.guidance.ontologyMaxHorizontalStep;
stepNorm = norm(step);
if stepNorm > maxStep
    step = step ./ max(stepNorm, eps) .* maxStep;
end
if dot(step, direction) <= 0.0
    step = minStep * direction;
end
end

function step = marker_closure_step(rel, range, maxStep, cfg)
if range <= eps
    step = [0.0, 0.0];
    return;
end
direction = rel ./ range;
desiredStep = min(maxStep, range);
step = desiredStep * direction;
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end

function prefix = clean_env_command_prefix()
keys = {'HOME', 'USER', 'LOGNAME', 'SHELL', 'LANG', 'DISPLAY', ...
    'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', ...
    'ROS_LOCALHOST_ONLY'};
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
parts{end+1} = 'RMW_IMPLEMENTATION="rmw_fastrtps_cpp"';
if isempty(getenv('ROS_LOCALHOST_ONLY'))
    parts{end+1} = 'ROS_LOCALHOST_ONLY="0"';
end
prefix = strjoin(parts, ' ');
end
