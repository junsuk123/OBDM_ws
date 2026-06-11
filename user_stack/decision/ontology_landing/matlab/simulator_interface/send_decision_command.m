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
    log_trajectory_debug('skip_command_period', sim, uavState, observation, semantic, [], [], timestamp);
    return;
end

trajectory = generate_ontology_trajectory(sim, classProbability, confidence, cfg, uavState, observation, semantic, timestamp);
if isfield(trajectory, 'hasMarker') && trajectory.hasMarker
    [sim, trajectory] = update_direct_land_request(sim, trajectory, uavState, timestamp, cfg);
end
sim.guidance.mode = trajectory.mode;
sim.guidance.trajectory = trajectory;

if isfield(cfg, 'visualization') && isfield(cfg.visualization, 'showOntologyGraph') && cfg.visualization.showOntologyGraph
    if isfield(trajectory, 'ontologyGraph') && isstruct(trajectory.ontologyGraph) && ...
            isfield(trajectory.ontologyGraph, 'nodes') && isfield(trajectory.ontologyGraph, 'edges')
        plotOntologyGraphRealtime(trajectory.ontologyGraph, cfg);
    end
end

runtimeExportActive = isfield(cfg, 'visualization') && isfield(cfg.visualization, 'web3dRuntimeExport') && ...
    cfg.visualization.web3dRuntimeExport;
if ~runtimeExportActive && isfield(trajectory, 'ontologyGraph') && isstruct(trajectory.ontologyGraph) && ...
        isfield(trajectory.ontologyGraph, 'nodes') && isfield(trajectory.ontologyGraph, 'edges')
    try
        export_ontology_graph_snapshot(trajectory.ontologyGraph, cfg, trajectory, action, classProbability, confidence);
    catch exportError
        warning('ontology_viewer:ExportFailed', ...
            '[ontology_viewer] Failed to export web graph snapshot: %s', exportError.message);
    end
end

if strcmpi(trajectory.command, 'land')
    log_trajectory_debug('land_request', sim, uavState, observation, semantic, trajectory, [0.0, 0.0, 0.0], timestamp);
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
    log_trajectory_debug('local_wp', sim, uavState, observation, semantic, trajectory, localTarget, timestamp);
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
semantic.relativeDistanceState = 0.5;
semantic.altitudeState = 0.5;
semantic.droneMotionStability = 0.5;
semantic.roverMotionStability = 0.5;
semantic.altitudeAbovePad = 0.0;
semantic.roverSpeed = 0.0;
semantic.pathLengthEstimate = 0.0;
semantic.timeToTargetEstimate = 0.0;
semantic.energyCostEstimate = 0.0;
semantic.dragForce = 0.0;
semantic.dragPower = 0.0;
semantic.relativeAirSpeed = 0.0;
semantic.vector = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5];
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
touchdownAltitude = pad_touchdown_altitude(markerPosition, cfg);
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

ontologyGraph = evaluate_landing_ontology_graph(markerPosition, markerVelocity, uavState, semantic, ...
    classProbability, confidence, range, closingSpeed, cfg);
intent = ontologyGraph.values.landingIntent;
risk = ontologyGraph.values.operationalRisk;
trackQuality = ontologyGraph.values.trackQuality;
efficiency = ontologyGraph.values.efficiency;
leadTime = ontologyGraph.values.leadTime;
futureTime = ontologyGraph.values.futurePredictionTime;
futurePoint = ontologyGraph.values.futureMarkerPoint;
navGain = ontologyGraph.values.navigationGain;
maxStep = ontologyGraph.values.maxHorizontalStep;
descentRate = ontologyGraph.values.descentRate;
descentStartRadius = ontologyGraph.values.descentStartRadius;

pnStep = navGain * (ontologyGraph.values.approachAimPoint - uavState.position(1:2));
stepNorm = norm(pnStep);
if stepNorm > maxStep
    pnStep = pnStep / stepNorm * maxStep;
end
directStep = marker_closure_step(rel, range, maxStep, cfg);
pnStep = (1.0 - cfg.guidance.ontologyDistancePriority) * pnStep + ...
    cfg.guidance.ontologyDistancePriority * directStep;
pnStep = enforce_marker_closure_step(pnStep, rel, range, cfg);
pnStep = pnStep * get_guidance_value(cfg, 'horizontalResponseScale', 1.0);
pnStep = clamp_step_norm(pnStep, min(maxStep, cfg.guidance.ontologyMaxHorizontalStep));
pnStep = precision_landing_step(pnStep, rel, range, uavVelocity, markerVelocity, cfg);

targetXY = uavState.position(1:2) + pnStep;

descentReferenceAltitude = uavAltitude;
if isfield(sim.guidance, 'lastTarget') && numel(sim.guidance.lastTarget) >= 3 && ...
        ~isnan(sim.guidance.lastTarget(3))
    descentReferenceAltitude = min(uavAltitude, sim.guidance.lastTarget(3));
end
descentDt = guidance_descent_dt(sim, timestamp, cfg);
horizontalSpeed = norm(uavVelocity(1:2));
relativeSpeed = norm(uavVelocity(1:2) - markerVelocity(1:2));
[descentRate, targetHoldAltitude, precisionReady] = precision_descent_gate( ...
    descentRate, touchdownAltitude, markerPosition, uavAltitude, range, horizontalSpeed, relativeSpeed, cfg);
if range <= descentStartRadius
    targetAltitude = max(touchdownAltitude, ...
        descentReferenceAltitude - descentRate * descentDt);
else
    farDescentRate = descentRate * cfg.guidance.ontologyFarDescentRateScale;
    targetAltitude = max(touchdownAltitude, ...
        descentReferenceAltitude - farDescentRate * descentDt);
end
targetAltitude = max(targetAltitude, targetHoldAltitude);

trajectory.mode = 'ONTOLOGY_ARUCO_TRAJECTORY';
trajectory.target = [targetXY, targetAltitude];
trajectory.markerWorldPosition = markerPosition;
trajectory.landingPadCenterWorld = markerPosition;
trajectory.markerWorldVelocity = markerVelocity;
trajectory.touchdownAltitude = touchdownAltitude;
trajectory.descentStartRadius = descentStartRadius;
trajectory.landingIntent = intent;
trajectory.leadTime = leadTime;
trajectory.futurePredictionTime = futureTime;
trajectory.futureMarkerPoint = futurePoint;
trajectory.ontologyGraph = ontologyGraph;
trajectory.navigationGain = navGain;
trajectory.maxHorizontalStep = maxStep;
trajectory.approachStep = norm(pnStep);
trajectory.distanceToMarker = range;
trajectory.descentRate = descentRate;
trajectory.trackQuality = trackQuality;
trajectory.precisionReady = precisionReady;
trajectory.horizontalSpeed = horizontalSpeed;
trajectory.relativeSpeed = relativeSpeed;
trajectory.efficiency = efficiency;
trajectory.risk = risk;
trajectory.pathLengthEstimate = norm([trajectory.target(1:2) - uavState.position(1:2), ...
    max(uavAltitude - trajectory.target(3), 0.0)]);
trajectory.timeToTargetEstimate = trajectory.pathLengthEstimate / max(norm(uavVelocity(1:2)), 0.25);
trajectory.dragForce = semantic.dragForce;
trajectory.dragPower = semantic.dragPower;
trajectory.energyCostEstimate = semantic.energyCostEstimate;
trajectory.finalLand = range <= cfg.guidance.finalLandRadius && ...
    uavAltitude <= max(cfg.guidance.finalLandAltitude, touchdownAltitude + 0.2) && ...
    precisionReady && ...
    semantic.alignmentState >= cfg.guidance.ontologyFinalLandMinAlignment && ...
    semantic.visualStability >= cfg.guidance.ontologyFinalLandMinVisualStability && ...
    semantic.windRisk <= cfg.guidance.ontologyFinalLandMaxWindRisk && ...
    semantic.energyEfficiency >= 0.25;
if trajectory.finalLand
    trajectory.command = 'land';
end
end

function [sim, trajectory] = update_direct_land_request(sim, trajectory, uavState, timestamp, cfg)
directRadius = get_guidance_value(cfg, 'directLandRadius', cfg.guidance.finalLandRadius);
directAltitudeAbovePad = get_guidance_value(cfg, 'directLandAltitudeAbovePad', 1.20);
directStableTime = get_guidance_value(cfg, 'directLandStableTime', 0.70);
maxHorizontalSpeed = get_guidance_value(cfg, 'directLandMaxHorizontalSpeed', 1.20);
maxRelativeSpeed = get_guidance_value(cfg, 'directLandMaxRelativeSpeed', maxHorizontalSpeed);
requireRawTag = logical(get_guidance_value(cfg, 'directLandRequireRawTag', true));
maxEkfAge = get_guidance_value(cfg, 'directLandMaxEkfAge', 0.60);
minTrackQuality = get_guidance_value(cfg, 'directLandMinTrackQuality', 0.65);

uavAltitude = uavState.position(3);
if isfield(uavState, 'altitude')
    uavAltitude = uavState.altitude;
end
horizontalSpeed = 0.0;
if isfield(uavState, 'velocity') && numel(uavState.velocity) >= 2
    horizontalSpeed = norm(uavState.velocity(1:2));
end
relativeSpeed = horizontalSpeed;
if isfield(trajectory, 'markerWorldVelocity') && numel(trajectory.markerWorldVelocity) >= 2 && ...
        isfield(uavState, 'velocity') && numel(uavState.velocity) >= 2
    relativeSpeed = norm(uavState.velocity(1:2) - trajectory.markerWorldVelocity(1:2));
end

padAltitude = trajectory.touchdownAltitude - get_guidance_value(cfg, 'touchdownHeightAbovePad', 0.0);
altitudeAbovePad = uavAltitude - padAltitude;
aligned = trajectory.distanceToMarker <= directRadius;
lowEnough = altitudeAbovePad <= directAltitudeAbovePad;
slowEnough = horizontalSpeed <= maxHorizontalSpeed;
relativeSlowEnough = relativeSpeed <= maxRelativeSpeed;
rawTagDetected = isfield(trajectory, 'rawTagDetected') && logical(trajectory.rawTagDetected);
padEkfAge = inf;
if isfield(trajectory, 'padEkfAge') && isnumeric(trajectory.padEkfAge) && isscalar(trajectory.padEkfAge)
    padEkfAge = trajectory.padEkfAge;
end
trackQuality = 0.0;
if isfield(trajectory, 'trackQuality') && isnumeric(trajectory.trackQuality) && isscalar(trajectory.trackQuality)
    trackQuality = trajectory.trackQuality;
end
freshMarker = isfinite(padEkfAge) && padEkfAge <= maxEkfAge && (~requireRawTag || rawTagDetected);
trackReady = isfinite(trackQuality) && trackQuality >= minTrackQuality;

if aligned && lowEnough && slowEnough && relativeSlowEnough && freshMarker && trackReady
    if ~isfield(sim.guidance, 'directLandSince') || isempty(sim.guidance.directLandSince) || ...
            ~isfinite(sim.guidance.directLandSince)
        sim.guidance.directLandSince = timestamp;
    end
    stableTime = max(0.0, timestamp - sim.guidance.directLandSince);
else
    sim.guidance.directLandSince = nan;
    stableTime = 0.0;
end

trajectory.directLand = struct( ...
    'aligned', aligned, ...
    'lowEnough', lowEnough, ...
    'slowEnough', slowEnough, ...
    'relativeSlowEnough', relativeSlowEnough, ...
    'stableTime', stableTime, ...
    'requiredStableTime', directStableTime, ...
    'altitudeAbovePad', altitudeAbovePad, ...
    'horizontalSpeed', horizontalSpeed, ...
    'relativeSpeed', relativeSpeed, ...
    'freshMarker', freshMarker, ...
    'trackReady', trackReady, ...
    'rawTagDetected', rawTagDetected, ...
    'padEkfAge', padEkfAge, ...
    'maxEkfAge', maxEkfAge, ...
    'trackQuality', trackQuality, ...
    'minTrackQuality', minTrackQuality);

if stableTime >= directStableTime
    trajectory.finalLand = true;
    trajectory.command = 'land';
    trajectory.mode = 'DIRECT_STABLE_LAND';
end
end

function altitude = pad_touchdown_altitude(markerPosition, cfg)
heightAbovePad = get_guidance_value(cfg, 'touchdownHeightAbovePad', 0.0);
altitude = max(cfg.labeling.touchdownAltitude, markerPosition(3) + heightAbovePad);
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
risk = clamp01(0.25 * semantic.windRisk + 0.20 * semantic.dragRisk + ...
    0.18 * (1.0 - semantic.energyEfficiency) + 0.14 * (1.0 - semantic.droneMotionStability) + ...
    0.13 * (1.0 - semantic.roverMotionStability) + 0.10 * (1.0 - semantic.visualStability));
targetAltitude = uavState.position(3);
if isfield(sim, 'guidance') && isfield(sim.guidance, 'lastTarget') && ~any(isnan(sim.guidance.lastTarget))
    holdTarget = sim.guidance.lastTarget;
end
searchAltitude = get_guidance_value(cfg, 'searchAltitude', cfg.simulator.takeoffAltitude);
maxSearchAltitude = get_guidance_value(cfg, 'maxSearchAltitude', cfg.simulator.takeoffAltitude);
searchAltitude = min(searchAltitude, maxSearchAltitude);
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
trajectory.trackQuality = clamp01(0.30 * semantic.alignmentState + 0.25 * semantic.relativeDistanceState + ...
    0.25 * semantic.visualStability + 0.20 * semantic.droneMotionStability);
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

function step = precision_landing_step(step, rel, range, uavVelocity, markerVelocity, cfg)
precisionRadius = get_guidance_value(cfg, 'precisionApproachRadius', cfg.guidance.ontologyDescentStartRadius);
finalRadius = get_guidance_value(cfg, 'precisionFinalAlignmentRadius', cfg.guidance.finalLandRadius);
if range > precisionRadius || range <= eps
    return;
end

precisionBlend = clamp01(1.0 - (range - finalRadius) / max(precisionRadius - finalRadius, eps));
direction = rel ./ max(range, eps);
closingComponent = dot(step, direction);
lateral = step - closingComponent * direction;

maxStep = lerp( ...
    get_guidance_value(cfg, 'precisionMaxHorizontalStep', 0.85), ...
    get_guidance_value(cfg, 'precisionFinalMaxHorizontalStep', 0.22), ...
    precisionBlend);
minClosing = min( ...
    get_guidance_value(cfg, 'precisionMinApproachStep', 0.03), ...
    range);

relativeVelocity = uavVelocity(1:2) - markerVelocity(1:2);
brakeStep = -relativeVelocity * get_guidance_value(cfg, 'commandBrakeTime', 0.35);
brakeClosing = dot(brakeStep, direction);
if brakeClosing > 0.0
    closingComponent = min(closingComponent, brakeClosing);
end
closingComponent = max(minClosing, min(closingComponent, range));
lateral = lateral * (1.0 - 0.75 * precisionBlend);
step = closingComponent * direction + lateral;
step = clamp_step_norm(step, maxStep);
if dot(step, direction) <= 0.0
    step = min(range, maxStep) * direction;
end
end

function [descentRate, holdAltitude, precisionReady] = precision_descent_gate(descentRate, touchdownAltitude, markerPosition, uavAltitude, range, horizontalSpeed, relativeSpeed, cfg)
precisionRadius = get_guidance_value(cfg, 'precisionApproachRadius', cfg.guidance.ontologyDescentStartRadius);
alignmentRadius = get_guidance_value(cfg, 'precisionAlignmentRadius', cfg.guidance.alignRadius);
maxHorizontalSpeed = get_guidance_value(cfg, 'precisionMaxHorizontalSpeedForDescent', 0.70);
maxRelativeSpeed = get_guidance_value(cfg, 'precisionMaxRelativeSpeedForDescent', 0.45);
holdHeightAbovePad = get_guidance_value(cfg, 'precisionDescentHoldHeightAbovePad', 0.70);

aligned = range <= alignmentRadius;
slow = horizontalSpeed <= maxHorizontalSpeed && relativeSpeed <= maxRelativeSpeed;
precisionReady = aligned && slow;
holdAltitude = touchdownAltitude;

if range <= precisionRadius
    precisionBlend = clamp01(1.0 - range / max(precisionRadius, eps));
    descentRate = min(descentRate, lerp( ...
        get_guidance_value(cfg, 'precisionMaxDescentRate', 0.35), ...
        get_guidance_value(cfg, 'precisionFinalDescentRate', 0.18), ...
        precisionBlend));
    if ~precisionReady
        descentRate = descentRate * 0.25;
        holdAltitude = max(touchdownAltitude, markerPosition(3) + holdHeightAbovePad);
    end
end

if uavAltitude <= holdAltitude + 0.05 && ~precisionReady
    descentRate = 0.0;
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

function step = clamp_step_norm(step, maxNorm)
if ~isfinite(maxNorm) || maxNorm <= 0.0
    return;
end
stepNorm = norm(step);
if stepNorm > maxNorm
    step = step ./ max(stepNorm, eps) .* maxNorm;
end
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end

function log_trajectory_debug(stage, sim, uavState, observation, semantic, trajectory, localTarget, timestamp)
logPath = '/tmp/obdm_matlab_trajectory_debug.log';
fid = fopen(logPath, 'a');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>

uavPosition = get_vector_field(uavState, 'position', [nan, nan, nan]);
uavVelocity = get_vector_field(uavState, 'velocity', [nan, nan, nan]);
markerPosition = get_vector_field(observation, 'markerPosition', [nan, nan, nan]);
markerVelocity = get_vector_field(observation, 'markerVelocity', [nan, nan, nan]);
target = get_vector_field(trajectory, 'target', [nan, nan, nan]);
if isempty(localTarget)
    localTarget = [nan, nan, nan];
end

hasMarker = get_bool_field(trajectory, 'hasMarker', false);
tagDetected = get_bool_field(observation, 'tagDetected', false);
isMockUav = get_bool_field(uavState, 'isMock', true);
isMockObs = get_bool_field(observation, 'isMock', true);
ekfInitialized = false;
ekfAge = nan;
ekfPosition = [nan, nan, nan];
if isfield(sim, 'guidance') && isfield(sim.guidance, 'padEkf')
    ekfInitialized = get_bool_field(sim.guidance.padEkf, 'initialized', false);
    if isfield(sim.guidance.padEkf, 'state') && numel(sim.guidance.padEkf.state) >= 3
        ekfPosition = sim.guidance.padEkf.state(1:3)';
    end
    if isfield(sim.guidance.padEkf, 'lastMeasurementTimestamp')
        ekfAge = max(0.0, timestamp - sim.guidance.padEkf.lastMeasurementTimestamp);
    end
end

mode = get_char_field(trajectory, 'mode', 'none');
command = get_char_field(trajectory, 'command', 'none');
source = get_char_field(observation, 'source', 'none');
range = get_numeric_field(trajectory, 'distanceToMarker', nan);
approachStep = get_numeric_field(trajectory, 'approachStep', nan);
maxStep = get_numeric_field(trajectory, 'maxHorizontalStep', nan);
navGain = get_numeric_field(trajectory, 'navigationGain', nan);
alignment = get_numeric_field(semantic, 'alignmentState', nan);
visual = get_numeric_field(semantic, 'visualStability', nan);
windRisk = get_numeric_field(semantic, 'windRisk', nan);

fprintf(fid, ['t=%.3f stage=%s mode=%s command=%s hasMarker=%d ekfInit=%d ekfAge=%.3f ' ...
    'tag=%d source=%s uavMock=%d obsMock=%d ' ...
    'uav=%s uavVel=%s marker=%s markerVel=%s ekf=%s target=%s local=%s ' ...
    'range=%.3f approachStep=%.3f maxStep=%.3f navGain=%.3f ' ...
    'alignment=%.3f visual=%.3f windRisk=%.3f\n'], ...
    timestamp, stage, mode, command, hasMarker, ekfInitialized, ekfAge, ...
    tagDetected, source, isMockUav, isMockObs, ...
    format_vector(uavPosition), format_vector(uavVelocity), ...
    format_vector(markerPosition), format_vector(markerVelocity), ...
    format_vector(ekfPosition), format_vector(target), format_vector(localTarget), ...
    range, approachStep, maxStep, navGain, alignment, visual, windRisk);
end

function value = get_vector_field(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && numel(item.(fieldName)) >= numel(defaultValue)
    candidate = item.(fieldName);
    value = candidate(1:numel(defaultValue));
end
end

function value = get_bool_field(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && ~isempty(item.(fieldName))
    value = logical(item.(fieldName));
end
end

function value = get_numeric_field(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && ~isempty(item.(fieldName))
    value = double(item.(fieldName));
end
end

function value = get_char_field(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && ~isempty(item.(fieldName))
    value = char(item.(fieldName));
end
end

function text = format_vector(value)
value = double(value(:)');
text = sprintf('[%.3f,%.3f,%.3f]', value(1), value(2), value(3));
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
