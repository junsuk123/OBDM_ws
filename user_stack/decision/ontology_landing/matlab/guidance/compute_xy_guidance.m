function xy = compute_xy_guidance(uavState, padState, observation, semantic, ontologyGraph, selection, cfg, dt)
%COMPUTE_XY_GUIDANCE Compute modular horizontal landing guidance.

if nargin < 8 || isempty(dt)
    dt = get_guidance_value(cfg, 'commandPeriod', cfg.simulation.dt);
end

uavPos = uavState.position(1:2);
uavVel = get_state_velocity(uavState, 2);
padPos = padState.position(1:2);
padVel = padState.velocity(1:2);
eXY = padPos - uavPos;
vRel = padVel - uavVel;
range = norm(eXY);
tuning = get_selected_tuning(ontologyGraph);
speedScale = get_profile_value(tuning, 'speedScale', 1.0);
accelScale = get_profile_value(tuning, 'accelScale', 1.0);
controlActivation = get_profile_value(tuning, 'controlActivation', 1.0);
riskGate = clamp01(1.0 - ontologyGraph.values.operationalRisk);
trackGate = clamp01(0.55 + 0.45 * ontologyGraph.values.trackQuality);
authorityGate = clamp(0.35 + 0.65 * controlActivation, 0.20, 1.20);
maxStep = max(0.0, ontologyGraph.values.maxHorizontalStep) * clamp(speedScale * authorityGate, 0.15, 1.35);
maxSpeed = get_guidance_value(cfg, 'xyMaxSpeed', maxStep / max(dt, eps)) * ...
    clamp(speedScale * trackGate * authorityGate, 0.15, 1.50);
algorithm = char(string(selection.xyAlgorithm));

switch algorithm
    case 'MPC_NMPC_Pad_Prediction_Tracking'
        xy = mpc_prediction_guidance(eXY, vRel, padPos, padVel, uavPos, ontologyGraph, cfg, dt, tuning);
    case 'Receding_Horizon_Trajectory_Optimization_Robust_Guidance'
        xy = robust_receding_horizon_guidance(eXY, vRel, padVel, semantic, ontologyGraph, cfg, tuning);
    otherwise
        algorithm = 'Vision_FOV_Constrained_Guidance';
        xy = vision_fov_guidance(eXY, vRel, padVel, observation, semantic, ontologyGraph, cfg, tuning);
end

xy.algorithm = algorithm;
xy.tuning = tuning;
xy.error = eXY;
xy.errorNorm = range;
xy.relativeVelocity = vRel;
xy.velocity = limit_vector(xy.velocity, maxSpeed);
xy.step = limit_vector(xy.velocity * max(dt, eps), maxStep);
accelMax = get_guidance_value(cfg, 'xyMaxAccel', inf) * clamp(accelScale * riskGate, 0.20, 1.25);
if isfinite(accelMax) && accelMax > 0.0
    maxAccelStep = accelMax * max(dt, eps)^2;
    if norm(xy.step) > max(maxAccelStep, eps) && range < get_guidance_value(cfg, 'xyAccelLimitRadius', 1.2)
        xy.step = limit_vector(xy.step, maxAccelStep);
        xy.velocity = xy.step / max(dt, eps);
        xy.accelLimited = true;
    else
        xy.accelLimited = false;
    end
else
    xy.accelLimited = false;
end
xy.guidanceWaypoint = uavPos + xy.step;
xy.targetXY = xy.guidanceWaypoint;
if get_guidance_value(cfg, 'commandPadCenter', false)
    xy.targetXY = padPos;
end
xy.maxHorizontalStep = maxStep;
xy.approachStep = norm(xy.step);
xy.speedScale = speedScale;
xy.accelScale = accelScale;
xy.controlActivation = controlActivation;
end

function xy = vision_fov_guidance(eXY, vRel, padVel, observation, semantic, ontologyGraph, cfg, tuning)
tagError = get_observation_value(observation, 'tagCenterError', norm(eXY));
tagRef = get_guidance_value(cfg, 'visionTagErrorReference', cfg.ontology.positionErrorReference);
tagErrorNorm = clamp01(tagError / max(tagRef, eps));
visionGate = clamp01(semantic.visualStability * (1.0 - tagErrorNorm));
gain = ontologyGraph.values.navigationGain * clamp(get_profile_value(tuning, 'navigationGain', 1.0), 0.35, 1.65);
kp = get_guidance_value(cfg, 'visionKpXY', 0.85) * gain;
kd = get_guidance_value(cfg, 'visionKdXY', 0.35);
kff = get_guidance_value(cfg, 'visionPadVelocityFeedforward', 0.55);
xy = struct();
xy.velocity = visionGate * (kp * eXY + kd * vRel + kff * padVel);
xy.visionGate = visionGate;
xy.tagCenterErrorNormalized = tagErrorNorm;
xy.predictedPadPoint = [nan, nan];
xy.robustTerm = [0.0, 0.0];
xy.riskGate = 1.0 - ontologyGraph.values.operationalRisk;
end

function xy = mpc_prediction_guidance(eXY, vRel, padPos, padVel, uavPos, ontologyGraph, cfg, dt, tuning)
horizon = get_guidance_value(cfg, 'mpcPredictionHorizon', ontologyGraph.values.futurePredictionTime) * ...
    clamp(get_profile_value(tuning, 'horizonScale', 1.0), 0.35, 1.80);
horizon = max(dt, min(horizon, get_guidance_value(cfg, 'ontologyPredictionHorizon', horizon)));
predictedPad = padPos + padVel * horizon;
predictionError = predictedPad - uavPos;
qualityGate = clamp01(0.55 * ontologyGraph.values.trackQuality + 0.45 * (1.0 - ontologyGraph.values.operationalRisk));
kp = get_guidance_value(cfg, 'mpcKpXY', 1.05) * qualityGate * ...
    clamp(get_profile_value(tuning, 'controlActivation', 1.0), 0.35, 1.35);
kd = get_guidance_value(cfg, 'mpcKdXY', 0.45);
xy = struct();
xy.velocity = kp * predictionError + kd * vRel;
xy.visionGate = qualityGate;
xy.tagCenterErrorNormalized = nan;
xy.predictedPadPoint = predictedPad;
xy.robustTerm = [0.0, 0.0];
xy.riskGate = 1.0 - ontologyGraph.values.operationalRisk;
xy.cost = norm(eXY)^2 + 0.25 * norm(vRel)^2;
end

function xy = robust_receding_horizon_guidance(eXY, vRel, padVel, semantic, ontologyGraph, cfg, tuning)
lambda = get_guidance_value(cfg, 'robustLambdaXY', 0.85);
phi = get_guidance_value(cfg, 'robustBoundaryLayerXY', 0.65);
riskGate = clamp01(1.0 - ontologyGraph.values.operationalRisk);
s = vRel + lambda * eXY;
robustTerm = sat_vector(s / max(phi, eps));
kp = get_guidance_value(cfg, 'robustKpXY', 0.90) * ontologyGraph.values.navigationGain * ...
    clamp(get_profile_value(tuning, 'controlActivation', 1.0), 0.35, 1.35);
kd = get_guidance_value(cfg, 'robustKdXY', 0.40);
krobust = get_guidance_value(cfg, 'robustCorrectionGainXY', 0.55) * ...
    clamp(get_profile_value(tuning, 'windGain', 1.0), 0.25, 1.80);
windScale = clamp01(0.4 + 0.6 * max(semantic.windRisk, semantic.dragRisk));
xy = struct();
xy.velocity = kp * eXY + kd * vRel + krobust * windScale * riskGate * robustTerm + 0.25 * padVel;
xy.visionGate = ontologyGraph.values.trackQuality;
xy.tagCenterErrorNormalized = nan;
xy.predictedPadPoint = [nan, nan];
xy.robustTerm = robustTerm;
xy.riskGate = riskGate;
xy.slidingVariable = s;
end

function tuning = get_selected_tuning(ontologyGraph)
tuning = struct();
if isstruct(ontologyGraph) && isfield(ontologyGraph, 'values') && ...
        isfield(ontologyGraph.values, 'selectedGuidanceParameters') && ...
        isstruct(ontologyGraph.values.selectedGuidanceParameters)
    tuning = ontologyGraph.values.selectedGuidanceParameters;
end
end

function value = get_profile_value(profile, fieldName, defaultValue)
value = defaultValue;
if isstruct(profile) && isfield(profile, fieldName) && ~isempty(profile.(fieldName))
    value = profile.(fieldName);
end
end

function velocity = get_state_velocity(state, count)
velocity = zeros(1, count);
if isstruct(state) && isfield(state, 'velocity') && numel(state.velocity) >= count
    velocity = state.velocity(1:count);
end
end

function value = get_observation_value(observation, fieldName, defaultValue)
value = defaultValue;
if isstruct(observation) && isfield(observation, fieldName) && ~isempty(observation.(fieldName))
    value = observation.(fieldName);
end
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end

function v = limit_vector(v, maxNorm)
if ~isfinite(maxNorm) || maxNorm <= 0.0
    return;
end
n = norm(v);
if n > maxNorm
    v = v ./ max(n, eps) .* maxNorm;
end
end

function y = sat_vector(x)
y = max(-1.0, min(1.0, x));
end

function value = clamp01(value)
value = max(0.0, min(1.0, value));
end

function value = clamp(value, lo, hi)
value = max(lo, min(hi, value));
end
