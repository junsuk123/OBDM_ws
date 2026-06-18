function [z, state] = compute_altitude_guidance(uavState, padState, semantic, ontologyGraph, selection, cfg, timestamp, state)
%COMPUTE_ALTITUDE_GUIDANCE Compute ontology-gated vertical landing command.

if nargin < 8 || isempty(state)
    state = struct();
end

uavZ = uavState.position(3);
uavVz = get_vertical_velocity(uavState);
padZ = padState.position(3);
touchdownZ = padState.touchdownAltitude;
altitudeAbovePad = uavZ - padZ;
dt = get_guidance_value(cfg, 'commandPeriod', cfg.simulation.dt);
descentAuthority = ontologyGraph.values.descentAuthority;
finalReadiness = ontologyGraph.values.finalLandReadiness;
xyError = ontologyGraph.values.currentXYError;
tuning = get_selected_tuning(ontologyGraph);
descentLimit = ontology_descent_limit(ontologyGraph, tuning, cfg);
algorithm = char(string(selection.altitudeAlgorithm));

terminalReady = is_terminal_ready(xyError, altitudeAbovePad, semantic, ontologyGraph, cfg);
if strcmp(algorithm, 'Minimum_Jerk_Terminal_Descent') && terminalReady
    [nominalVz, zRef, state, terminalActive] = minimum_jerk_command(uavZ, uavVz, touchdownZ, cfg, timestamp, state, descentLimit);
else
    if strcmp(algorithm, 'Minimum_Jerk_Terminal_Descent') && ~terminalReady
        algorithm = 'Ontology_Gated_PD_Descent';
    end
    state.terminalActive = false;
    state.terminalStartTime = nan;
    state.terminalStartAltitude = nan;
    terminalActive = false;
    [nominalVz, zRef] = ontology_gated_pd_command(uavZ, uavVz, touchdownZ, descentAuthority, cfg, descentLimit);
end

filterInput = struct( ...
    'nominalVz', nominalVz, ...
    'uavZ', uavZ, ...
    'uavVz', uavVz, ...
    'padZ', padZ, ...
    'touchdownZ', touchdownZ, ...
    'xyError', xyError, ...
    'descentAuthority', descentAuthority, ...
    'operationalRisk', ontologyGraph.values.operationalRisk, ...
    'maxDescentRate', descentLimit, ...
    'safetyActivation', get_profile_value(tuning, 'safetyActivation', 1.0));
if selection.applyAltitudeSafetyFilter
    safety = apply_altitude_safety_filter(filterInput, cfg);
else
    safety = struct('vZCmd', nominalVz, 'safetyLimited', false, 'filterReason', 'disabled', ...
        'hXY', inf, 'hVZ', inf);
end

vZCmd = safety.vZCmd;
targetZ = uavZ + vZCmd * max(dt, eps);
targetZ = max(touchdownZ, targetZ);
maxAscentRate = get_guidance_value(cfg, 'maxAscentRate', 0.70);
targetZ = min(targetZ, uavZ + maxAscentRate * max(dt, eps));

z = struct();
z.algorithm = algorithm;
z.targetZ = targetZ;
z.zReference = zRef;
z.nominalVz = nominalVz;
z.vZCmd = vZCmd;
z.descentAuthority = descentAuthority;
z.descentRateLimit = descentLimit;
z.finalLandReadiness = finalReadiness;
z.terminalActive = terminalActive;
z.safety = safety;
z.altitudeError = zRef - uavZ;
z.tuning = tuning;
end

function [vZ, zRef] = ontology_gated_pd_command(uavZ, uavVz, touchdownZ, descentAuthority, cfg, descentLimit)
kp = get_guidance_value(cfg, 'altitudePdKp', 0.75);
kd = get_guidance_value(cfg, 'altitudePdKd', 0.25);
maxDescent = descentLimit;
maxAscent = get_guidance_value(cfg, 'maxAscentRate', 0.70);
zRef = touchdownZ;
vPd = kp * (zRef - uavZ) - kd * uavVz;
vZ = min(max(vPd, -maxDescent * clamp01(descentAuthority)), maxAscent);
end

function [vZ, zRef, state, active] = minimum_jerk_command(uavZ, uavVz, touchdownZ, cfg, timestamp, state, descentLimit)
duration = get_guidance_value(cfg, 'terminalDescentDuration', 2.8);
if ~isfield(state, 'terminalActive') || ~state.terminalActive
    state.terminalActive = true;
    state.terminalStartTime = timestamp;
    state.terminalStartAltitude = uavZ;
end
elapsed = max(0.0, timestamp - state.terminalStartTime);
s = max(0.0, min(1.0, elapsed / max(duration, eps)));
q = 10.0 * s^3 - 15.0 * s^4 + 6.0 * s^5;
dqds = 30.0 * s^2 - 60.0 * s^3 + 30.0 * s^4;
z0 = state.terminalStartAltitude;
zf = touchdownZ;
zRef = z0 + (zf - z0) * q;
dzRef = (zf - z0) * dqds / max(duration, eps);
kp = get_guidance_value(cfg, 'minimumJerkKp', 0.95);
kd = get_guidance_value(cfg, 'minimumJerkKd', 0.35);
vZ = kp * (zRef - uavZ) + kd * (dzRef - uavVz);
vZ = max(vZ, -max(descentLimit, eps));
active = true;
end

function maxDescent = ontology_descent_limit(ontologyGraph, tuning, cfg)
baseLimit = get_guidance_value(cfg, 'maxDescentRate', cfg.guidance.ontologyMaxDescentRate);
if isfield(ontologyGraph.values, 'ontologyMaxDescentRate')
    baseLimit = min(baseLimit, ontologyGraph.values.ontologyMaxDescentRate);
elseif isfield(ontologyGraph.values, 'descentRate')
    baseLimit = min(baseLimit, ontologyGraph.values.descentRate);
end
controlGate = clamp(0.35 + 0.65 * get_profile_value(tuning, 'controlActivation', 1.0), 0.25, 1.15);
safetyGate = clamp(1.15 - 0.35 * get_profile_value(tuning, 'safetyActivation', 0.0), 0.45, 1.15);
riskGate = clamp(1.0 - 0.45 * ontologyGraph.values.operationalRisk, 0.35, 1.0);
maxDescent = max(0.05, baseLimit * controlGate * safetyGate * riskGate);
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

function tf = is_terminal_ready(xyError, altitudeAbovePad, semantic, ontologyGraph, cfg)
tf = xyError <= get_guidance_value(cfg, 'terminalXYThreshold', cfg.guidance.finalLandRadius) && ...
    semantic.visualStability >= get_guidance_value(cfg, 'terminalVisionThreshold', 0.70) && ...
    semantic.windRisk <= get_guidance_value(cfg, 'terminalWindRiskThreshold', 0.45) && ...
    altitudeAbovePad <= get_guidance_value(cfg, 'terminalAltitudeThreshold', 1.20) && ...
    ontologyGraph.values.finalLandReadiness >= get_guidance_value(cfg, 'terminalReadinessThreshold', 0.68);
end

function vz = get_vertical_velocity(uavState)
vz = 0.0;
if isfield(uavState, 'velocity') && numel(uavState.velocity) >= 3
    vz = uavState.velocity(3);
end
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end

function value = clamp01(value)
value = max(0.0, min(1.0, value));
end

function value = clamp(value, lo, hi)
value = max(lo, min(hi, value));
end
