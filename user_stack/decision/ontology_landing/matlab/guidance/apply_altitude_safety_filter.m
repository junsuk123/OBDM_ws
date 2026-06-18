function safety = apply_altitude_safety_filter(input, cfg)
%APPLY_ALTITUDE_SAFETY_FILTER Deterministic CBF-style vertical safety clamp.

maxDescent = get_input_value(input, 'maxDescentRate', get_guidance_value(cfg, 'maxDescentRate', cfg.guidance.ontologyMaxDescentRate));
maxAscent = get_guidance_value(cfg, 'maxAscentRate', 0.70);
touchdownAltitude = input.touchdownZ;
safetyActivation = get_input_value(input, 'safetyActivation', 1.0);
xyAllowed = allowed_xy_error(input.uavZ - input.padZ, cfg, safetyActivation);
hXY = xyAllowed - input.xyError;
hVZ = maxDescent - abs(input.uavVz);
riskGate = max(0.0, min(1.0, 1.0 - input.operationalRisk));
authority = max(0.0, min(1.0, input.descentAuthority));

vZ = min(max(input.nominalVz, -maxDescent * authority), maxAscent);
reason = "nominal";
limited = abs(vZ - input.nominalVz) > 1.0e-6;

if hXY < 0.0
    vZ = max(vZ, 0.0);
    limited = true;
    reason = "xy_barrier";
elseif riskGate < get_guidance_value(cfg, 'altitudeRiskGateMin', 0.35)
    vZ = max(vZ, -0.25 * maxDescent * authority);
    limited = true;
    reason = "risk_barrier";
elseif input.uavZ <= touchdownAltitude + get_guidance_value(cfg, 'touchdownHeightAbovePad', 0.15) && ...
        input.xyError > cfg.labeling.touchdownPositionError
    vZ = max(vZ, 0.0);
    limited = true;
    reason = "touchdown_barrier";
end

if hVZ < 0.0 && vZ < 0.0
    vZ = max(vZ, -0.35 * maxDescent * authority);
    limited = true;
    reason = "vertical_speed_barrier";
end

safety = struct();
safety.vZCmd = vZ;
safety.safetyLimited = limited;
safety.filterReason = char(reason);
safety.hXY = hXY;
safety.hVZ = hVZ;
safety.xyAllowed = xyAllowed;
safety.descentAuthority = authority;
end

function allowed = allowed_xy_error(altitudeAbovePad, cfg, safetyActivation)
near = cfg.labeling.touchdownPositionError;
far = get_guidance_value(cfg, 'altitudeCbfFarXYError', cfg.guidance.ontologyDescentStartRadius);
refAlt = get_guidance_value(cfg, 'altitudeCbfReferenceAltitude', cfg.ontology.altitudeApproachReference);
t = max(0.0, min(1.0, altitudeAbovePad / max(refAlt, eps)));
allowed = near + (far - near) * t;
allowed = allowed * max(0.65, 1.0 - 0.20 * max(0.0, min(1.0, safetyActivation)));
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end

function value = get_input_value(input, fieldName, defaultValue)
value = defaultValue;
if isstruct(input) && isfield(input, fieldName) && ~isempty(input.(fieldName))
    value = input.(fieldName);
end
end
