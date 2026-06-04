function semantic = compute_ontology_features(uavState, windState, observation, cfg)
%COMPUTE_ONTOLOGY_FEATURES Convert raw data into normalized semantic features.

positionError = observation.positionError;
horizontalSpeed = observation.horizontalSpeed;
verticalSpeed = abs(observation.verticalSpeed);
uavVelocity = uavState.velocity;
markerPosition = get_vector_field(observation, 'markerPosition', [nan, nan, nan]);
markerVelocity = get_vector_field(observation, 'markerVelocity', [0.0, 0.0, 0.0]);
if numel(markerPosition) >= 3 && all(isfinite(markerPosition(1:3)))
    altitudeAbovePad = uavState.altitude - markerPosition(3);
else
    altitudeAbovePad = uavState.altitude;
end
if ~isfield(windState, 'vector') || isempty(windState.vector)
    windVector = [0.0, 0.0, 0.0];
else
    windVector = windState.vector;
end
if numel(windVector) < 3
    windVector = [windVector(:)', zeros(1, 3 - numel(windVector))];
end

windSpeedNorm = normalize_feature(windState.speed, 0.0, cfg.ontology.windSpeedReference);
windAccelerationNorm = normalize_feature(abs(windState.acceleration), 0.0, cfg.ontology.windAccelerationReference);
windDirectionChangeNorm = normalize_feature(abs(windState.directionChange), 0.0, cfg.ontology.windDirectionChangeReference);
relativeAirVelocity = uavVelocity(:)' - windVector(1:3);
relativeAirSpeed = norm(relativeAirVelocity);
dragForce = 0.5 * cfg.ontology.airDensity * cfg.ontology.dragCoefficient * ...
    cfg.ontology.referenceArea * relativeAirSpeed^2;
dragPower = dragForce * max(horizontalSpeed, abs(uavState.velocity(3)));
dragDisturbance = horizontalSpeed * windState.speed / max(cfg.ontology.dragReference, eps);
dragNorm = normalize_feature(dragDisturbance, 0.0, 1.0);
dragForceNorm = normalize_feature(dragForce, 0.0, cfg.ontology.dragForceReference);
dragPowerNorm = normalize_feature(dragPower, 0.0, cfg.ontology.dragPowerReference);
dragRisk = max(0.0, min(1.0, 0.35 * dragNorm + 0.35 * dragForceNorm + 0.30 * dragPowerNorm));

windRisk = 0.30 * windSpeedNorm + 0.20 * windAccelerationNorm + ...
    0.15 * windDirectionChangeNorm + 0.35 * dragRisk;
windRisk = max(0.0, min(1.0, windRisk));

visualStability = observation.visualStability;

yawError = abs(wrap_to_pi(uavState.attitude(3) - deg2rad(cfg.ontology.referenceYawDeg)));
positionAlignment = 1.0 - normalize_feature(positionError, 0.0, cfg.ontology.positionErrorReference);
yawAlignment = 1.0 - normalize_feature(yawError, 0.0, deg2rad(cfg.ontology.yawErrorReferenceDeg));
alignmentState = max(0.0, min(1.0, 0.6 * positionAlignment + 0.4 * yawAlignment));
relativeDistanceState = max(0.0, min(1.0, positionAlignment));

rollStability = 1.0 - normalize_feature(abs(uavState.attitude(1)), 0.0, deg2rad(cfg.ontology.rollReferenceDeg));
pitchStability = 1.0 - normalize_feature(abs(uavState.attitude(2)), 0.0, deg2rad(cfg.ontology.pitchReferenceDeg));
attitudeStability = max(0.0, min(1.0, 0.5 * rollStability + 0.5 * pitchStability));

descentError = abs(verticalSpeed - cfg.ontology.targetDescentRate);
descentStability = 1.0 - normalize_feature(descentError, 0.0, cfg.ontology.descentTolerance);
descentStability = max(0.0, min(1.0, descentStability));

altitudeState = 1.0 - normalize_feature(max(altitudeAbovePad - cfg.labeling.touchdownAltitude, 0.0), ...
    0.0, cfg.ontology.altitudeApproachReference);
altitudeState = max(0.0, min(1.0, altitudeState));
droneSpeedStability = 1.0 - normalize_feature(horizontalSpeed, 0.0, cfg.ontology.droneHorizontalSpeedReference);
droneMotionStability = max(0.0, min(1.0, ...
    0.35 * droneSpeedStability + 0.35 * descentStability + 0.30 * attitudeStability));
roverSpeed = norm(markerVelocity(1:min(2, numel(markerVelocity))));
roverMotionStability = 1.0 - normalize_feature(roverSpeed, 0.0, cfg.ontology.roverSpeedReference);
roverMotionStability = max(0.0, min(1.0, roverMotionStability));

pathLengthEstimate = sqrt(positionError^2 + max(altitudeAbovePad - cfg.labeling.touchdownAltitude, 0.0)^2);
groundSpeed = max(0.25, norm(uavVelocity(1:2)));
timeToTargetEstimate = pathLengthEstimate / groundSpeed;
timeEfficiency = 1.0 - normalize_feature(timeToTargetEstimate, 0.0, cfg.ontology.timeToTargetReference);
timeEfficiency = max(0.0, min(1.0, timeEfficiency));
energyCostEstimate = pathLengthEstimate * dragForce + abs(uavState.altitude) * 0.15 * dragForce;
energyCostNorm = normalize_feature(energyCostEstimate, 0.0, cfg.ontology.pathLengthReference * cfg.ontology.dragForceReference);
energyEfficiency = max(0.0, min(1.0, 1.0 - energyCostNorm));

semantic = struct();
semantic.windRisk = windRisk;
semantic.visualStability = visualStability;
semantic.alignmentState = alignmentState;
semantic.attitudeStability = attitudeStability;
semantic.descentStability = descentStability;
semantic.dragRisk = dragRisk;
semantic.timeEfficiency = timeEfficiency;
semantic.energyEfficiency = energyEfficiency;
semantic.relativeDistanceState = relativeDistanceState;
semantic.altitudeState = altitudeState;
semantic.droneMotionStability = droneMotionStability;
semantic.roverMotionStability = roverMotionStability;
semantic.altitudeAbovePad = altitudeAbovePad;
semantic.roverSpeed = roverSpeed;
semantic.pathLengthEstimate = pathLengthEstimate;
semantic.timeToTargetEstimate = timeToTargetEstimate;
semantic.energyCostEstimate = energyCostEstimate;
semantic.dragForce = dragForce;
semantic.dragPower = dragPower;
semantic.relativeAirSpeed = relativeAirSpeed;
semantic.vector = [windRisk, visualStability, alignmentState, attitudeStability, descentStability, ...
    dragRisk, timeEfficiency, energyEfficiency, relativeDistanceState, altitudeState, ...
    droneMotionStability, roverMotionStability];
semantic.featureNames = cfg.featureFusion.semanticNames;
semantic.raw = struct();
semantic.raw.windSpeed = windState.speed;
semantic.raw.windAcceleration = windState.acceleration;
semantic.raw.windDirectionChange = windState.directionChange;
semantic.raw.dragDisturbance = dragDisturbance;
semantic.raw.relativeAirSpeed = relativeAirSpeed;
semantic.raw.dragForce = dragForce;
semantic.raw.dragPower = dragPower;
semantic.raw.altitudeAbovePad = altitudeAbovePad;
semantic.raw.roverSpeed = roverSpeed;
semantic.raw.pathLengthEstimate = pathLengthEstimate;
semantic.raw.timeToTargetEstimate = timeToTargetEstimate;
semantic.raw.energyCostEstimate = energyCostEstimate;

function value = get_vector_field(data, fieldName, defaultValue)
value = defaultValue;
if isstruct(data) && isfield(data, fieldName) && ~isempty(data.(fieldName))
    candidate = data.(fieldName);
    if isnumeric(candidate)
        value = candidate(:)';
    end
end

end

function wrappedAngle = wrap_to_pi(angleValue)
wrappedAngle = atan2(sin(angleValue), cos(angleValue));

end

end
