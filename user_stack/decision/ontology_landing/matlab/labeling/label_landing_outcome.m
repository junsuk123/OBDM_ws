function [label, labelName, details] = label_landing_outcome(uavState, observation, cfg)
%LABEL_LANDING_OUTCOME Auto-label safe and unsafe landing outcomes.

safeAltitude = uavState.altitude <= cfg.labeling.safeAltitude;
safeVerticalSpeed = abs(uavState.velocity(3)) <= cfg.labeling.safeVerticalSpeed;
safeAttitude = max(abs(rad2deg(uavState.attitude(1:2)))) <= cfg.labeling.safeAttitudeDeg;
safePosition = observation.positionError <= cfg.labeling.safePositionError;
safeTag = observation.visualStability >= cfg.labeling.safeTagStability;
if cfg.labeling.requireTagDetection
    safeTag = safeTag && observation.tagDetected;
end

score = mean([safeAltitude, safeVerticalSpeed, safeAttitude, safePosition, safeTag]);
label = double(score >= cfg.labeling.safeScoreThreshold);

if label == 1
    labelName = 'SAFE_LANDING';
else
    labelName = 'UNSAFE_LANDING';
end

details = struct();
details.score = score;
details.safeAltitude = safeAltitude;
details.safeVerticalSpeed = safeVerticalSpeed;
details.safeAttitude = safeAttitude;
details.safePosition = safePosition;
details.safeTag = safeTag;