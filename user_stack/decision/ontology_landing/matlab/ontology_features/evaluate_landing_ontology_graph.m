function ontologyGraph = evaluate_landing_ontology_graph(markerPosition, markerVelocity, uavState, semantic, landingIntentEvidence, decisionConfidence, range, closingSpeed, cfg)
%EVALUATE_LANDING_ONTOLOGY_GRAPH Unified ontology network for landing guidance.
% The graph turns semantic evidence into trajectory-generation concepts:
% risk, track quality, landing intent, future pad prediction, and descent authority.

if nargin < 5 || isempty(landingIntentEvidence)
    landingIntentEvidence = semantic_landing_intent_evidence(semantic);
end
if nargin < 6 || isempty(decisionConfidence)
    decisionConfidence = 1.0;
end
landingIntent = max(cfg.guidance.ontologyLandingIntentFloor, ...
    clamp01(landingIntentEvidence * max(0.0, min(1.0, decisionConfidence))));

operationalRisk = clamp01(0.25 * semantic.windRisk + 0.20 * semantic.dragRisk + ...
    0.18 * (1.0 - semantic.energyEfficiency) + 0.14 * (1.0 - semantic.droneMotionStability) + ...
    0.13 * (1.0 - semantic.roverMotionStability) + 0.10 * (1.0 - semantic.visualStability));
trackQuality = clamp01(0.30 * semantic.alignmentState + 0.25 * semantic.relativeDistanceState + ...
    0.25 * semantic.visualStability + 0.20 * semantic.droneMotionStability);
efficiency = clamp01(0.55 * semantic.energyEfficiency + 0.45 * semantic.timeEfficiency);

leadTimeMax = lerp(cfg.guidance.ontologyMinLeadTime, cfg.guidance.ontologyMaxLeadTime, ...
    clamp01(trackQuality * efficiency * (1.0 - 0.5 * operationalRisk)));
leadDistanceGain = get_guidance_value(cfg, 'ontologyLeadDistanceGain', 0.0);
leadTime = min(leadTimeMax, max(cfg.guidance.ontologyMinLeadTime, range / max(closingSpeed, eps) + leadDistanceGain * range));
[aimPoint, futureTime, futurePoint, leadPoint] = predicted_approach_point(markerPosition, markerVelocity, leadTime, cfg);
aimPoint = clamp_aim_offset(markerPosition, aimPoint, get_guidance_value(cfg, 'ontologyMaxAimOffset', inf));
if get_guidance_value(cfg, 'commandPadCenter', false)
    aimPoint = markerPosition(1:2);
end

precisionRadius = get_guidance_value(cfg, 'precisionApproachRadius', cfg.guidance.ontologyDescentStartRadius);
finalAlignmentRadius = get_guidance_value(cfg, 'precisionFinalAlignmentRadius', cfg.guidance.finalLandRadius);
precisionBlend = clamp01(1.0 - (range - finalAlignmentRadius) / max(precisionRadius - finalAlignmentRadius, eps));
if precisionBlend > 0.0
    precisionLeadTime = lerp( ...
        get_guidance_value(cfg, 'precisionLeadTime', 0.35), ...
        get_guidance_value(cfg, 'precisionFinalLeadTime', 0.08), ...
        precisionBlend);
    precisionFutureBlend = lerp( ...
        get_guidance_value(cfg, 'precisionFuturePointBlend', 0.20), ...
        0.0, ...
        precisionBlend);
    [precisionAim, precisionFutureTime, precisionFuturePoint, precisionLeadPoint] = ...
        predicted_approach_point_with_blend(markerPosition, markerVelocity, precisionLeadTime, precisionFutureBlend, cfg);
    precisionAim = clamp_aim_offset(markerPosition, precisionAim, get_guidance_value(cfg, 'precisionMaxAimOffset', 0.20));
    if get_guidance_value(cfg, 'commandPadCenter', false)
        precisionAim = markerPosition(1:2);
    end
    aimPoint = (1.0 - precisionBlend) * aimPoint + precisionBlend * precisionAim;
    futureTime = (1.0 - precisionBlend) * futureTime + precisionBlend * precisionFutureTime;
    futurePoint = (1.0 - precisionBlend) * futurePoint + precisionBlend * precisionFuturePoint;
    leadPoint = (1.0 - precisionBlend) * leadPoint + precisionBlend * precisionLeadPoint;
    leadTime = (1.0 - precisionBlend) * leadTime + precisionBlend * precisionLeadTime;
end

navigationGain = lerp(cfg.guidance.ontologyMinNavigationConstant, cfg.guidance.ontologyMaxNavigationConstant, ...
    clamp01((0.45 + 0.70 * trackQuality + 0.25 * semantic.timeEfficiency) * ...
    (0.65 + 0.35 * landingIntent) * (1.0 - 0.30 * operationalRisk)));
maxHorizontalStep = lerp(cfg.guidance.ontologyMinHorizontalStep, cfg.guidance.ontologyMaxHorizontalStep, ...
    clamp01((0.40 + 0.65 * trackQuality + 0.30 * efficiency) * ...
    (0.80 + 0.20 * landingIntent) * (1.0 - 0.20 * operationalRisk)));
if precisionBlend > 0.0
    precisionStep = lerp( ...
        get_guidance_value(cfg, 'precisionMaxHorizontalStep', 0.85), ...
        get_guidance_value(cfg, 'precisionFinalMaxHorizontalStep', 0.22), ...
        precisionBlend);
    maxHorizontalStep = min(maxHorizontalStep, precisionStep);
    navigationGain = lerp(navigationGain, min(navigationGain, 1.0), precisionBlend);
end
descentAuthority = clamp01(landingIntent * semantic.relativeDistanceState * semantic.altitudeState * ...
    semantic.visualStability * semantic.droneMotionStability * semantic.roverMotionStability * ...
    semantic.energyEfficiency * (1.0 - operationalRisk));
descentRate = lerp(cfg.guidance.ontologyMinDescentRate, cfg.guidance.ontologyMaxDescentRate, descentAuthority);
if precisionBlend > 0.0
    descentRate = min(descentRate, lerp( ...
        get_guidance_value(cfg, 'precisionMaxDescentRate', 0.35), ...
        get_guidance_value(cfg, 'precisionFinalDescentRate', 0.18), ...
        precisionBlend));
end
alignRadius = cfg.guidance.alignRadius * lerp(0.75, 1.25, semantic.alignmentState);
descentStartRadius = max(alignRadius, cfg.guidance.ontologyDescentStartRadius * ...
    lerp(0.65, 1.15, trackQuality) * lerp(1.05, 0.70, operationalRisk));
horizontalSpeed = 0.0;
if isfield(uavState, 'velocity') && numel(uavState.velocity) >= 2
    horizontalSpeed = norm(uavState.velocity(1:2));
end
relativeSpeed = horizontalSpeed;
if isfield(uavState, 'velocity') && numel(uavState.velocity) >= 2
    relativeSpeed = norm(uavState.velocity(1:2) - markerVelocity(1:2));
end
landingAlignment = clamp01(0.50 * semantic.alignmentState + 0.20 * semantic.visualStability + ...
    0.15 * (1.0 - normalize_range(relativeSpeed, get_guidance_value(cfg, 'precisionMaxRelativeSpeedForDescent', 0.45))) + ...
    0.15 * (1.0 - normalize_range(horizontalSpeed, get_guidance_value(cfg, 'precisionMaxHorizontalSpeedForDescent', 0.70))));
[xyGuidanceScores, selectedXYAlgorithm] = xy_guidance_algorithm_scores( ...
    semantic, trackQuality, operationalRisk, range, relativeSpeed, markerVelocity, landingAlignment, cfg);
[altitudeGuidanceScores, selectedAltitudeAlgorithm] = altitude_guidance_algorithm_scores( ...
    semantic, landingIntent, operationalRisk, range, descentAuthority, landingAlignment, cfg);
selectedGuidanceAlgorithm = selectedXYAlgorithm;
guidanceAlgorithmActivation = max(struct_values(xyGuidanceScores));
altitudeGuidanceActivation = max(struct_values(altitudeGuidanceScores));
[guidanceParameterProfiles, selectedGuidanceParameters] = guidance_parameter_settings( ...
    semantic, trackQuality, operationalRisk, range, relativeSpeed, markerVelocity, landingAlignment, ...
    descentRate, navigationGain, maxHorizontalStep, selectedGuidanceAlgorithm, cfg);

nodes = [
    node('VehicleState', 1.0, 'evidence', 'UAV pose, velocity, and attitude')
    node('PadTrack', clamp01(1.0 - normalize_range(range, cfg.ontology.positionErrorReference)), 'evidence', 'Estimated landing pad center and velocity')
    node('RelativeDistanceState', semantic.relativeDistanceState, 'semantic', 'UAV horizontal distance to landing pad')
    node('AltitudeState', semantic.altitudeState, 'semantic', 'UAV altitude above landing pad')
    node('DroneMotionStability', semantic.droneMotionStability, 'semantic', 'UAV speed, attitude, and descent stability')
    node('RoverMotionStability', semantic.roverMotionStability, 'semantic', 'Landing-pad rover/platform motion stability')
    node('WindRisk', semantic.windRisk, 'semantic', 'Wind and gust disturbance risk')
    node('DragRisk', semantic.dragRisk, 'semantic', 'Aerodynamic drag disturbance risk')
    node('VisualStability', semantic.visualStability, 'semantic', 'Marker visibility and image stability')
    node('AlignmentState', semantic.alignmentState, 'semantic', 'Horizontal/yaw alignment to pad')
    node('AttitudeStability', semantic.attitudeStability, 'semantic', 'Roll/pitch stability')
    node('DescentStability', semantic.descentStability, 'semantic', 'Vertical-speed stability')
    node('TimeEfficiency', semantic.timeEfficiency, 'semantic', 'Time-to-target efficiency')
    node('EnergyEfficiency', semantic.energyEfficiency, 'semantic', 'Energy cost efficiency')
    node('OperationalRisk', operationalRisk, 'inference', 'Unified landing risk')
    node('TrackQuality', trackQuality, 'inference', 'Quality of visual/pad tracking')
    node('LandingIntent', landingIntent, 'decision', 'Ontology-evidence-weighted intent to land')
    node('FuturePadPrediction', clamp01(futureTime / max(cfg.guidance.ontologyPredictionHorizon, eps)), 'prediction', 'Multi-step future pad estimate')
    node('ApproachAimPoint', trackQuality, 'trajectory', 'Predicted point used for horizontal guidance')
    node('HorizontalGuidance', clamp01(navigationGain / max(cfg.guidance.ontologyMaxNavigationConstant, eps)), 'trajectory', 'Horizontal pursuit authority')
    node('XYGuidanceAlgorithm', guidanceAlgorithmActivation, 'guidance_selection', 'Ontology-selected horizontal guidance law')
    node('AltitudeGuidanceAlgorithm', altitudeGuidanceActivation, 'guidance_selection', 'Ontology-selected vertical guidance law')
    node('GuidanceParameterSetting', selectedGuidanceParameters.activation, 'parameter_setting', 'Ontology-derived live tuning for the selected guidance law')
    node('LookaheadTuning', selectedGuidanceParameters.lookaheadActivation, 'parameter', 'Look-ahead and prediction-preview setting')
    node('PredictionHorizonTuning', selectedGuidanceParameters.horizonActivation, 'parameter', 'Future prediction horizon or terminal preview setting')
    node('ControlAuthorityTuning', selectedGuidanceParameters.controlActivation, 'parameter', 'Velocity, acceleration, and navigation-gain authority')
    node('SafetyEnvelopeTuning', selectedGuidanceParameters.safetyActivation, 'parameter', 'CBF corridor, terminal damping, and impact safety envelope')
    node('WindCompensationTuning', selectedGuidanceParameters.windActivation, 'parameter', 'Wind and sensor-uncertainty compensation gain')
    node('Vision_FOV_Constrained_Guidance', algorithm_activation(xyGuidanceScores.Vision_FOV_Constrained_Guidance, selectedXYAlgorithm, 'Vision_FOV_Constrained_Guidance'), 'xy_guidance_algorithm', 'Marker/FOV-stability constrained horizontal guidance')
    node('MPC_NMPC_Pad_Prediction_Tracking', algorithm_activation(xyGuidanceScores.MPC_NMPC_Pad_Prediction_Tracking, selectedXYAlgorithm, 'MPC_NMPC_Pad_Prediction_Tracking'), 'xy_guidance_algorithm', 'Short-horizon EKF pad prediction tracking')
    node('Receding_Horizon_Trajectory_Optimization_Robust_Guidance', algorithm_activation(xyGuidanceScores.Receding_Horizon_Trajectory_Optimization_Robust_Guidance, selectedXYAlgorithm, 'Receding_Horizon_Trajectory_Optimization_Robust_Guidance'), 'xy_guidance_algorithm', 'Risk-aware robust receding-horizon correction')
    node('Ontology_Gated_PD_Descent', algorithm_activation(altitudeGuidanceScores.Ontology_Gated_PD_Descent, selectedAltitudeAlgorithm, 'Ontology_Gated_PD_Descent'), 'altitude_guidance_algorithm', 'DescentAuthority-gated altitude PD')
    node('Minimum_Jerk_Terminal_Descent', algorithm_activation(altitudeGuidanceScores.Minimum_Jerk_Terminal_Descent, selectedAltitudeAlgorithm, 'Minimum_Jerk_Terminal_Descent'), 'altitude_guidance_algorithm', 'Smooth terminal minimum-jerk altitude profile')
    node('CBF_Safety_Constrained_Descent_Filter', algorithm_activation(altitudeGuidanceScores.CBF_Safety_Constrained_Descent_Filter, selectedAltitudeAlgorithm, 'CBF_Safety_Constrained_Descent_Filter'), 'altitude_guidance_algorithm', 'CBF-style deterministic vertical safety clamp')
    node('DescentAuthority', descentAuthority, 'trajectory', 'Permission to descend')
    node('LandingAlignment', landingAlignment, 'terminal', 'Stable XY and relative-speed alignment before touchdown')
    node('FinalLandReadiness', clamp01(landingIntent * trackQuality * (1.0 - operationalRisk)), 'terminal', 'Readiness for final LAND mode')
];

edges = [
    edge('WindRisk', 'OperationalRisk', 'increases', 0.25)
    edge('DragRisk', 'OperationalRisk', 'increases', 0.20)
    edge('EnergyEfficiency', 'OperationalRisk', 'reduces', 0.18)
    edge('DroneMotionStability', 'OperationalRisk', 'reduces', 0.14)
    edge('RoverMotionStability', 'OperationalRisk', 'reduces', 0.13)
    edge('VisualStability', 'OperationalRisk', 'reduces', 0.10)
    edge('AlignmentState', 'TrackQuality', 'supports', 0.30)
    edge('RelativeDistanceState', 'TrackQuality', 'supports', 0.25)
    edge('VisualStability', 'TrackQuality', 'supports', 0.25)
    edge('DroneMotionStability', 'TrackQuality', 'supports', 0.20)
    edge('TimeEfficiency', 'ApproachAimPoint', 'extends_prediction', 0.45)
    edge('PadTrack', 'FuturePadPrediction', 'predicts', 1.00)
    edge('FuturePadPrediction', 'ApproachAimPoint', 'selects', 0.55)
    edge('TrackQuality', 'HorizontalGuidance', 'scales', 0.55)
    edge('LandingIntent', 'HorizontalGuidance', 'enables', 0.30)
    edge('OperationalRisk', 'HorizontalGuidance', 'limits', 0.45)
    edge('HorizontalGuidance', 'XYGuidanceAlgorithm', 'selects_class', 0.80)
    edge('TrackQuality', 'XYGuidanceAlgorithm', 'scores', 0.45)
    edge('VisualStability', 'XYGuidanceAlgorithm', 'scores', 0.35)
    edge('OperationalRisk', 'XYGuidanceAlgorithm', 'constrains', 0.40)
    edge('WindRisk', 'XYGuidanceAlgorithm', 'biases_selection', 0.30)
    edge('DescentAuthority', 'AltitudeGuidanceAlgorithm', 'enables_descent_law', 0.45)
    edge('LandingAlignment', 'AltitudeGuidanceAlgorithm', 'enables_terminal_profile', 0.35)
    edge('OperationalRisk', 'AltitudeGuidanceAlgorithm', 'selects_safety_filter', 0.40)
    edge('XYGuidanceAlgorithm', 'GuidanceParameterSetting', 'selects_parameter_profile', guidanceAlgorithmActivation)
    edge('TrackQuality', 'GuidanceParameterSetting', 'tunes_prediction_confidence', 0.45)
    edge('VisualStability', 'GuidanceParameterSetting', 'tunes_sensor_confidence', 0.35)
    edge('OperationalRisk', 'GuidanceParameterSetting', 'limits_authority', 0.40)
    edge('WindRisk', 'GuidanceParameterSetting', 'tunes_wind_compensation', 0.35)
    edge('RoverMotionStability', 'GuidanceParameterSetting', 'tunes_target_motion_model', 0.30)
    edge('GuidanceParameterSetting', 'LookaheadTuning', 'sets', selectedGuidanceParameters.lookaheadActivation)
    edge('GuidanceParameterSetting', 'PredictionHorizonTuning', 'sets', selectedGuidanceParameters.horizonActivation)
    edge('GuidanceParameterSetting', 'ControlAuthorityTuning', 'sets', selectedGuidanceParameters.controlActivation)
    edge('GuidanceParameterSetting', 'SafetyEnvelopeTuning', 'sets', selectedGuidanceParameters.safetyActivation)
    edge('GuidanceParameterSetting', 'WindCompensationTuning', 'sets', selectedGuidanceParameters.windActivation)
    edge('LookaheadTuning', 'HorizontalGuidance', 'configures', selectedGuidanceParameters.lookaheadActivation)
    edge('PredictionHorizonTuning', 'FuturePadPrediction', 'configures', selectedGuidanceParameters.horizonActivation)
    edge('ControlAuthorityTuning', 'HorizontalGuidance', 'scales', selectedGuidanceParameters.controlActivation)
    edge('SafetyEnvelopeTuning', 'DescentAuthority', 'constrains', selectedGuidanceParameters.safetyActivation)
    edge('WindCompensationTuning', 'Receding_Horizon_Trajectory_Optimization_Robust_Guidance', 'compensates', selectedGuidanceParameters.windActivation)
    edge('XYGuidanceAlgorithm', 'Vision_FOV_Constrained_Guidance', 'classifies', xyGuidanceScores.Vision_FOV_Constrained_Guidance)
    edge('XYGuidanceAlgorithm', 'MPC_NMPC_Pad_Prediction_Tracking', 'classifies', xyGuidanceScores.MPC_NMPC_Pad_Prediction_Tracking)
    edge('XYGuidanceAlgorithm', 'Receding_Horizon_Trajectory_Optimization_Robust_Guidance', 'classifies', xyGuidanceScores.Receding_Horizon_Trajectory_Optimization_Robust_Guidance)
    edge('Vision_FOV_Constrained_Guidance', 'HorizontalGuidance', 'implements', xyGuidanceScores.Vision_FOV_Constrained_Guidance)
    edge('MPC_NMPC_Pad_Prediction_Tracking', 'HorizontalGuidance', 'implements', xyGuidanceScores.MPC_NMPC_Pad_Prediction_Tracking)
    edge('Receding_Horizon_Trajectory_Optimization_Robust_Guidance', 'HorizontalGuidance', 'implements', xyGuidanceScores.Receding_Horizon_Trajectory_Optimization_Robust_Guidance)
    edge('AltitudeGuidanceAlgorithm', 'Ontology_Gated_PD_Descent', 'classifies', altitudeGuidanceScores.Ontology_Gated_PD_Descent)
    edge('AltitudeGuidanceAlgorithm', 'Minimum_Jerk_Terminal_Descent', 'classifies', altitudeGuidanceScores.Minimum_Jerk_Terminal_Descent)
    edge('AltitudeGuidanceAlgorithm', 'CBF_Safety_Constrained_Descent_Filter', 'classifies', altitudeGuidanceScores.CBF_Safety_Constrained_Descent_Filter)
    edge('Ontology_Gated_PD_Descent', 'DescentAuthority', 'uses', altitudeGuidanceScores.Ontology_Gated_PD_Descent)
    edge('Minimum_Jerk_Terminal_Descent', 'FinalLandReadiness', 'requires', altitudeGuidanceScores.Minimum_Jerk_Terminal_Descent)
    edge('CBF_Safety_Constrained_Descent_Filter', 'DescentAuthority', 'filters', altitudeGuidanceScores.CBF_Safety_Constrained_Descent_Filter)
    edge('LandingIntent', 'DescentAuthority', 'enables', 0.20)
    edge('RelativeDistanceState', 'DescentAuthority', 'requires', 0.18)
    edge('AltitudeState', 'DescentAuthority', 'requires', 0.16)
    edge('VisualStability', 'DescentAuthority', 'requires', 0.16)
    edge('DroneMotionStability', 'DescentAuthority', 'requires', 0.16)
    edge('RoverMotionStability', 'DescentAuthority', 'requires', 0.14)
    edge('EnergyEfficiency', 'DescentAuthority', 'supports', 0.12)
    edge('OperationalRisk', 'DescentAuthority', 'limits', 0.18)
    edge('AlignmentState', 'LandingAlignment', 'requires', 0.50)
    edge('VisualStability', 'LandingAlignment', 'supports', 0.20)
    edge('DroneMotionStability', 'LandingAlignment', 'limits_relative_speed', 0.30)
    edge('LandingAlignment', 'FinalLandReadiness', 'requires', 0.45)
    edge('DescentAuthority', 'FinalLandReadiness', 'supports', 0.35)
    edge('TrackQuality', 'FinalLandReadiness', 'supports', 0.35)
    edge('OperationalRisk', 'FinalLandReadiness', 'limits', 0.30)
];

values = struct();
values.landingIntentEvidence = landingIntentEvidence;
values.decisionConfidence = decisionConfidence;
values.landingIntent = landingIntent;
values.operationalRisk = operationalRisk;
values.trackQuality = trackQuality;
values.efficiency = efficiency;
values.leadTime = leadTime;
values.leadPoint = leadPoint;
values.futurePredictionTime = futureTime;
values.futureMarkerPoint = futurePoint;
values.approachAimPoint = aimPoint;
values.navigationGain = navigationGain;
values.maxHorizontalStep = maxHorizontalStep;
values.descentAuthority = descentAuthority;
values.descentRate = descentRate;
values.descentStartRadius = descentStartRadius;
values.precisionBlend = precisionBlend;
values.landingAlignment = landingAlignment;
values.horizontalSpeed = horizontalSpeed;
values.relativeSpeed = relativeSpeed;
values.currentXYError = range;
values.finalLandReadiness = clamp01(landingIntent * trackQuality * landingAlignment * descentAuthority * (1.0 - operationalRisk));
values.selectedXYAlgorithm = selectedXYAlgorithm;
values.selectedAltitudeAlgorithm = selectedAltitudeAlgorithm;
values.selectedGuidanceAlgorithm = selectedGuidanceAlgorithm;
values.xyGuidanceScores = xyGuidanceScores;
values.altitudeGuidanceScores = altitudeGuidanceScores;
values.guidanceAlgorithmScores = xyGuidanceScores;
values.guidanceParameterProfiles = guidanceParameterProfiles;
values.selectedGuidanceParameters = selectedGuidanceParameters;
values.guidanceTuning = selectedGuidanceParameters;
values.ontologySpeedScale = selectedGuidanceParameters.speedScale;
values.ontologyAccelScale = selectedGuidanceParameters.accelScale;
values.ontologyWindGain = selectedGuidanceParameters.windGain;
values.ontologyHorizonScale = selectedGuidanceParameters.horizonScale;
values.ontologyNavigationGain = selectedGuidanceParameters.navigationGain;
values.ontologyControlAuthority = selectedGuidanceParameters.controlActivation;
values.ontologySafetyActivation = selectedGuidanceParameters.safetyActivation;
values.ontologyMaxHorizontalSpeed = get_guidance_value(cfg, 'xyMaxSpeed', maxHorizontalStep / max(cfg.simulation.dt, eps)) * ...
    clamp(selectedGuidanceParameters.speedScale * (0.35 + 0.65 * selectedGuidanceParameters.controlActivation), 0.15, 1.50);
values.ontologyMaxHorizontalAccel = get_guidance_value(cfg, 'xyMaxAccel', inf) * ...
    clamp(selectedGuidanceParameters.accelScale * (1.0 - operationalRisk), 0.20, 1.25);
values.ontologyMaxDescentRate = descentRate * ...
    clamp(0.35 + 0.65 * selectedGuidanceParameters.controlActivation, 0.25, 1.15) * ...
    clamp(1.15 - 0.35 * selectedGuidanceParameters.safetyActivation, 0.45, 1.15) * ...
    clamp(1.0 - 0.45 * operationalRisk, 0.35, 1.0);

ontologyGraph = struct();
ontologyGraph.name = 'LandingOntologyTrajectoryGraph';
ontologyGraph.nodes = nodes;
ontologyGraph.edges = edges;
ontologyGraph.values = values;
ontologyGraph.markerPosition = markerPosition;
ontologyGraph.markerVelocity = markerVelocity;
ontologyGraph.uavPosition = uavState.position;
ontologyGraph.timestamp = get_state_timestamp(uavState);
end

function [aimPoint, futureTime, futurePoint, leadPoint] = predicted_approach_point(markerPosition, markerVelocity, leadTime, cfg)
futureBlend = clamp01(get_guidance_value(cfg, 'ontologyFuturePointBlend', 0.0));
[aimPoint, futureTime, futurePoint, leadPoint] = ...
    predicted_approach_point_with_blend(markerPosition, markerVelocity, leadTime, futureBlend, cfg);
end

function [aimPoint, futureTime, futurePoint, leadPoint] = predicted_approach_point_with_blend(markerPosition, markerVelocity, leadTime, futureBlend, cfg)
predictionSteps = get_guidance_value(cfg, 'ontologyPredictionSteps', 0);
predictionStepTime = get_guidance_value(cfg, 'ontologyPredictionStepTime', 0.0);
predictionHorizon = get_guidance_value(cfg, 'ontologyPredictionHorizon', cfg.guidance.ontologyMaxLeadTime);

futureTime = min(max(leadTime, leadTime + predictionSteps * predictionStepTime), predictionHorizon);
leadPoint = markerPosition(1:2) + markerVelocity(1:2) * leadTime;
futurePoint = markerPosition(1:2) + markerVelocity(1:2) * futureTime;
aimPoint = (1.0 - futureBlend) * leadPoint + futureBlend * futurePoint;
end

function aimPoint = clamp_aim_offset(markerPosition, aimPoint, maxOffset)
offset = aimPoint - markerPosition(1:2);
offsetNorm = norm(offset);
if isfinite(maxOffset) && maxOffset > 0.0 && offsetNorm > maxOffset
    aimPoint = markerPosition(1:2) + offset ./ max(offsetNorm, eps) .* maxOffset;
end
end

function evidence = semantic_landing_intent_evidence(semantic)
riskPenalty = clamp01(0.55 * get_struct_value(semantic, 'windRisk', 0.5) + ...
    0.45 * get_struct_value(semantic, 'dragRisk', 0.5));
evidence = clamp01( ...
    0.18 * get_struct_value(semantic, 'alignmentState', 0.5) + ...
    0.16 * get_struct_value(semantic, 'relativeDistanceState', 0.5) + ...
    0.13 * get_struct_value(semantic, 'altitudeState', 0.5) + ...
    0.13 * get_struct_value(semantic, 'visualStability', 0.5) + ...
    0.11 * get_struct_value(semantic, 'descentStability', 0.5) + ...
    0.10 * get_struct_value(semantic, 'droneMotionStability', 0.5) + ...
    0.08 * get_struct_value(semantic, 'roverMotionStability', 0.5) + ...
    0.06 * get_struct_value(semantic, 'energyEfficiency', 0.5) + ...
    0.05 * (1.0 - riskPenalty));
end

function [scores, selected] = xy_guidance_algorithm_scores(semantic, trackQuality, operationalRisk, range, relativeSpeed, markerVelocity, landingAlignment, cfg)
markerSpeed = norm(markerVelocity(1:2));
close = 1.0 - normalize_range(range, get_guidance_value(cfg, 'precisionApproachRadius', cfg.guidance.alignRadius));
far = normalize_range(max(0.0, range - cfg.guidance.alignRadius), cfg.ontology.positionErrorReference);
moving = normalize_range(markerSpeed, cfg.ontology.roverSpeedReference);
relativeMotion = normalize_range(relativeSpeed, get_guidance_value(cfg, 'precisionMaxRelativeSpeedForDescent', 0.45));
sensorUncertainty = 1.0 - clamp01(0.55 * trackQuality + 0.45 * semantic.visualStability);
terminal = clamp01(landingAlignment);

scores = struct();
scores.Vision_FOV_Constrained_Guidance = clamp01( ...
    0.30 + 0.35 * semantic.visualStability + 0.20 * trackQuality + ...
    0.10 * close + 0.05 * (1.0 - sensorUncertainty));
scores.MPC_NMPC_Pad_Prediction_Tracking = clamp01( ...
    0.22 + 0.35 * trackQuality + 0.20 * moving + 0.15 * far + ...
    0.08 * semantic.roverMotionStability - 0.15 * operationalRisk);
scores.Receding_Horizon_Trajectory_Optimization_Robust_Guidance = clamp01( ...
    0.20 + 0.30 * max(semantic.windRisk, semantic.dragRisk) + ...
    0.20 * relativeMotion + 0.15 * moving + 0.10 * sensorUncertainty + ...
    0.05 * (1.0 - terminal));

if close > 0.70 && semantic.visualStability >= 0.65
    scores.Vision_FOV_Constrained_Guidance = clamp01(scores.Vision_FOV_Constrained_Guidance + 0.12);
end
if trackQuality >= 0.65 && moving > 0.10
    scores.MPC_NMPC_Pad_Prediction_Tracking = clamp01(scores.MPC_NMPC_Pad_Prediction_Tracking + 0.12);
end
if operationalRisk >= 0.45 || semantic.windRisk >= 0.45
    scores.Receding_Horizon_Trajectory_Optimization_Robust_Guidance = clamp01(scores.Receding_Horizon_Trajectory_Optimization_Robust_Guidance + 0.15);
end

names = fieldnames(scores);
values = struct_values(scores);
[~, bestIdx] = max(values);
selected = names{bestIdx};
end

function [scores, selected] = altitude_guidance_algorithm_scores(semantic, landingIntent, operationalRisk, range, descentAuthority, landingAlignment, cfg)
terminalRange = get_guidance_value(cfg, 'terminalXYThreshold', cfg.guidance.finalLandRadius);
terminalClose = 1.0 - normalize_range(range, max(terminalRange, eps));
finalReadiness = clamp01(landingIntent * landingAlignment * descentAuthority * ...
    semantic.visualStability * (1.0 - operationalRisk));

scores = struct();
scores.Ontology_Gated_PD_Descent = clamp01(0.45 + 0.30 * descentAuthority + ...
    0.15 * semantic.descentStability + 0.10 * (1.0 - operationalRisk));
scores.Minimum_Jerk_Terminal_Descent = clamp01(0.10 + 0.55 * finalReadiness + ...
    0.20 * terminalClose + 0.15 * semantic.visualStability);
scores.CBF_Safety_Constrained_Descent_Filter = clamp01(0.20 + 0.35 * operationalRisk + ...
    0.25 * (1.0 - landingAlignment) + 0.20 * (1.0 - descentAuthority));

if scores.Minimum_Jerk_Terminal_Descent < get_guidance_value(cfg, 'terminalReadinessThreshold', 0.68)
    scores.Minimum_Jerk_Terminal_Descent = 0.0;
end

names = fieldnames(scores);
values = struct_values(scores);
[~, bestIdx] = max(values);
selected = names{bestIdx};
if strcmp(selected, 'CBF_Safety_Constrained_Descent_Filter')
    selected = 'Ontology_Gated_PD_Descent';
end
end

function [profiles, selectedProfile] = guidance_parameter_settings(semantic, trackQuality, operationalRisk, range, relativeSpeed, markerVelocity, landingAlignment, descentRate, navigationGain, maxHorizontalStep, selected, cfg)
markerSpeed = norm(markerVelocity(1:2));
visualStability = get_struct_value(semantic, 'visualStability', trackQuality);
windRisk = get_struct_value(semantic, 'windRisk', 0.0);
providerConfidence = visualStability;
relativeMotion = normalize_range(relativeSpeed, get_guidance_value(cfg, 'precisionMaxRelativeSpeedForDescent', 0.45));
moving = normalize_range(markerSpeed, cfg.ontology.roverSpeedReference);
close = 1.0 - normalize_range(range, get_guidance_value(cfg, 'precisionApproachRadius', cfg.guidance.alignRadius));
sensorUncertainty = 1.0 - clamp01(0.55 * trackQuality + 0.30 * visualStability + 0.15 * providerConfidence);
damping = clamp01(0.25 * operationalRisk + 0.25 * windRisk + 0.25 * relativeMotion + 0.15 * sensorUncertainty + 0.10 * close);
predictionConfidence = clamp01(0.55 * trackQuality + 0.30 * visualStability + 0.15 * (1.0 - sensorUncertainty));
horizonScale = clamp_range(0.45 + 0.75 * predictionConfidence + 0.35 * moving - 0.30 * landingAlignment, 0.25, 1.25);
speedScale = clamp_range(1.0 - 0.45 * damping - 0.15 * landingAlignment + 0.15 * moving, 0.30, 1.10);
accelScale = clamp_range(1.0 - 0.35 * damping + 0.15 * moving - 0.15 * landingAlignment, 0.35, 1.10);
windGain = clamp_range(0.10 + 0.45 * windRisk + 0.15 * sensorUncertainty, 0.0, 0.65);
safetyEnvelope = clamp01(0.25 + 0.45 * operationalRisk + 0.25 * close + 0.20 * landingAlignment);

baseProfile = struct( ...
    'activation', clamp01(0.40 + 0.30 * predictionConfidence + 0.20 * (1.0 - operationalRisk) + 0.10 * visualStability), ...
    'lookaheadActivation', clamp01(0.35 + 0.45 * predictionConfidence + 0.20 * (1.0 - close)), ...
    'horizonActivation', clamp01(horizonScale / 1.25), ...
    'controlActivation', clamp01(0.50 * speedScale + 0.50 * accelScale), ...
    'safetyActivation', safetyEnvelope, ...
    'windActivation', clamp01(windGain / 0.65), ...
    'horizonScale', horizonScale, ...
    'speedScale', speedScale, ...
    'accelScale', accelScale, ...
    'windGain', windGain, ...
    'navigationGain', navigationGain, ...
    'maxHorizontalStep', maxHorizontalStep, ...
    'descentRate', descentRate);

profiles = struct();
profiles.Vision_FOV_Constrained_Guidance = with_algorithm_parameters(baseProfile, 'Vision_FOV_Constrained_Guidance', 0.60, 0.45, 0.75);
profiles.MPC_NMPC_Pad_Prediction_Tracking = with_algorithm_parameters(baseProfile, 'MPC_NMPC_Pad_Prediction_Tracking', 0.85, 1.00, 0.85);
profiles.Receding_Horizon_Trajectory_Optimization_Robust_Guidance = with_algorithm_parameters(baseProfile, 'Receding_Horizon_Trajectory_Optimization_Robust_Guidance', 0.70, 0.80, max(0.95, safetyEnvelope));

if isfield(profiles, selected)
    selectedProfile = profiles.(selected);
else
    selectedProfile = baseProfile;
    selectedProfile.algorithm = char(string(selected));
end
selectedProfile.activation = clamp01(max(selectedProfile.activation, 0.82));
end

function profile = with_algorithm_parameters(baseProfile, algorithm, lookaheadWeight, horizonWeight, safetyWeight)
profile = baseProfile;
profile.algorithm = algorithm;
profile.lookaheadActivation = clamp01(profile.lookaheadActivation * lookaheadWeight);
profile.horizonActivation = clamp01(profile.horizonActivation * horizonWeight);
profile.safetyActivation = clamp01(profile.safetyActivation * safetyWeight);
profile.activation = clamp01(mean([profile.lookaheadActivation, profile.horizonActivation, profile.controlActivation, profile.safetyActivation, profile.windActivation]));
end

function value = get_struct_value(source, fieldName, defaultValue)
value = defaultValue;
if isstruct(source) && isfield(source, fieldName) && ~isempty(source.(fieldName))
    value = source.(fieldName);
end
end

function value = clamp_range(value, lo, hi)
value = max(lo, min(hi, value));
end

function activation = algorithm_activation(score, selected, name)
activation = clamp01(score);
if strcmp(char(string(selected)), name)
    activation = clamp01(max(activation, 0.82));
end
end

function values = struct_values(source)
names = fieldnames(source);
values = zeros(numel(names), 1);
for idx = 1:numel(names)
    values(idx) = double(source.(names{idx}));
end
end

function item = node(name, activation, kind, description)
item = struct('name', name, 'activation', clamp01(activation), 'kind', kind, 'description', description);
end

function item = edge(source, target, relation, weight)
item = struct('source', source, 'target', target, 'relation', relation, 'weight', weight);
end

function value = normalize_range(value, reference)
value = max(0.0, min(1.0, value / max(reference, eps)));
end

function value = lerp(a, b, t)
value = a + (b - a) * clamp01(t);
end

function value = clamp01(value)
value = max(0.0, min(1.0, value));
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end

function timestamp = get_state_timestamp(uavState)
timestamp = NaN;
if isfield(uavState, 'timestamp')
    timestamp = uavState.timestamp;
end
end
