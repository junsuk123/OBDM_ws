function ontologyGraph = evaluate_landing_ontology_graph(markerPosition, markerVelocity, uavState, semantic, classProbability, confidence, range, closingSpeed, cfg)
%EVALUATE_LANDING_ONTOLOGY_GRAPH Unified ontology network for landing guidance.
% The graph turns semantic evidence into trajectory-generation concepts:
% risk, track quality, landing intent, future pad prediction, and descent authority.

landingPosterior = classProbability(min(2, numel(classProbability)));
landingIntent = max(cfg.guidance.ontologyLandingIntentFloor, ...
    clamp01(landingPosterior * max(0.0, min(1.0, confidence))));

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

navigationGain = lerp(cfg.guidance.ontologyMinNavigationConstant, cfg.guidance.ontologyMaxNavigationConstant, ...
    clamp01((0.45 + 0.70 * trackQuality + 0.25 * semantic.timeEfficiency) * ...
    (0.65 + 0.35 * landingIntent) * (1.0 - 0.30 * operationalRisk)));
maxHorizontalStep = lerp(cfg.guidance.ontologyMinHorizontalStep, cfg.guidance.ontologyMaxHorizontalStep, ...
    clamp01((0.40 + 0.65 * trackQuality + 0.30 * efficiency) * ...
    (0.80 + 0.20 * landingIntent) * (1.0 - 0.20 * operationalRisk)));
descentAuthority = clamp01(landingIntent * semantic.relativeDistanceState * semantic.altitudeState * ...
    semantic.visualStability * semantic.droneMotionStability * semantic.roverMotionStability * ...
    semantic.energyEfficiency * (1.0 - operationalRisk));
descentRate = lerp(cfg.guidance.ontologyMinDescentRate, cfg.guidance.ontologyMaxDescentRate, descentAuthority);
alignRadius = cfg.guidance.alignRadius * lerp(0.75, 1.25, semantic.alignmentState);
descentStartRadius = max(alignRadius, cfg.guidance.ontologyDescentStartRadius * ...
    lerp(0.65, 1.15, trackQuality) * lerp(1.05, 0.70, operationalRisk));

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
    node('LandingIntent', landingIntent, 'decision', 'Posterior-weighted intent to land')
    node('FuturePadPrediction', clamp01(futureTime / max(cfg.guidance.ontologyPredictionHorizon, eps)), 'prediction', 'Multi-step future pad estimate')
    node('ApproachAimPoint', trackQuality, 'trajectory', 'Predicted point used for horizontal guidance')
    node('HorizontalGuidance', clamp01(navigationGain / max(cfg.guidance.ontologyMaxNavigationConstant, eps)), 'trajectory', 'Horizontal pursuit authority')
    node('DescentAuthority', descentAuthority, 'trajectory', 'Permission to descend')
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
    edge('LandingIntent', 'DescentAuthority', 'enables', 0.20)
    edge('RelativeDistanceState', 'DescentAuthority', 'requires', 0.18)
    edge('AltitudeState', 'DescentAuthority', 'requires', 0.16)
    edge('VisualStability', 'DescentAuthority', 'requires', 0.16)
    edge('DroneMotionStability', 'DescentAuthority', 'requires', 0.16)
    edge('RoverMotionStability', 'DescentAuthority', 'requires', 0.14)
    edge('EnergyEfficiency', 'DescentAuthority', 'supports', 0.12)
    edge('OperationalRisk', 'DescentAuthority', 'limits', 0.18)
    edge('DescentAuthority', 'FinalLandReadiness', 'supports', 0.35)
    edge('TrackQuality', 'FinalLandReadiness', 'supports', 0.35)
    edge('OperationalRisk', 'FinalLandReadiness', 'limits', 0.30)
];

values = struct();
values.landingPosterior = landingPosterior;
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
predictionSteps = get_guidance_value(cfg, 'ontologyPredictionSteps', 0);
predictionStepTime = get_guidance_value(cfg, 'ontologyPredictionStepTime', 0.0);
predictionHorizon = get_guidance_value(cfg, 'ontologyPredictionHorizon', cfg.guidance.ontologyMaxLeadTime);
futureBlend = clamp01(get_guidance_value(cfg, 'ontologyFuturePointBlend', 0.0));

futureTime = min(max(leadTime, leadTime + predictionSteps * predictionStepTime), predictionHorizon);
leadPoint = markerPosition(1:2) + markerVelocity(1:2) * leadTime;
futurePoint = markerPosition(1:2) + markerVelocity(1:2) * futureTime;
aimPoint = (1.0 - futureBlend) * leadPoint + futureBlend * futurePoint;
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
