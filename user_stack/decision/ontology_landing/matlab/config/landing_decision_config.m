function cfg = landing_decision_config()
%LANDING_DECISION_CONFIG Base configuration for the landing decision framework.

cfg = struct();

cfg.project = struct();
cfg.project.name = 'ontology_landing_framework';
cfg.project.description = 'Ontology-based probabilistic UAV autonomous landing decision framework';

cfg.simulation = struct();
cfg.simulation.duration = 45;
cfg.simulation.dt = 0.2;
cfg.simulation.timeVector = 0:cfg.simulation.dt:cfg.simulation.duration;
cfg.simulation.numScenarios = 8;
cfg.simulation.seed = 17;
cfg.simulation.backend = 'integrated_main_flow';
autoLaunchEnv = getenv('OBDM_ENABLE_AUTO_SIM_EXECUTION');
if isempty(autoLaunchEnv)
    cfg.simulation.autoLaunch = true;
else
    cfg.simulation.autoLaunch = ~strcmpi(strtrim(autoLaunchEnv), '0');
end
cfg.simulation.runMode = getenv('OBDM_RUN_MODE');
if isempty(cfg.simulation.runMode)
    cfg.simulation.runMode = 'gazebo_bridge';
end
cfg.simulation.waitForTopics = true;
cfg.simulation.postLaunchWaitSec = 15;
cfg.simulation.rosDomainId = 0;
cfg.simulation.gzVersion = 'harmonic';
cfg.simulation.allowFallbackToMock = false;
cfg.simulation.forceInitialHardReset = true;
cfg.simulation.logEveryNSteps = 10;
cfg.simulation.stopOnTouchdown = true;
resetMethodEnv = getenv('OBDM_RESET_METHOD');
if isempty(resetMethodEnv)
    cfg.simulation.resetMethod = 'soft';
else
    cfg.simulation.resetMethod = lower(strtrim(resetMethodEnv));
end

cfg.runtime = struct();
cfg.runtime.verbose = true;

cfg.simulator = struct();
cfg.simulator.namespace = '/iris';
cfg.simulator.initialAltitude = 14.0;
cfg.simulator.initialPosition = [0.0, 0.0, 14.0];
cfg.simulator.takeoffAltitude = 5.0;
cfg.simulator.takeoffDetectionAltitude = 0.8;
cfg.simulator.rosMaxPositionJump = 8.0;
cfg.simulator.rosMaxHorizontalSpeed = 25.0;
cfg.simulator.rosMaxVerticalSpeed = 8.0;
cfg.simulator.holdLastRosStateOnDropout = true;
cfg.simulator.initialHoverDuration = 3.0;
cfg.simulator.nominalDescentRate = 0.55;
cfg.simulator.hoverDescentRate = 0.03;
cfg.simulator.windDriftGain = 0.05;
cfg.simulator.rollGain = 0.8;
cfg.simulator.pitchGain = 0.7;
cfg.simulator.yawDriftGain = 0.04;
cfg.simulator.landingPadMotion = struct('amplitude', 1.0, 'frequency', 0.05, 'speed', 0.25);

cfg.simulator.useRosMeasurements = true;
cfg.simulator.rosReadEveryNSteps = 1;
cfg.simulator.publishWindToGazebo = true;
cfg.simulator.windPublishPeriod = 0.1;
cfg.simulator.gazeboWorldName = 'runway';
cfg.simulator.reset = struct();
cfg.simulator.reset.request = 'pose';
cfg.simulator.reset.gazeboServiceTimeoutMs = 2000;
cfg.simulator.reset.serviceAttempts = 2;
cfg.simulator.reset.settleSeconds = 0.25;
cfg.simulator.reset.poseSampleTimeoutSeconds = 1.5;
cfg.simulator.reset.poseVerifyTimeoutSeconds = 8.0;
cfg.simulator.reset.poseVerifySuccesses = 2;
cfg.simulator.reset.poseVerifyIntervalSeconds = 0.35;
cfg.simulator.reset.rosCleanupGraceSeconds = 0.4;
cfg.simulator.resetControl = struct();
cfg.simulator.resetControl.padStopSettleSeconds = 0.20;
cfg.simulator.resetControl.droneStopSettleSeconds = 0.20;
cfg.simulator.resetControl.disarmTimeoutSeconds = 4.0;
cfg.simulator.resetControl.landDisarmTimeoutSeconds = 6.0;
cfg.simulator.resetControl.runtimeCleanupGraceSeconds = 0.35;
cfg.simulator.reset.models = struct( ...
    'name', {'iris', 'r1_rover'}, ...
    'position', {[0.0, 0.0, 0.2], [5.0, 0.0, 0.05]}, ...
    'attitude', {[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]}, ...
    'tolerance', {0.75, 0.35}, ...
    'attitudeTolerance', {0.35, 0.35});

cfg.communication = struct();
cfg.communication.uavStateTopic = '/iris/odometry';
cfg.communication.windTopic = '/iris/wind';
cfg.communication.padObservationTopic = '/landing_pad/observation';
cfg.communication.takeoffCommand = '/mavros/cmd/takeoff';
cfg.communication.hoverCommand = '/mavros/set_mode/hover';
cfg.communication.landCommand = '/mavros/set_mode/land';

cfg.wind = struct();
cfg.wind.meanVelocity = [3.0, 0.8, 0.0];
cfg.wind.gustStd = [1.6, 1.1, 0.4];
cfg.wind.turbulenceTau = 2.5;
cfg.wind.directionChangeStd = 0.18;
cfg.wind.maxSpeed = 14.0;
cfg.wind.accelerationScale = 2.5;

cfg.sensor = struct();
cfg.sensor.positionNoiseStd = 0.18;
cfg.sensor.velocityNoiseStd = 0.08;
cfg.sensor.attitudeNoiseStdDeg = 1.2;
cfg.sensor.windNoiseStd = 0.35;
cfg.sensor.tagCenterNoiseStd = 0.06;
cfg.sensor.stabilityNoiseStd = 0.05;

cfg.ontology = struct();
cfg.ontology.referenceYawDeg = 0.0;
cfg.ontology.windSpeedReference = 10.0;
cfg.ontology.windAccelerationReference = 3.0;
cfg.ontology.windDirectionChangeReference = 0.75;
cfg.ontology.dragReference = 6.0;
cfg.ontology.positionErrorReference = 2.5;
cfg.ontology.yawErrorReferenceDeg = 20.0;
cfg.ontology.rollReferenceDeg = 12.0;
cfg.ontology.pitchReferenceDeg = 12.0;
cfg.ontology.verticalSpeedReference = 1.2;
cfg.ontology.targetDescentRate = 0.55;
cfg.ontology.descentTolerance = 0.75;
cfg.ontology.tagVisibleAltitude = 6.0;
cfg.ontology.altitudeApproachReference = cfg.ontology.tagVisibleAltitude;
cfg.ontology.droneHorizontalSpeedReference = 3.0;
cfg.ontology.roverSpeedReference = 1.2;
cfg.ontology.tagDetectionThreshold = 0.65;
cfg.ontology.airDensity = 1.2041;
cfg.ontology.dragCoefficient = 1.05;
cfg.ontology.referenceArea = 0.08;
cfg.ontology.dragForceReference = 8.0;
cfg.ontology.dragPowerReference = 45.0;
cfg.ontology.pathLengthReference = 8.0;
cfg.ontology.timeToTargetReference = 12.0;

cfg.featureFusion = struct();
cfg.featureFusion.rawNames = { ...
    'altitude', 'verticalSpeed', 'horizontalSpeed', 'positionError', ...
    'windSpeed', 'windAcceleration', 'tagCenterError', 'visualStability', 'tagDetected'};
cfg.featureFusion.rawMin = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
cfg.featureFusion.rawMax = [18.0, 4.0, 10.0, 6.0, 14.0, 5.0, 1.0, 1.0, 1.0];
cfg.featureFusion.semanticNames = { ...
    'windRisk', 'visualStability', 'alignmentState', 'attitudeStability', 'descentStability', ...
    'dragRisk', 'timeEfficiency', 'energyEfficiency', 'relativeDistanceState', 'altitudeState', ...
    'droneMotionStability', 'roverMotionStability'};

cfg.decision = struct();
cfg.decision.classNames = {'HOVER', 'LAND'};
cfg.decision.classLabels = [0, 1];
cfg.decision.ontologyIntentThreshold = 0.62;
cfg.decision.epsilon = 1.0e-6;

cfg.guidance = struct();
cfg.guidance.enabled = true;
cfg.guidance.trajectorySource = 'ontology';
cfg.guidance.commandPeriod = 0.1;
cfg.guidance.navigationConstant = 1.2;
cfg.guidance.maxHorizontalStep = 1.20;
cfg.guidance.maxLeadTime = 1.0;
cfg.guidance.smoothingAlpha = 1.00;
cfg.guidance.alignRadius = 0.75;
cfg.guidance.finalLandRadius = 0.35;
cfg.guidance.finalLandAltitude = 0.8;
cfg.guidance.descentRate = 0.25;
cfg.guidance.searchAltitude = cfg.simulator.takeoffAltitude;
cfg.guidance.maxSearchAltitude = cfg.simulator.takeoffAltitude;
cfg.guidance.ontologyMinNavigationConstant = 1.50;
cfg.guidance.ontologyMaxNavigationConstant = 4.80;
cfg.guidance.ontologyMinHorizontalStep = 0.45;
cfg.guidance.ontologyMaxHorizontalStep = 3.00;
cfg.guidance.ontologyMinApproachStep = 0.20;
cfg.guidance.ontologyDistancePriority = 0.85;
cfg.guidance.ontologyMinLeadTime = 0.15;
cfg.guidance.ontologyMaxLeadTime = 0.75;
cfg.guidance.ontologyLeadDistanceGain = 0.05;
cfg.guidance.horizontalResponseScale = 1.00;
cfg.guidance.ontologyPredictionSteps = 3;
cfg.guidance.ontologyPredictionStepTime = 0.10;
cfg.guidance.ontologyPredictionHorizon = 1.00;
cfg.guidance.ontologyFuturePointBlend = 0.15;
cfg.guidance.ontologyMaxAimOffset = 0.60;
cfg.guidance.commandPadCenter = true;
cfg.guidance.useRealtimeTrajectoryGenerator = true;
cfg.guidance.matlabSetpointPublisherEnabled = false;
cfg.guidance.defaultXYGuidanceAlgorithm = 'Vision_FOV_Constrained_Guidance';
cfg.guidance.defaultAltitudeGuidanceAlgorithm = 'Ontology_Gated_PD_Descent';
cfg.guidance.altitudeSafetyFilterEnabled = true;
cfg.guidance.xyMaxSpeed = 2.20;
cfg.guidance.xyMaxAccel = 1.50;
cfg.guidance.xyAccelLimitRadius = 1.20;
cfg.guidance.visionTagErrorReference = 2.50;
cfg.guidance.visionKpXY = 0.85;
cfg.guidance.visionKdXY = 0.35;
cfg.guidance.visionPadVelocityFeedforward = 0.55;
cfg.guidance.mpcPredictionHorizon = 0.80;
cfg.guidance.mpcKpXY = 1.05;
cfg.guidance.mpcKdXY = 0.45;
cfg.guidance.robustLambdaXY = 0.85;
cfg.guidance.robustBoundaryLayerXY = 0.65;
cfg.guidance.robustKpXY = 0.90;
cfg.guidance.robustKdXY = 0.40;
cfg.guidance.robustCorrectionGainXY = 0.55;
cfg.guidance.altitudePdKp = 0.75;
cfg.guidance.altitudePdKd = 0.25;
cfg.guidance.maxDescentRate = 0.55;
cfg.guidance.maxAscentRate = 0.70;
cfg.guidance.terminalXYThreshold = 0.45;
cfg.guidance.terminalVisionThreshold = 0.70;
cfg.guidance.terminalWindRiskThreshold = 0.45;
cfg.guidance.terminalAltitudeThreshold = 1.20;
cfg.guidance.terminalReadinessThreshold = 0.68;
cfg.guidance.terminalDescentDuration = 2.80;
cfg.guidance.minimumJerkKp = 0.95;
cfg.guidance.minimumJerkKd = 0.35;
cfg.guidance.altitudeCbfFarXYError = 2.20;
cfg.guidance.altitudeCbfReferenceAltitude = 5.00;
cfg.guidance.altitudeRiskGateMin = 0.35;
cfg.guidance.precisionApproachRadius = 2.50;
cfg.guidance.precisionAlignmentRadius = 0.75;
cfg.guidance.precisionFinalAlignmentRadius = 0.40;
cfg.guidance.precisionMaxHorizontalStep = 0.85;
cfg.guidance.precisionFinalMaxHorizontalStep = 0.22;
cfg.guidance.precisionMinApproachStep = 0.03;
cfg.guidance.precisionLeadTime = 0.35;
cfg.guidance.precisionFinalLeadTime = 0.08;
cfg.guidance.precisionFuturePointBlend = 0.05;
cfg.guidance.precisionMaxAimOffset = 0.20;
cfg.guidance.precisionDescentHoldHeightAbovePad = 0.70;
cfg.guidance.precisionMaxRelativeSpeedForDescent = 1.25;
cfg.guidance.precisionMaxHorizontalSpeedForDescent = 1.20;
cfg.guidance.precisionMaxDescentRate = 0.35;
cfg.guidance.precisionFinalDescentRate = 0.18;
cfg.guidance.markerLostClimbRate = 1.20;
cfg.guidance.ontologyMinDescentRate = 0.15;
cfg.guidance.ontologyMaxDescentRate = 0.55;
cfg.guidance.ontologyFarDescentRateScale = 0.35;
cfg.guidance.ontologyLandingIntentFloor = 0.45;
cfg.guidance.ontologyDescentStartRadius = 2.20;
cfg.guidance.ontologyFinalLandMinAlignment = 0.72;
cfg.guidance.ontologyFinalLandMinVisualStability = 0.55;
cfg.guidance.ontologyFinalLandMaxWindRisk = 0.78;
cfg.guidance.padEkfProcessNoisePosition = 0.04;
cfg.guidance.padEkfProcessNoiseVelocity = 0.12;
cfg.guidance.padEkfMeasurementNoise = 0.18;
cfg.guidance.padEkfUseFallbackMarkerMeasurement = true;
cfg.guidance.padEkfResetDistance = 4.0;
cfg.guidance.padEkfMaxGuidanceAge = 2.5;
cfg.guidance.touchdownHeightAbovePad = 0.15;
cfg.guidance.directLandRadius = 0.25;
cfg.guidance.directLandAltitudeAbovePad = 0.55;
cfg.guidance.directLandStableTime = 0.15;
cfg.guidance.directLandMaxHorizontalSpeed = 1.20;
cfg.guidance.directLandMaxRelativeSpeed = 1.35;
cfg.guidance.directLandRequireRawTag = false;
cfg.guidance.directLandMaxEkfAge = 0.60;
cfg.guidance.directLandMinTrackQuality = 0.65;

cfg.labeling = struct();
cfg.labeling.safeAltitude = 0.55;
cfg.labeling.safeVerticalSpeed = 1.50;
cfg.labeling.safeAttitudeDeg = 10.0;
cfg.labeling.safePositionError = 1.15;
cfg.labeling.safeTagStability = 0.65;
cfg.labeling.requireTagDetection = true;
cfg.labeling.safeScoreThreshold = 0.75;
cfg.labeling.touchdownAltitude = 0.80;
cfg.labeling.touchdownPositionError = 0.35;
cfg.labeling.touchdownGroundAltitude = 1.00;
cfg.labeling.touchdownGroundPositionError = 1000.0;
cfg.labeling.touchdownGroundResetPositionError = inf;
cfg.labeling.touchdownMaxAbsVz = 3.00;
cfg.labeling.contactArmingAltitude = 1.50;
cfg.labeling.contactMinFlightTime = 1.50;
cfg.labeling.contactMinSamples = 5;
cfg.labeling.stopOnlyOnPadTouchdown = true;
cfg.labeling.resetOnGroundContact = true;
cfg.labeling.allowTouchdownThresholdFallback = true;

cfg.evaluation = struct();
cfg.evaluation.windBins = [0, 2, 4, 6, 8, 10, 12, inf];
cfg.evaluation.positiveClassLabel = 1;

cfg.reinforcement = struct();
cfg.reinforcement.enabled = true;
cfg.reinforcement.trackingErrorReference = cfg.ontology.positionErrorReference;
cfg.reinforcement.targetSmoothnessReference = 4.0;
cfg.reinforcement.smoothnessWeight = 0.22;
cfg.reinforcement.trackingWeight = 0.30;
cfg.reinforcement.efficiencyWeight = 0.16;
cfg.reinforcement.coverageWeight = 0.10;
cfg.reinforcement.trackQualityWeight = 0.12;
cfg.reinforcement.riskWeight = 0.10;
cfg.reinforcement.successBonus = 1.0;
cfg.reinforcement.failurePenalty = 1.0;

cfg.visualization = struct();
cfg.visualization.enabled = true;
cfg.visualization.saveFigures = true;
cfg.visualization.visible = false;
cfg.visualization.showOntologyGraph = true;

cfg.results = struct();
cfg.results.outputDir = '';
