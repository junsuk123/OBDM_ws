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
cfg.simulation.logEveryNSteps = 10;
cfg.simulation.stopOnTouchdown = true;

cfg.runtime = struct();
cfg.runtime.verbose = true;

cfg.simulator = struct();
cfg.simulator.namespace = '/iris';
cfg.simulator.initialAltitude = 14.0;
cfg.simulator.initialPosition = [0.0, 0.0, 14.0];
cfg.simulator.takeoffAltitude = 5.0;
cfg.simulator.takeoffDetectionAltitude = 0.8;
cfg.simulator.initialHoverDuration = 3.0;
cfg.simulator.nominalDescentRate = 0.55;
cfg.simulator.hoverDescentRate = 0.03;
cfg.simulator.windDriftGain = 0.05;
cfg.simulator.rollGain = 0.8;
cfg.simulator.pitchGain = 0.7;
cfg.simulator.yawDriftGain = 0.04;
cfg.simulator.landingPadMotion = struct('amplitude', 1.0, 'frequency', 0.05, 'speed', 0.25);

cfg.simulator.useRosMeasurements = true;
cfg.simulator.rosReadEveryNSteps = 5;
cfg.simulator.publishWindToGazebo = true;
cfg.simulator.windPublishPeriod = 1.0;
cfg.simulator.gazeboWorldName = 'runway';

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
    'dragRisk', 'timeEfficiency', 'energyEfficiency'};

cfg.decision = struct();
cfg.decision.classNames = {'HOVER', 'LAND'};
cfg.decision.classLabels = [0, 1];
cfg.decision.minimumPosterior = 0.60;
cfg.decision.minTrainingSamples = 20;
cfg.decision.baselineThreshold = 0.60;
cfg.decision.epsilon = 1.0e-6;

cfg.guidance = struct();
cfg.guidance.enabled = true;
cfg.guidance.trajectorySource = 'ontology';
cfg.guidance.commandPeriod = 0.2;
cfg.guidance.navigationConstant = 1.2;
cfg.guidance.maxHorizontalStep = 0.35;
cfg.guidance.maxLeadTime = 2.0;
cfg.guidance.smoothingAlpha = 0.65;
cfg.guidance.alignRadius = 0.75;
cfg.guidance.finalLandRadius = 0.35;
cfg.guidance.finalLandAltitude = 0.8;
cfg.guidance.descentRate = 0.25;
cfg.guidance.searchAltitude = cfg.simulator.takeoffAltitude;
cfg.guidance.ontologyMinNavigationConstant = 0.45;
cfg.guidance.ontologyMaxNavigationConstant = 1.65;
cfg.guidance.ontologyMinHorizontalStep = 0.08;
cfg.guidance.ontologyMaxHorizontalStep = 0.85;
cfg.guidance.ontologyMinApproachStep = 0.25;
cfg.guidance.ontologyDistancePriority = 0.70;
cfg.guidance.ontologyMinLeadTime = 0.15;
cfg.guidance.ontologyMaxLeadTime = 2.0;
cfg.guidance.ontologyMinDescentRate = 0.15;
cfg.guidance.ontologyMaxDescentRate = 0.80;
cfg.guidance.ontologyFarDescentRateScale = 0.35;
cfg.guidance.ontologyLandingIntentFloor = 0.45;
cfg.guidance.ontologyDescentStartRadius = 2.20;
cfg.guidance.ontologyFinalLandMinAlignment = 0.72;
cfg.guidance.ontologyFinalLandMinVisualStability = 0.55;
cfg.guidance.ontologyFinalLandMaxWindRisk = 0.78;
cfg.guidance.padEkfProcessNoisePosition = 0.04;
cfg.guidance.padEkfProcessNoiseVelocity = 0.12;
cfg.guidance.padEkfMeasurementNoise = 0.18;

cfg.labeling = struct();
cfg.labeling.safeAltitude = 0.35;
cfg.labeling.safeVerticalSpeed = 0.75;
cfg.labeling.safeAttitudeDeg = 10.0;
cfg.labeling.safePositionError = 0.75;
cfg.labeling.safeTagStability = 0.65;
cfg.labeling.requireTagDetection = true;
cfg.labeling.safeScoreThreshold = 0.75;
cfg.labeling.touchdownAltitude = 0.35;
cfg.labeling.touchdownPositionError = 0.75;

cfg.evaluation = struct();
cfg.evaluation.windBins = [0, 2, 4, 6, 8, 10, 12, inf];
cfg.evaluation.positiveClassLabel = 1;

cfg.visualization = struct();
cfg.visualization.enabled = true;
cfg.visualization.saveFigures = true;
cfg.visualization.visible = false;

cfg.results = struct();
cfg.results.outputDir = '';
