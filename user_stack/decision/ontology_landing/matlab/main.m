% Main entry point for the ontology-based probabilistic landing framework.

close all force;
clearvars;
clear classes;
clc;

% ===================== USER PIPELINE SWITCHES =====================
% A single run can collect data, train a model/semantic policy, and validate
% the result. Turn stages on/off here without editing lower-level modules.
MAIN_PIPELINE = struct();
MAIN_PIPELINE.collectData = true;
MAIN_PIPELINE.trainModel = true;
MAIN_PIPELINE.trainOntologyWeights = true;
MAIN_PIPELINE.validateModel = true;
MAIN_PIPELINE.useHistoricalTrainingData = true;
MAIN_PIPELINE.maxHistoricalRuns = 20;
MAIN_PIPELINE.runTraceCollector = true;
MAIN_PIPELINE.runAutoTuningAnalysis = true;
MAIN_PIPELINE.savePlots = true;
MAIN_PIPELINE.shutdownSimulatorOnFinish = true;
% Used when collectData=false. Leave empty to auto-use the latest result MAT.
MAIN_PIPELINE.inputResultsMat = '';
% Used when trainModel=false and validateModel=true. Leave empty to use a
% model stored in inputResultsMat or the model trained during this run.
MAIN_PIPELINE.pretrainedModelMat = '';

% ===================== USER TUNING PARAMETERS =====================
% These values are copied into cfg before any collection/training/validation.
MAIN_TUNING = struct();
MAIN_TUNING.dataCollectionRepeats = 100;
MAIN_TUNING.numScenarios = MAIN_TUNING.dataCollectionRepeats; % Backward-compatible alias.
MAIN_TUNING.resetMethod = 'soft'; % soft: Gazebo world reset + ROS node cleanup; hard: kill Gazebo/RViz/ROS.
MAIN_TUNING.softResetRequest = 'pose';
MAIN_TUNING.softResetSettleSeconds = 0.25;
MAIN_TUNING.softResetPoseSampleTimeoutSeconds = 1.5;
MAIN_TUNING.softResetRosCleanupGraceSeconds = 0.4;
MAIN_TUNING.softResetPoseToleranceDrone = 0.75;
MAIN_TUNING.softResetPoseToleranceRover = 0.35;
MAIN_TUNING.duration = 45.0;
MAIN_TUNING.dt = 0.2;
MAIN_TUNING.stopOnTouchdown = true;
MAIN_TUNING.showOntologyGraph = true;
MAIN_TUNING.visualizationVisible = true;
MAIN_TUNING.takeoffAltitude = 5.0;
MAIN_TUNING.padMotionAmplitude = 1.0;
MAIN_TUNING.padMotionFrequency = 0.01;
MAIN_TUNING.padMotionSpeed = 3.0;
MAIN_TUNING.directLandRadius = 0.7;
MAIN_TUNING.directLandAltitudeAbovePad = 0.45;
MAIN_TUNING.directLandStableTime = 0.40;
MAIN_TUNING.directLandMaxHorizontalSpeed = 2.00;
MAIN_TUNING.contactArmingAltitude = 1.50;
MAIN_TUNING.contactMinFlightTime = 1.50;
MAIN_TUNING.contactMinSamples = 5;
MAIN_TUNING.windDifficulty = 1.4;
MAIN_TUNING.windMeanVelocity = [3.0, 0.8, 0.0];
MAIN_TUNING.windGustStd = [1.5, 1.4, 0.4];
MAIN_TUNING.windMaxSpeed = 7.0;
MAIN_TUNING.windPublishPeriod = 0.1;
MAIN_TUNING.ontologyLandingIntentFloor = 0.45;
MAIN_TUNING.ontologyFinalLandMinAlignment = 0.72;
MAIN_TUNING.ontologyFinalLandMinVisualStability = 0.55;
MAIN_TUNING.ontologyFinalLandMaxWindRisk = 0.78;

algorithmRoot = fileparts(mfilename('fullpath'));
repoRoot = locate_repo_root(algorithmRoot);
projectRoot = repoRoot;
addpath(genpath(algorithmRoot));
legacyMatlabRoot = fullfile(repoRoot, 'matlab');
if isfolder(legacyMatlabRoot)
    addpath(genpath(legacyMatlabRoot));
end

cfg = default_config();
cfg.project.root = repoRoot;
cfg.project.workspaceRoot = repoRoot;
cfg = apply_main_pipeline_config(cfg, MAIN_PIPELINE, MAIN_TUNING);

resultsRoot = fullfile(repoRoot, 'results', 'matlab_logs', 'ontology_landing');
if ~exist(resultsRoot, 'dir')
    mkdir(resultsRoot);
end

runStamp = datestr(now, 'yyyymmdd_HHMMSS');
runDir = fullfile(resultsRoot, runStamp);
if ~exist(runDir, 'dir')
    mkdir(runDir);
end
cfg.results.outputDir = runDir;

fprintf('[main] Project root: %s\n', projectRoot);
fprintf('[main] Results directory: %s\n', runDir);
fprintf('[main] Pipeline: collect=%d trainModel=%d trainOntology=%d validate=%d trace=%d\n', ...
    cfg.pipeline.collectData, cfg.pipeline.trainModel, cfg.pipeline.trainOntologyWeights, ...
    cfg.pipeline.validateModel, cfg.pipeline.runTraceCollector);
fprintf('[main] Wind: difficulty=%.2f mean=[%.2f %.2f %.2f] gustStd=[%.2f %.2f %.2f] max=%.2f publishPeriod=%.2f\n', ...
    cfg.wind.difficulty, cfg.wind.meanVelocity(1), cfg.wind.meanVelocity(2), cfg.wind.meanVelocity(3), ...
    cfg.wind.gustStd(1), cfg.wind.gustStd(2), cfg.wind.gustStd(3), cfg.wind.maxSpeed, cfg.simulator.windPublishPeriod);
save(fullfile(runDir, 'main_pipeline_config.mat'), 'MAIN_PIPELINE', 'MAIN_TUNING', 'cfg');

historicalData = empty_training_data();
previousModel = struct();
historicalSummary = struct('runCount', 0, 'sampleCount', 0, 'sourceFiles', {{}});
if cfg.pipeline.useHistoricalTrainingData && cfg.pipeline.collectData
    [historicalData, previousModel, historicalSummary] = load_historical_training_data(resultsRoot, runDir, cfg);
    fprintf('[main] Historical training data: runs=%d samples=%d\n', ...
        historicalSummary.runCount, historicalSummary.sampleCount);
end

sim = struct();
if cfg.pipeline.collectData
    sim = initSimulator(cfg);

    if isfield(sim, 'launchInfo') && isfield(sim.launchInfo, 'message') && ~isempty(sim.launchInfo.message)
        fprintf('[main] Simulation setup: %s\n', sim.launchInfo.message);
    end
    if isfield(sim, 'isConnected') && sim.isConnected
        fprintf('[main] Simulation namespace: %s\n', sim.namespace);
    end

    if cfg.pipeline.runTraceCollector
        start_runtime_trace_collector(repoRoot, runDir, cfg);
    end
else
    fprintf('[main] Data collection disabled; loading dataset from previous result.\n');
end

featureMatrix = [];
semanticMatrix = [];
rawMatrix = [];
labelVector = [];
windSpeedVector = [];
timeVector = [];
scenarioVector = [];
altitudeVector = [];
rollVector = [];
pitchVector = [];
yawVector = [];
positionErrorVector = [];
decisionOnlineVector = [];
decisionOnlineScore = [];
decisionOnlineConfidence = [];

scenarioLogs = repmat(struct( ...
    'scenarioIndex', [], ...
    'time', [], ...
    'wind', [], ...
    'state', [], ...
    'observation', [], ...
    'semantic', [], ...
    'feature', [], ...
    'onlineDecision', [], ...
    'finalLabel', []), cfg.simulation.numScenarios, 1);

expectedFeatureCount = numel(cfg.featureFusion.rawNames) + numel(cfg.featureFusion.semanticNames);
if is_model_compatible(previousModel, expectedFeatureCount)
    model = previousModel;
    fprintf('[main] Starting online decisions from previous trained model (%d samples).\n', model.sampleCount);
elseif isfield(previousModel, 'isTrained') && previousModel.isTrained
    fprintf('[main] Previous model has %d features, current ontology uses %d; using baseline until retraining.\n', ...
        model_feature_count(previousModel), expectedFeatureCount);
    model = struct();
else
    model = struct();
end
touchdownInfo = default_contact_info();

if cfg.pipeline.collectData
for scenarioIdx = 1:cfg.simulation.numScenarios
    if scenarioIdx > 1
        pause(1.5);
        sim = initSimulator(cfg);
        if isfield(sim, 'launchInfo') && isfield(sim.launchInfo, 'message') && ~isempty(sim.launchInfo.message)
            fprintf('[main] Simulation setup: %s\n', sim.launchInfo.message);
        end
        if isfield(sim, 'isConnected') && sim.isConnected
            fprintf('[main] Simulation namespace: %s\n', sim.namespace);
        end
        if cfg.pipeline.runTraceCollector
            start_runtime_trace_collector(repoRoot, runDir, cfg);
        end
    end

    fprintf('\n[main] Data collection repeat %d / %d\n', scenarioIdx, cfg.simulation.numScenarios);
    sim = resetScenario(sim, cfg);
    sim.currentScenario = scenarioIdx;
    sim = startLandingPadMotion(sim, cfg);
    sim = performTakeoff(sim, cfg);
    sim = releaseLandingPadMotion(sim, cfg);
    windProcess = generate_wind_process(cfg, scenarioIdx);

    scenarioStart = numel(timeVector) + 1;
    scenarioSampleCount = 0;
    scenarioState = struct();
    scenarioObservation = struct();
    scenarioSemantic = struct();
    scenarioFeature = struct();
    scenarioAction = cell(numel(cfg.simulation.timeVector), 1);
    takeoffConfirmed = false;
    scenarioContact = default_contact_info();

    for stepIdx = 1:numel(cfg.simulation.timeVector)
        t = cfg.simulation.timeVector(stepIdx);
        currentWind = windProcess(stepIdx, :);

        [uavState, sim] = read_uav_state(sim, t, currentWind, cfg);
        contactArmingAltitude = max([ ...
            cfg.labeling.contactArmingAltitude, ...
            cfg.simulator.takeoffDetectionAltitude, ...
            cfg.labeling.touchdownGroundAltitude + 0.5]);
        takeoffConfirmed = takeoffConfirmed || uavState.altitude >= contactArmingAltitude;
        [windState, sim] = read_wind_condition(sim, t, currentWind, cfg);
        [observation, sim] = read_landing_observation(sim, uavState, windState, cfg);
        semantic = compute_ontology_features(uavState, windState, observation, cfg);
        fused = fuse_features(observation.continuous, semantic, cfg);

        if is_model_compatible(model, numel(fused.vector))
            [predictedLabel, classProbability, confidence, predictedAction] = predict_gaussian_nb(model, fused.vector);
        else
            [predictedLabel, baselineScore, predictedAction] = threshold_baseline_decision(semantic, cfg);
            classProbability = [1.0 - baselineScore, baselineScore];
            confidence = max(classProbability);
        end

        sim = send_decision_command(sim, predictedAction, classProbability, confidence, cfg, uavState, observation, semantic);

        featureMatrix = [featureMatrix; fused.vector(:)']; %#ok<AGROW>
        semanticMatrix = [semanticMatrix; semantic.vector(:)']; %#ok<AGROW>
        rawMatrix = [rawMatrix; observation.continuous(:)']; %#ok<AGROW>
        timeVector = [timeVector; t]; %#ok<AGROW>
        windSpeedVector = [windSpeedVector; windState.speed]; %#ok<AGROW>
        scenarioVector = [scenarioVector; scenarioIdx]; %#ok<AGROW>
        altitudeVector = [altitudeVector; uavState.altitude]; %#ok<AGROW>
        rollVector = [rollVector; uavState.attitude(1)]; %#ok<AGROW>
        pitchVector = [pitchVector; uavState.attitude(2)]; %#ok<AGROW>
        yawVector = [yawVector; uavState.attitude(3)]; %#ok<AGROW>
        positionErrorVector = [positionErrorVector; observation.positionError]; %#ok<AGROW>
        decisionOnlineVector = [decisionOnlineVector; predictedLabel]; %#ok<AGROW>
        decisionOnlineScore = [decisionOnlineScore; classProbability(2)]; %#ok<AGROW>
        decisionOnlineConfidence = [decisionOnlineConfidence; confidence]; %#ok<AGROW>

        scenarioSampleCount = scenarioSampleCount + 1;
        scenarioState(scenarioSampleCount).uav = uavState; %#ok<AGROW>
        scenarioObservation(scenarioSampleCount).value = observation; %#ok<AGROW>
        scenarioSemantic(scenarioSampleCount).value = semantic; %#ok<AGROW>
        scenarioFeature(scenarioSampleCount).value = fused; %#ok<AGROW>
        scenarioAction{scenarioSampleCount} = struct( ...
            'action', predictedAction, ...
            'probability', classProbability, ...
            'confidence', confidence, ...
            'trajectory', sim.guidance.trajectory);

        if mod(stepIdx, cfg.simulation.logEveryNSteps) == 0
            trajStatus = format_trajectory_status(sim);
            fprintf('[main]   t = %5.1f s | wind = %4.2f m/s | action = %s | traj = %s | conf = %4.2f | %s\n', ...
                t, windState.speed, predictedAction, sim.guidance.mode, confidence, trajStatus);
        end

        contactTimeArmed = t >= get_labeling_value_local(cfg, 'contactMinFlightTime', 0.0);
        contactSampleArmed = stepIdx >= get_labeling_value_local(cfg, 'contactMinSamples', 1);
        contactArmed = takeoffConfirmed && contactTimeArmed && contactSampleArmed;
        if cfg.simulation.stopOnTouchdown
            contact = detect_landing_contact(uavState, observation, cfg, contactArmed);
        else
            contact = default_contact_info();
        end
        if contact.detected
            contact.scenarioIndex = scenarioIdx;
            contact.time = t;
            scenarioContact = contact;
            touchdownInfo = contact;
            sim = stopLandingPadMotion(sim);
            if contact.success
                fprintf('[main] Landing-pad contact detected at t = %.1f s (xy=%.3f, z=%.3f). Immediate DISARM and clean restart.\n', ...
                    t, contact.positionError, contact.altitude);
            else
                fprintf('[main] Ground contact detected after takeoff at t = %.1f s (xy=%.3f, z=%.3f). Marking landing FAILED and clean restart.\n', ...
                    t, contact.positionError, contact.altitude);
            end
            disarmOk = run_mavlink_command('disarm', 0, 0, 0, 4, sim, cfg);
            if contact.success && ~disarmOk
                run_mavlink_command('land_disarm', 0, 0, 0, 6, sim, cfg);
            end
            sim = shutdownSimulator(sim);
            break;
        end
    end

    if scenarioContact.detected
        finalLabel = double(scenarioContact.success);
    else
        finalLabel = label_landing_outcome(scenarioState(end).uav, scenarioObservation(end).value, cfg);
    end
    scenarioRange = scenarioStart:(scenarioStart + scenarioSampleCount - 1);
    labelVector = [labelVector; repmat(finalLabel, scenarioSampleCount, 1)]; %#ok<AGROW>

    scenarioLogs(scenarioIdx).scenarioIndex = scenarioIdx;
    scenarioLogs(scenarioIdx).time = timeVector(scenarioRange);
    scenarioLogs(scenarioIdx).wind = windProcess(1:scenarioSampleCount, :);
    scenarioLogs(scenarioIdx).state = scenarioState;
    scenarioLogs(scenarioIdx).observation = scenarioObservation;
    scenarioLogs(scenarioIdx).semantic = scenarioSemantic;
    scenarioLogs(scenarioIdx).feature = scenarioFeature;
    scenarioLogs(scenarioIdx).onlineDecision = scenarioAction;
    scenarioLogs(scenarioIdx).finalLabel = finalLabel;

    [onlineTrainingFeatures, onlineTrainingLabels] = combined_training_dataset(historicalData, featureMatrix, labelVector);
    if cfg.pipeline.trainModel && numel(unique(onlineTrainingLabels)) >= 2 && numel(onlineTrainingLabels) >= cfg.decision.minTrainingSamples
        model = train_gaussian_nb(onlineTrainingFeatures, onlineTrainingLabels, cfg);
    end

    if cfg.pipeline.shutdownSimulatorOnFinish && ...
            (~isfield(sim, 'runtimeShutdown') || ~sim.runtimeShutdown)
        sim = stopLandingPadMotion(sim);
        sim = shutdownSimulator(sim);
    end
end
else
    [loadedData, loadedModel] = load_pipeline_dataset(cfg, repoRoot);
    featureMatrix = loadedData.featureMatrix;
    semanticMatrix = loadedData.semanticMatrix;
    rawMatrix = loadedData.rawMatrix;
    labelVector = loadedData.labelVector;
    windSpeedVector = loadedData.windSpeedVector;
    timeVector = loadedData.timeVector;
    scenarioVector = loadedData.scenarioVector;
    altitudeVector = loadedData.altitudeVector;
    rollVector = loadedData.rollVector;
    pitchVector = loadedData.pitchVector;
    yawVector = loadedData.yawVector;
    positionErrorVector = loadedData.positionErrorVector;
    decisionOnlineVector = loadedData.decisionOnlineVector;
    decisionOnlineScore = loadedData.decisionOnlineScore;
    decisionOnlineConfidence = loadedData.decisionOnlineConfidence;
    scenarioLogs = loadedData.scenarioLogs;
    touchdownInfo = loadedData.touchdown;
    model = loadedModel;
end

[trainingFeatureMatrix, trainingLabelVector] = combined_training_dataset(historicalData, featureMatrix, labelVector);
trainingSemanticMatrix = semanticMatrix;
if ~isempty(historicalData.semanticMatrix) && ~isempty(semanticMatrix) && ...
        size(historicalData.semanticMatrix, 2) == size(semanticMatrix, 2)
    trainingSemanticMatrix = [historicalData.semanticMatrix; semanticMatrix];
elseif isempty(semanticMatrix) && ~isempty(historicalData.semanticMatrix)
    trainingSemanticMatrix = historicalData.semanticMatrix;
end
if size(trainingSemanticMatrix, 1) ~= numel(trainingLabelVector)
    trainingSemanticMatrix = semanticMatrix;
end

if cfg.pipeline.trainModel
    if numel(unique(trainingLabelVector)) >= 2 && numel(trainingLabelVector) >= cfg.decision.minTrainingSamples
        model = train_gaussian_nb(trainingFeatureMatrix, trainingLabelVector, cfg);
    elseif ~isfield(model, 'isTrained') || ~model.isTrained
        fprintf('[main] Model training skipped: need at least %d samples and both labels; got %d samples, %d class(es).\n', ...
            cfg.decision.minTrainingSamples, numel(trainingLabelVector), numel(unique(trainingLabelVector)));
        model = struct('isTrained', false);
    end
elseif ~cfg.pipeline.trainModel
    model = load_pretrained_model_if_needed(model, cfg);
end

ontologyPolicy = struct();
if cfg.pipeline.trainOntologyWeights
    ontologyPolicy = train_ontology_weight_artifact(trainingSemanticMatrix, trainingLabelVector, cfg, runDir);
end

if cfg.pipeline.validateModel
    [proposedLabels, proposedProbabilities, proposedConfidence, proposedActions] = predict_gaussian_nb(model, featureMatrix);
    [baselineLabels, baselineScore, baselineActions] = threshold_baseline_decision(semanticMatrix, cfg); %#ok<ASGLU>
    results = evaluate_landing_decisions(labelVector, proposedLabels, baselineLabels, windSpeedVector, cfg);
    previousResults = evaluate_previous_model(previousModel, labelVector, featureMatrix, baselineLabels, windSpeedVector, cfg);
else
    fprintf('[main] Validation disabled; saving collected/trained artifacts only.\n');
    proposedLabels = [];
    proposedProbabilities = [];
    proposedConfidence = [];
    proposedActions = {};
    baselineLabels = [];
    baselineActions = {};
    results = empty_validation_results();
    previousResults = struct();
end
results.model = model;
results.previousModel = previousModel;
results.previousModelEvaluation = previousResults;
results.ontologyPolicy = ontologyPolicy;
results.trainingSampleCount = numel(trainingLabelVector);
results.currentRunSampleCount = numel(labelVector);
results.historicalRunCount = historicalSummary.runCount;
results.historicalSampleCount = historicalSummary.sampleCount;
results.historicalSourceFiles = historicalSummary.sourceFiles;
results.proposedProbabilities = proposedProbabilities;
results.proposedConfidence = proposedConfidence;
results.proposedActions = proposedActions;
results.proposedLabels = proposedLabels;
results.trueLabels = labelVector;
results.baselineLabels = baselineLabels;
results.baselineActions = baselineActions;
results.featureMatrix = featureMatrix;
results.semanticMatrix = semanticMatrix;
results.rawMatrix = rawMatrix;
results.timeVector = timeVector;
results.windSpeedVector = windSpeedVector;
results.scenarioVector = scenarioVector;
results.altitudeVector = altitudeVector;
results.rollVector = rollVector;
results.pitchVector = pitchVector;
results.yawVector = yawVector;
results.positionErrorVector = positionErrorVector;
results.decisionOnlineVector = decisionOnlineVector;
results.decisionOnlineScore = decisionOnlineScore;
results.decisionOnlineConfidence = decisionOnlineConfidence;
results.touchdown = touchdownInfo;
results.completedScenarioCount = numel(unique(scenarioVector));
results.sampleCount = numel(labelVector);
results.pathGeneration = evaluate_path_generation(scenarioLogs, cfg);
results.reinforcementTraining = build_reinforcement_training_data(scenarioLogs, results.pathGeneration, cfg);

save(fullfile(runDir, 'landing_framework_results.mat'), 'cfg', 'results', 'scenarioLogs');

summaryPath = fullfile(runDir, 'summary.txt');
fid = fopen(summaryPath, 'w');
fprintf(fid, 'Ontology-based probabilistic landing framework\n');
fprintf(fid, 'Run directory: %s\n', runDir);
fprintf(fid, 'Completed scenarios evaluated: %d / %d\n', results.completedScenarioCount, cfg.simulation.numScenarios);
fprintf(fid, 'Samples evaluated: %d\n', results.sampleCount);
fprintf(fid, 'Training samples used: %d\n', results.trainingSampleCount);
fprintf(fid, 'Historical runs used: %d\n', results.historicalRunCount);
fprintf(fid, 'Historical samples used: %d\n', results.historicalSampleCount);
fprintf(fid, 'Touchdown detected: %d\n', results.touchdown.detected);
if results.touchdown.detected
    fprintf(fid, 'Touchdown type: %s\n', results.touchdown.type);
    fprintf(fid, 'Touchdown success: %d\n', results.touchdown.success);
    fprintf(fid, 'Touchdown source: %s\n', results.touchdown.source);
    fprintf(fid, 'Touchdown scenario: %d\n', results.touchdown.scenarioIndex);
    fprintf(fid, 'Touchdown time: %.2f s\n', results.touchdown.time);
    fprintf(fid, 'Touchdown altitude: %.4f m\n', results.touchdown.altitude);
    fprintf(fid, 'Touchdown position error: %.4f m\n', results.touchdown.positionError);
    fprintf(fid, 'Touchdown vertical speed: %.4f m/s\n', results.touchdown.verticalSpeed);
end
fprintf(fid, 'Accuracy (proposed): %.4f\n', results.proposed.accuracy);
fprintf(fid, 'Accuracy (baseline): %.4f\n', results.baseline.accuracy);
if isfield(results.previousModelEvaluation, 'isAvailable') && results.previousModelEvaluation.isAvailable
    fprintf(fid, 'Accuracy (previous model on current run): %.4f\n', results.previousModelEvaluation.metrics.proposed.accuracy);
    fprintf(fid, 'Accuracy gain vs previous model: %.4f\n', ...
        results.proposed.accuracy - results.previousModelEvaluation.metrics.proposed.accuracy);
end
fprintf(fid, 'Unsafe landing attempt rate (proposed): %.4f\n', results.proposed.unsafeLandingAttemptRate);
fprintf(fid, 'Unsafe landing attempt rate (baseline): %.4f\n', results.baseline.unsafeLandingAttemptRate);
if isfield(results, 'pathGeneration') && isfield(results.pathGeneration, 'summary')
    pathSummary = results.pathGeneration.summary;
    fprintf(fid, 'Path mean tracking error: %.4f m\n', pathSummary.meanTrackingError);
    fprintf(fid, 'Path final tracking error: %.4f m\n', pathSummary.finalTrackingError);
    fprintf(fid, 'Path efficiency: %.4f\n', pathSummary.pathEfficiency);
    fprintf(fid, 'Target smoothness: %.4f m/s^2\n', pathSummary.targetSmoothness);
    fprintf(fid, 'Command coverage: %.4f\n', pathSummary.commandCoverage);
    fprintf(fid, 'Marker coverage: %.4f\n', pathSummary.markerCoverage);
end
if isfield(results, 'reinforcementTraining') && isfield(results.reinforcementTraining, 'summary')
    rlSummary = results.reinforcementTraining.summary;
    fprintf(fid, 'RL reward mean return: %.4f\n', rlSummary.meanReturn);
    fprintf(fid, 'RL reward mean smoothness score: %.4f\n', rlSummary.meanSmoothnessScore);
    fprintf(fid, 'RL reward mean target smoothness: %.4f m/s^2\n', rlSummary.meanTargetSmoothness);
end
fclose(fid);

metricsTable = table( ...
    results.completedScenarioCount, results.sampleCount, double(results.touchdown.detected), ...
    results.trainingSampleCount, results.historicalRunCount, results.historicalSampleCount, ...
    double(results.touchdown.success), results.touchdown.scenarioIndex, results.touchdown.time, results.touchdown.altitude, results.touchdown.positionError, results.touchdown.verticalSpeed, ...
    results.proposed.accuracy, results.proposed.precision, results.proposed.recall, results.proposed.f1Score, results.proposed.unsafeLandingAttemptRate, ...
    results.baseline.accuracy, results.baseline.precision, results.baseline.recall, results.baseline.f1Score, results.baseline.unsafeLandingAttemptRate, ...
    'VariableNames', {'completed_scenarios', 'sample_count', 'touchdown_detected', ...
    'training_sample_count', 'historical_run_count', 'historical_sample_count', ...
    'touchdown_success', 'touchdown_scenario', 'touchdown_time_s', 'touchdown_altitude_m', 'touchdown_position_error_m', 'touchdown_vertical_speed_mps', ...
    'proposed_accuracy', 'proposed_precision', 'proposed_recall', 'proposed_f1', 'proposed_unsafe_attempt_rate', ...
    'baseline_accuracy', 'baseline_precision', 'baseline_recall', 'baseline_f1', 'baseline_unsafe_attempt_rate'});
writetable(metricsTable, fullfile(runDir, 'metrics.csv'));
write_path_metrics(results.pathGeneration, runDir);
write_reinforcement_rewards(results.reinforcementTraining, runDir);

if cfg.pipeline.validateModel && cfg.pipeline.savePlots
    plot_landing_results(results, cfg, runDir);
end
if cfg.pipeline.collectData && cfg.pipeline.shutdownSimulatorOnFinish && ~isempty(fieldnames(sim))
    sim = shutdownSimulator(sim);
end

fprintf('\n[main] Completed successfully.\n');
fprintf('[main] Results saved to %s\n', runDir);

% Run automatic trace analysis and tuning suggestion if possible
if cfg.pipeline.runAutoTuningAnalysis
try
    analyze_py = fullfile(repoRoot, 'scripts', 'analyze_trace_and_suggest_tuning.py');
    if exist(analyze_py, 'file')
        fprintf('[main] Running automatic trace analysis...\n');
        analyze_cmd = sprintf('python3 "%s" "%s" > "%s" 2>&1', analyze_py, runDir, fullfile(runDir, 'auto_tuning.txt'));
        status = system(analyze_cmd);
        if status == 0
            fprintf('[main] Analysis complete; results: %s\n', fullfile(runDir, 'auto_tuning.txt'));
        else
            fprintf('[main] Analysis script exited with status %d; see %s\n', status, fullfile(runDir, 'auto_tuning.txt'));
        end
    else
        fprintf('[main] Analysis script not found: %s\n', analyze_py);
    end
catch ME
    fprintf('[main] Failed to run analysis: %s\n', ME.message);
end
end

function cfg = apply_main_pipeline_config(cfg, pipeline, tuning)
cfg.pipeline = pipeline;
cfg.pipeline.inputResultsMat = char(string(cfg.pipeline.inputResultsMat));
cfg.pipeline.pretrainedModelMat = char(string(cfg.pipeline.pretrainedModelMat));
if ~isfield(cfg.pipeline, 'useHistoricalTrainingData')
    cfg.pipeline.useHistoricalTrainingData = false;
end
if ~isfield(cfg.pipeline, 'maxHistoricalRuns') || isempty(cfg.pipeline.maxHistoricalRuns)
    cfg.pipeline.maxHistoricalRuns = 20;
end
cfg.pipeline.maxHistoricalRuns = max(0, round(double(cfg.pipeline.maxHistoricalRuns)));

if isfield(tuning, 'dataCollectionRepeats') && ~isempty(tuning.dataCollectionRepeats)
    cfg.simulation.numScenarios = max(1, round(tuning.dataCollectionRepeats));
else
    cfg.simulation.numScenarios = max(1, round(tuning.numScenarios));
end
cfg.simulation.duration = tuning.duration;
cfg.simulation.dt = tuning.dt;
cfg.simulation.timeVector = 0:cfg.simulation.dt:cfg.simulation.duration;
cfg.simulation.stopOnTouchdown = tuning.stopOnTouchdown;
if isfield(tuning, 'resetMethod') && ~isempty(tuning.resetMethod)
    cfg.simulation.resetMethod = lower(strtrim(char(string(tuning.resetMethod))));
end
if isfield(tuning, 'softResetRequest') && ~isempty(tuning.softResetRequest)
    cfg.simulator.reset.request = lower(strtrim(char(string(tuning.softResetRequest))));
end
if isfield(tuning, 'softResetSettleSeconds') && ~isempty(tuning.softResetSettleSeconds)
    cfg.simulator.reset.settleSeconds = max(0.1, double(tuning.softResetSettleSeconds));
end
if isfield(tuning, 'softResetPoseSampleTimeoutSeconds') && ~isempty(tuning.softResetPoseSampleTimeoutSeconds)
    cfg.simulator.reset.poseSampleTimeoutSeconds = max(0.3, double(tuning.softResetPoseSampleTimeoutSeconds));
end
if isfield(tuning, 'softResetRosCleanupGraceSeconds') && ~isempty(tuning.softResetRosCleanupGraceSeconds)
    cfg.simulator.reset.rosCleanupGraceSeconds = max(0.1, double(tuning.softResetRosCleanupGraceSeconds));
end
if isfield(tuning, 'softResetPoseToleranceDrone') && ~isempty(tuning.softResetPoseToleranceDrone)
    cfg.simulator.reset.models(1).tolerance = max(0.01, double(tuning.softResetPoseToleranceDrone));
end
if isfield(tuning, 'softResetPoseToleranceRover') && ~isempty(tuning.softResetPoseToleranceRover)
    cfg.simulator.reset.models(2).tolerance = max(0.01, double(tuning.softResetPoseToleranceRover));
end

cfg.visualization.showOntologyGraph = tuning.showOntologyGraph;
cfg.visualization.visible = tuning.visualizationVisible;
cfg.visualization.saveFigures = pipeline.savePlots;

cfg.simulator.takeoffAltitude = tuning.takeoffAltitude;
cfg.simulator.landingPadMotion.amplitude = tuning.padMotionAmplitude;
cfg.simulator.landingPadMotion.frequency = tuning.padMotionFrequency;
cfg.simulator.landingPadMotion.speed = tuning.padMotionSpeed;

cfg.guidance.directLandRadius = tuning.directLandRadius;
cfg.guidance.directLandAltitudeAbovePad = tuning.directLandAltitudeAbovePad;
cfg.guidance.directLandStableTime = tuning.directLandStableTime;
cfg.guidance.directLandMaxHorizontalSpeed = tuning.directLandMaxHorizontalSpeed;
if isfield(tuning, 'contactArmingAltitude') && ~isempty(tuning.contactArmingAltitude)
    cfg.labeling.contactArmingAltitude = tuning.contactArmingAltitude;
end
if isfield(tuning, 'contactMinFlightTime') && ~isempty(tuning.contactMinFlightTime)
    cfg.labeling.contactMinFlightTime = max(0.0, double(tuning.contactMinFlightTime));
end
if isfield(tuning, 'contactMinSamples') && ~isempty(tuning.contactMinSamples)
    cfg.labeling.contactMinSamples = max(1, round(double(tuning.contactMinSamples)));
end
cfg = apply_main_wind_config(cfg, tuning);
cfg.guidance.ontologyLandingIntentFloor = tuning.ontologyLandingIntentFloor;
cfg.guidance.ontologyFinalLandMinAlignment = tuning.ontologyFinalLandMinAlignment;
cfg.guidance.ontologyFinalLandMinVisualStability = tuning.ontologyFinalLandMinVisualStability;
cfg.guidance.ontologyFinalLandMaxWindRisk = tuning.ontologyFinalLandMaxWindRisk;
end

function cfg = apply_main_wind_config(cfg, tuning)
difficulty = 1.0;
if isfield(tuning, 'windDifficulty') && ~isempty(tuning.windDifficulty)
    difficulty = max(0.0, double(tuning.windDifficulty));
end

if isfield(tuning, 'windMeanVelocity') && numel(tuning.windMeanVelocity) == 3
    cfg.wind.meanVelocity = double(tuning.windMeanVelocity(:))' * difficulty;
else
    cfg.wind.meanVelocity = cfg.wind.meanVelocity * difficulty;
end

if isfield(tuning, 'windGustStd') && numel(tuning.windGustStd) == 3
    cfg.wind.gustStd = double(tuning.windGustStd(:))' * difficulty;
else
    cfg.wind.gustStd = cfg.wind.gustStd * difficulty;
end

if isfield(tuning, 'windMaxSpeed') && ~isempty(tuning.windMaxSpeed)
    cfg.wind.maxSpeed = max(0.1, double(tuning.windMaxSpeed) * max(difficulty, 1.0));
else
    cfg.wind.maxSpeed = cfg.wind.maxSpeed * max(difficulty, 1.0);
end
cfg.wind.accelerationScale = cfg.wind.accelerationScale * difficulty;
cfg.wind.difficulty = difficulty;

if isfield(tuning, 'windPublishPeriod') && ~isempty(tuning.windPublishPeriod)
    cfg.simulator.windPublishPeriod = max(0.05, double(tuning.windPublishPeriod));
end
end

function contact = default_contact_info()
contact = struct( ...
    'detected', false, ...
    'type', 'none', ...
    'success', false, ...
    'source', 'none', ...
    'scenarioIndex', NaN, ...
    'time', NaN, ...
    'altitude', NaN, ...
    'positionError', NaN, ...
    'verticalSpeed', NaN);
end

function start_runtime_trace_collector(repoRoot, runDir, cfg)
try
    collect_sh = fullfile(repoRoot, 'scripts', 'collect_runtime_trace.sh');
    collect_duration = ceil(cfg.simulation.duration + 10);
    collect_cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
        'export ROS_DOMAIN_ID=%d; bash "%s" %d "%s" >/tmp/obdm_runtime_trace_collector.log 2>&1 &'''], ...
        cfg.simulation.rosDomainId, collect_sh, collect_duration, runDir);
    fprintf('[main] Launching runtime trace collector (approx %d s)\n', collect_duration);
    system(collect_cmd);
catch ME
    fprintf('[main] Failed to start trace collector: %s\n', ME.message);
end
end

function [data, model] = load_pipeline_dataset(cfg, repoRoot)
inputPath = cfg.pipeline.inputResultsMat;
if isempty(inputPath)
    inputPath = latest_result_mat(fullfile(repoRoot, 'results', 'matlab_logs', 'ontology_landing'));
end
if isempty(inputPath) || ~exist(inputPath, 'file')
    error('[main] collectData=false but no previous result MAT was found.');
end
fprintf('[main] Loading pipeline input: %s\n', inputPath);
loaded = load(inputPath);
if ~isfield(loaded, 'results')
    error('[main] Input MAT does not contain a results struct: %s', inputPath);
end
results = loaded.results;
data = struct();
data.featureMatrix = field_or_default(results, 'featureMatrix', []);
data.semanticMatrix = field_or_default(results, 'semanticMatrix', []);
data.rawMatrix = field_or_default(results, 'rawMatrix', []);
data.labelVector = field_or_default(results, 'trueLabels', []);
data.windSpeedVector = field_or_default(results, 'windSpeedVector', []);
data.timeVector = field_or_default(results, 'timeVector', []);
data.scenarioVector = field_or_default(results, 'scenarioVector', []);
data.altitudeVector = field_or_default(results, 'altitudeVector', []);
data.rollVector = field_or_default(results, 'rollVector', []);
data.pitchVector = field_or_default(results, 'pitchVector', []);
data.yawVector = field_or_default(results, 'yawVector', []);
data.positionErrorVector = field_or_default(results, 'positionErrorVector', []);
data.decisionOnlineVector = field_or_default(results, 'decisionOnlineVector', []);
data.decisionOnlineScore = field_or_default(results, 'decisionOnlineScore', []);
data.decisionOnlineConfidence = field_or_default(results, 'decisionOnlineConfidence', []);
data.touchdown = normalize_contact_info(field_or_default(results, 'touchdown', default_contact_info()));
if isfield(loaded, 'scenarioLogs')
    data.scenarioLogs = loaded.scenarioLogs;
else
    data.scenarioLogs = struct();
end
model = field_or_default(results, 'model', struct());
end

function data = empty_training_data()
data = struct();
data.featureMatrix = [];
data.semanticMatrix = [];
data.labelVector = [];
end

function [data, previousModel, summary] = load_historical_training_data(resultsRoot, currentRunDir, cfg)
data = empty_training_data();
previousModel = struct();
summary = struct('runCount', 0, 'sampleCount', 0, 'sourceFiles', {{}});
expectedFeatureCount = numel(cfg.featureFusion.rawNames) + numel(cfg.featureFusion.semanticNames);
expectedSemanticCount = numel(cfg.featureFusion.semanticNames);

if cfg.pipeline.maxHistoricalRuns <= 0 || ~exist(resultsRoot, 'dir')
    return;
end

files = dir(fullfile(resultsRoot, '*', 'landing_framework_results.mat'));
if isempty(files)
    return;
end

[~, order] = sort([files.datenum], 'descend');
files = files(order);
currentRunDir = char(string(currentRunDir));

for idx = 1:numel(files)
    resultPath = fullfile(files(idx).folder, files(idx).name);
    if strcmp(files(idx).folder, currentRunDir)
        continue;
    end
    try
        loaded = load(resultPath);
    catch
        continue;
    end
    if ~isfield(loaded, 'results')
        continue;
    end

    result = loaded.results;
    features = field_or_default(result, 'featureMatrix', []);
    labels = field_or_default(result, 'trueLabels', []);
    semantics = field_or_default(result, 'semanticMatrix', []);
    if isempty(features) || isempty(labels) || size(features, 1) ~= numel(labels)
        continue;
    end
    if size(features, 2) ~= expectedFeatureCount
        fprintf('[main] Skipping historical run with %d features; current ontology expects %d: %s\n', ...
            size(features, 2), expectedFeatureCount, resultPath);
        continue;
    end
    if ~isempty(semantics) && size(semantics, 1) ~= numel(labels)
        semantics = [];
    elseif ~isempty(semantics) && size(semantics, 2) ~= expectedSemanticCount
        semantics = [];
    end

    data.featureMatrix = [data.featureMatrix; features]; %#ok<AGROW>
    data.labelVector = [data.labelVector; labels(:)]; %#ok<AGROW>
    if isempty(semantics)
        data.semanticMatrix = [];
    elseif ~isempty(data.semanticMatrix) || summary.runCount == 0
        data.semanticMatrix = [data.semanticMatrix; semantics]; %#ok<AGROW>
    end
    summary.runCount = summary.runCount + 1;
    summary.sampleCount = numel(data.labelVector);
    summary.sourceFiles{end+1} = resultPath; %#ok<AGROW>

    if (~isfield(previousModel, 'isTrained') || ~previousModel.isTrained) && ...
            isfield(result, 'model') && is_model_compatible(result.model, expectedFeatureCount)
        previousModel = result.model;
    end

    if summary.runCount >= cfg.pipeline.maxHistoricalRuns
        break;
    end
end
end

function [trainingFeatureMatrix, trainingLabelVector] = combined_training_dataset(historicalData, currentFeatureMatrix, currentLabelVector)
trainingFeatureMatrix = currentFeatureMatrix;
trainingLabelVector = currentLabelVector(:);
if ~isstruct(historicalData) || ~isfield(historicalData, 'featureMatrix') || ...
        isempty(historicalData.featureMatrix) || ~isfield(historicalData, 'labelVector') || ...
        size(historicalData.featureMatrix, 1) ~= numel(historicalData.labelVector)
    return;
end
if isempty(currentFeatureMatrix)
    trainingFeatureMatrix = historicalData.featureMatrix;
    trainingLabelVector = historicalData.labelVector(:);
elseif size(historicalData.featureMatrix, 2) == size(currentFeatureMatrix, 2)
    trainingFeatureMatrix = [historicalData.featureMatrix; currentFeatureMatrix];
    trainingLabelVector = [historicalData.labelVector(:); currentLabelVector(:)];
end
end

function previousResults = evaluate_previous_model(previousModel, trueLabels, featureMatrix, baselineLabels, windSpeedVector, cfg)
previousResults = struct('isAvailable', false);
if ~isfield(previousModel, 'isTrained') || ~previousModel.isTrained || isempty(featureMatrix) || isempty(trueLabels)
    return;
end
if ~is_model_compatible(previousModel, size(featureMatrix, 2))
    previousResults.reason = sprintf('feature_count_mismatch_previous_%d_current_%d', ...
        model_feature_count(previousModel), size(featureMatrix, 2));
    return;
end
[previousLabels, previousProbabilities, previousConfidence, previousActions] = predict_gaussian_nb(previousModel, featureMatrix);
previousResults.isAvailable = true;
previousResults.labels = previousLabels;
previousResults.probabilities = previousProbabilities;
previousResults.confidence = previousConfidence;
previousResults.actions = previousActions;
previousResults.metrics = evaluate_landing_decisions(trueLabels, previousLabels, baselineLabels, windSpeedVector, cfg);
end

function write_path_metrics(pathGeneration, runDir)
if ~isstruct(pathGeneration) || ~isfield(pathGeneration, 'scenario') || isempty(pathGeneration.scenario)
    return;
end
scenarioMetrics = pathGeneration.scenario;
metricFields = {'scenarioIndex', 'sampleCount', 'duration', 'commandCoverage', 'markerCoverage', ...
    'meanTrackingError', 'medianTrackingError', 'maxTrackingError', 'finalTrackingError', ...
    'minTrackingError', 'uavPathLength', 'targetPathLength', 'padPathLength', ...
    'straightDistance', 'pathEfficiency', 'targetPathEfficiency', 'targetSmoothness', ...
    'meanApproachStep', 'maxApproachStep', 'meanMaxStep', 'meanNavigationGain', ...
    'meanTrackQuality', 'meanRisk', 'meanDescentAuthority', 'timeToFirstMarker', ...
    'timeToCaptureRadius', 'timeToFinalRadius', 'finalLandCommanded', ...
    'timeToFinalLandCommand', 'successLabel'};
rows = numel(scenarioMetrics);
values = nan(rows, numel(metricFields));
for row = 1:rows
    for col = 1:numel(metricFields)
        values(row, col) = double(scenarioMetrics(row).(metricFields{col}));
    end
end
pathTable = array2table(values, 'VariableNames', metricFields);
writetable(pathTable, fullfile(runDir, 'path_generation_metrics.csv'));
end

function write_reinforcement_rewards(rlData, runDir)
if ~isstruct(rlData) || ~isfield(rlData, 'episodes') || isempty(rlData.episodes)
    return;
end
rows = 0;
for epIdx = 1:numel(rlData.episodes)
    rows = rows + numel(rlData.episodes(epIdx).reward);
end
scenarioIndex = nan(rows, 1);
time = nan(rows, 1);
reward = nan(rows, 1);
tracking = nan(rows, 1);
smoothness = nan(rows, 1);
targetSmoothness = nan(rows, 1);
efficiency = nan(rows, 1);
coverage = nan(rows, 1);
trackQuality = nan(rows, 1);
risk = nan(rows, 1);
terminal = nan(rows, 1);
action = strings(rows, 1);
cursor = 0;
for epIdx = 1:numel(rlData.episodes)
    episode = rlData.episodes(epIdx);
    count = numel(episode.reward);
    idx = cursor + (1:count);
    scenarioIndex(idx) = episode.scenarioIndex;
    time(idx) = episode.time(:);
    reward(idx) = episode.reward(:);
    tracking(idx) = episode.components.tracking(:);
    smoothness(idx) = episode.components.smoothness(:);
    targetSmoothness(idx) = episode.components.targetSmoothness(:);
    efficiency(idx) = episode.components.efficiency(:);
    coverage(idx) = episode.components.coverage(:);
    trackQuality(idx) = episode.components.trackQuality(:);
    risk(idx) = episode.components.risk(:);
    terminal(idx) = episode.components.terminal(:);
    action(idx) = string(episode.actions(:));
    cursor = cursor + count;
end
rewardTable = table(scenarioIndex, time, action, reward, tracking, smoothness, targetSmoothness, ...
    efficiency, coverage, trackQuality, risk, terminal, ...
    'VariableNames', {'scenario_index', 'time_s', 'action', 'reward', 'tracking_score', ...
    'smoothness_score', 'target_smoothness_mps2', 'efficiency_score', 'coverage_score', ...
    'track_quality_score', 'risk_score', 'terminal_reward'});
writetable(rewardTable, fullfile(runDir, 'reinforcement_training_rewards.csv'));
end

function tf = is_model_compatible(model, featureCount)
tf = isfield(model, 'isTrained') && model.isTrained && ...
    model_feature_count(model) == featureCount;
end

function count = model_feature_count(model)
count = NaN;
if isfield(model, 'featureCount') && ~isempty(model.featureCount)
    count = model.featureCount;
elseif isfield(model, 'mean') && ~isempty(model.mean)
    count = size(model.mean, 2);
end
end

function path = latest_result_mat(resultsRoot)
path = '';
if ~exist(resultsRoot, 'dir')
    return;
end
files = dir(fullfile(resultsRoot, '*', 'landing_framework_results.mat'));
if isempty(files)
    return;
end
[~, idx] = max([files.datenum]);
path = fullfile(files(idx).folder, files(idx).name);
end

function model = load_pretrained_model_if_needed(model, cfg)
if isfield(model, 'isTrained') && model.isTrained
    return;
end
if isempty(cfg.pipeline.pretrainedModelMat)
    if cfg.pipeline.validateModel
        error('[main] trainModel=false and no trained/pretrained model is available for validation.');
    end
    return;
end
loaded = load(cfg.pipeline.pretrainedModelMat);
if isfield(loaded, 'model')
    model = loaded.model;
elseif isfield(loaded, 'results') && isfield(loaded.results, 'model')
    model = loaded.results.model;
else
    error('[main] Pretrained model MAT does not contain model or results.model: %s', cfg.pipeline.pretrainedModelMat);
end
end

function value = field_or_default(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && ~isempty(item.(fieldName))
    value = item.(fieldName);
end
end

function value = get_labeling_value_local(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'labeling') && isfield(cfg.labeling, fieldName) && ~isempty(cfg.labeling.(fieldName))
    value = cfg.labeling.(fieldName);
end
end

function contact = normalize_contact_info(contact)
defaultContact = default_contact_info();
fields = fieldnames(defaultContact);
for idx = 1:numel(fields)
    fieldName = fields{idx};
    if ~isfield(contact, fieldName) || isempty(contact.(fieldName))
        contact.(fieldName) = defaultContact.(fieldName);
    end
end
end

function results = empty_validation_results()
emptyMetrics = struct('accuracy', NaN, 'precision', NaN, 'recall', NaN, 'f1Score', NaN, ...
    'unsafeLandingAttemptRate', NaN);
results = struct();
results.proposed = emptyMetrics;
results.baseline = emptyMetrics;
results.windBins = [];
end

function policy = train_ontology_weight_artifact(semanticMatrix, labelVector, cfg, runDir)
policy = default_ontology_policy();
if isempty(semanticMatrix) || isempty(labelVector) || numel(unique(labelVector)) < 2
    fprintf('[main] Ontology weight training skipped: not enough labeled semantic data.\n');
    write_ontology_policy(policy, runDir);
    return;
end

names = cfg.featureFusion.semanticNames;
idx = struct();
for i = 1:numel(names)
    idx.(names{i}) = i;
end

successMask = labelVector(:) == cfg.evaluation.positiveClassLabel;
failMask = ~successMask;
if ~any(successMask) || ~any(failMask)
    fprintf('[main] Ontology weight training skipped: only one outcome class present.\n');
    write_ontology_policy(policy, runDir);
    return;
end

S = semanticMatrix;
riskFeatures = [ ...
    S(:, idx.windRisk), ...
    S(:, idx.dragRisk), ...
    1.0 - S(:, idx.energyEfficiency), ...
    1.0 - S(:, idx.droneMotionStability), ...
    1.0 - S(:, idx.roverMotionStability), ...
    1.0 - S(:, idx.visualStability)];
trackFeatures = [ ...
    S(:, idx.alignmentState), ...
    S(:, idx.relativeDistanceState), ...
    S(:, idx.visualStability), ...
    S(:, idx.droneMotionStability), ...
    S(:, idx.timeEfficiency)];
intentFeatures = [ ...
    S(:, idx.alignmentState), ...
    S(:, idx.relativeDistanceState), ...
    S(:, idx.altitudeState), ...
    S(:, idx.visualStability), ...
    S(:, idx.energyEfficiency), ...
    1.0 - S(:, idx.windRisk)];
descentFeatures = [ ...
    S(:, idx.relativeDistanceState), ...
    S(:, idx.altitudeState), ...
    S(:, idx.visualStability), ...
    S(:, idx.droneMotionStability), ...
    S(:, idx.roverMotionStability), ...
    S(:, idx.energyEfficiency), ...
    1.0 - S(:, idx.windRisk), ...
    1.0 - S(:, idx.dragRisk)];

policy.riskWeights = normalize_positive(mean(riskFeatures(failMask, :), 1) - mean(riskFeatures(successMask, :), 1));
policy.trackQualityWeights = normalize_positive(mean(trackFeatures(successMask, :), 1) - mean(trackFeatures(failMask, :), 1));
policy.landingIntentWeights = normalize_positive(mean(intentFeatures(successMask, :), 1) - mean(intentFeatures(failMask, :), 1));
policy.descentAuthorityExponents = 0.5 + 1.5 * normalize_positive( ...
    mean(descentFeatures(successMask, :), 1) - mean(descentFeatures(failMask, :), 1));
policy.trainingMethod = 'semantic_contrast_warm_start';
policy.sampleCount = numel(labelVector);
policy.successCount = nnz(successMask);
policy.failureCount = nnz(failMask);

write_ontology_policy(policy, runDir);
fprintf('[main] Ontology weight artifact saved: %s\n', fullfile(runDir, 'ontology_policy.json'));
end

function policy = default_ontology_policy()
policy = struct();
policy.trainingMethod = 'default';
policy.riskFeatureNames = {'WindRisk', 'DragRisk', 'LowEnergyEfficiency', 'LowDroneMotionStability', 'LowRoverMotionStability', 'LowVisualStability'};
policy.riskWeights = [0.25, 0.20, 0.18, 0.14, 0.13, 0.10];
policy.trackQualityFeatureNames = {'AlignmentState', 'RelativeDistanceState', 'VisualStability', 'DroneMotionStability', 'TimeEfficiency'};
policy.trackQualityWeights = [0.30, 0.25, 0.25, 0.15, 0.05];
policy.landingIntentFeatureNames = {'AlignmentState', 'RelativeDistanceState', 'AltitudeState', 'VisualStability', 'EnergyEfficiency', 'LowWindRisk'};
policy.landingIntentWeights = [0.25, 0.22, 0.18, 0.18, 0.10, 0.07];
policy.descentAuthorityFeatureNames = {'RelativeDistanceState', 'AltitudeState', 'VisualStability', 'DroneMotionStability', 'RoverMotionStability', 'EnergyEfficiency', 'LowWindRisk', 'LowDragRisk'};
policy.descentAuthorityExponents = ones(1, 8);
policy.modeParameters = struct('padCaptureRadius', 3.0, 'finalDescentRadius', 0.9, ...
    'directLandRadius', 0.35, 'directLandAltitudeAbovePad', 0.45);
policy.sampleCount = 0;
policy.successCount = 0;
policy.failureCount = 0;
end

function weights = normalize_positive(raw)
raw = max(double(raw), 0.0);
if sum(raw) <= eps
    weights = ones(size(raw)) ./ max(numel(raw), 1);
else
    weights = raw ./ sum(raw);
end
end

function write_ontology_policy(policy, runDir)
save(fullfile(runDir, 'ontology_policy.mat'), 'policy');
fid = fopen(fullfile(runDir, 'ontology_policy.json'), 'w');
if fid < 0
    return;
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', jsonencode(policy, 'PrettyPrint', true));
end

function text = format_trajectory_status(sim)
text = 'pad=none';
if ~isfield(sim, 'guidance') || ~isfield(sim.guidance, 'trajectory')
    return;
end
trajectory = sim.guidance.trajectory;
parts = {};
if isfield(trajectory, 'distanceToMarker')
    parts{end+1} = sprintf('pad_dist=%.2f', trajectory.distanceToMarker); %#ok<AGROW>
end
if isfield(trajectory, 'padEkfSource')
    parts{end+1} = sprintf('ekf=%s', trajectory.padEkfSource); %#ok<AGROW>
end
if isfield(trajectory, 'hasMarker')
    parts{end+1} = sprintf('hasMarker=%d', trajectory.hasMarker); %#ok<AGROW>
end
if isfield(trajectory, 'approachStep')
    parts{end+1} = sprintf('step=%.2f', trajectory.approachStep); %#ok<AGROW>
end
if isfield(trajectory, 'maxHorizontalStep')
    parts{end+1} = sprintf('maxStep=%.2f', trajectory.maxHorizontalStep); %#ok<AGROW>
end
if isfield(trajectory, 'target') && numel(trajectory.target) >= 3
    target = trajectory.target;
    parts{end+1} = sprintf('target=[%.2f %.2f %.2f]', target(1), target(2), target(3)); %#ok<AGROW>
end
if isfield(sim.guidance, 'localTarget') && numel(sim.guidance.localTarget) >= 3
    localTarget = sim.guidance.localTarget;
    parts{end+1} = sprintf('local=[%.2f %.2f %.2f]', localTarget(1), localTarget(2), localTarget(3)); %#ok<AGROW>
end
if isempty(parts)
    return;
end
text = strjoin(parts, ' | ');
end

function repoRoot = locate_repo_root(startDir)
repoRoot = startDir;
for idx = 1:12
    if isfolder(fullfile(repoRoot, 'matlab', 'modules')) && ...
            (isfolder(fullfile(repoRoot, 'ardu_ws')) || isfolder(fullfile(repoRoot, 'external')))
        return;
    end
    parentDir = fileparts(repoRoot);
    if strcmp(parentDir, repoRoot)
        break;
    end
    repoRoot = parentDir;
end
repoRoot = fileparts(fileparts(fileparts(fileparts(startDir))));
end
