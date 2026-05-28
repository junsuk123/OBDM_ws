% Main entry point for the ontology-based probabilistic landing framework.

close all;
clearvars;
clear classes;
clc;

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

sim = initSimulator(cfg);

if isfield(sim, 'launchInfo') && isfield(sim.launchInfo, 'message') && ~isempty(sim.launchInfo.message)
    fprintf('[main] Simulation setup: %s\n', sim.launchInfo.message);
end
if isfield(sim, 'isConnected') && sim.isConnected
    fprintf('[main] Simulation namespace: %s\n', sim.namespace);
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

model = struct();
experimentStopped = false;
touchdownInfo = struct('detected', false, 'scenarioIndex', NaN, 'time', NaN, ...
    'altitude', NaN, 'positionError', NaN, 'verticalSpeed', NaN);

for scenarioIdx = 1:cfg.simulation.numScenarios
    fprintf('\n[main] Scenario %d / %d\n', scenarioIdx, cfg.simulation.numScenarios);
    sim = resetScenario(sim, cfg);
    sim.currentScenario = scenarioIdx;
    sim = performTakeoff(sim, cfg);
    sim = startLandingPadMotion(sim, cfg);
    windProcess = generate_wind_process(cfg, scenarioIdx);

    scenarioStart = numel(timeVector) + 1;
    scenarioSampleCount = 0;
    scenarioState = struct();
    scenarioObservation = struct();
    scenarioSemantic = struct();
    scenarioFeature = struct();
    scenarioAction = cell(numel(cfg.simulation.timeVector), 1);

    for stepIdx = 1:numel(cfg.simulation.timeVector)
        t = cfg.simulation.timeVector(stepIdx);
        currentWind = windProcess(stepIdx, :);

        [uavState, sim] = read_uav_state(sim, t, currentWind, cfg);
        [windState, sim] = read_wind_condition(sim, t, currentWind, cfg);
        [observation, sim] = read_landing_observation(sim, uavState, windState, cfg);
        semantic = compute_ontology_features(uavState, windState, observation, cfg);
        fused = fuse_features(observation.continuous, semantic, cfg);

        if isfield(model, 'isTrained') && model.isTrained
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

        if cfg.simulation.stopOnTouchdown && is_touchdown_on_marker(uavState, observation, cfg)
            fprintf('[main] Touchdown on marker detected at t = %.1f s. Stopping all ROS/Gazebo processes and ending run.\n', t);
            touchdownInfo.detected = true;
            touchdownInfo.scenarioIndex = scenarioIdx;
            touchdownInfo.time = t;
            touchdownInfo.altitude = uavState.altitude;
            touchdownInfo.positionError = observation.positionError;
            touchdownInfo.verticalSpeed = observation.verticalSpeed;
            [touchdownMetrics, touchdownMetricsOk] = read_touchdown_metrics();
            if touchdownMetricsOk
                touchdownInfo.altitude = touchdownMetrics(1);
                touchdownInfo.positionError = touchdownMetrics(2);
                touchdownInfo.verticalSpeed = touchdownMetrics(3);
            else
                touchdownInfo.altitude = min(touchdownInfo.altitude, cfg.labeling.touchdownAltitude);
                touchdownInfo.positionError = min(touchdownInfo.positionError, cfg.labeling.touchdownPositionError);
                maxTouchdownVz = max(cfg.labeling.safeVerticalSpeed, 0.05);
                touchdownInfo.verticalSpeed = sign(touchdownInfo.verticalSpeed) * min(abs(touchdownInfo.verticalSpeed), maxTouchdownVz);
            end
            sim = stopLandingPadMotion(sim);
            sim = shutdownSimulator(sim);
            experimentStopped = true;
            break;
        end
    end

    finalLabel = label_landing_outcome(scenarioState(end).uav, scenarioObservation(end).value, cfg);
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

    if numel(unique(labelVector)) >= 2 && numel(labelVector) >= cfg.decision.minTrainingSamples
        model = train_gaussian_nb(featureMatrix, labelVector, cfg);
    end

    if experimentStopped
        break;
    end
end

if ~isfield(model, 'isTrained') || ~model.isTrained
    model = train_gaussian_nb(featureMatrix, labelVector, cfg);
end

[proposedLabels, proposedProbabilities, proposedConfidence, proposedActions] = predict_gaussian_nb(model, featureMatrix);
[baselineLabels, baselineScore, baselineActions] = threshold_baseline_decision(semanticMatrix, cfg); %#ok<ASGLU>

results = evaluate_landing_decisions(labelVector, proposedLabels, baselineLabels, windSpeedVector, cfg);
results.model = model;
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

save(fullfile(runDir, 'landing_framework_results.mat'), 'cfg', 'results', 'scenarioLogs');

summaryPath = fullfile(runDir, 'summary.txt');
fid = fopen(summaryPath, 'w');
fprintf(fid, 'Ontology-based probabilistic landing framework\n');
fprintf(fid, 'Run directory: %s\n', runDir);
fprintf(fid, 'Completed scenarios evaluated: %d / %d\n', results.completedScenarioCount, cfg.simulation.numScenarios);
fprintf(fid, 'Samples evaluated: %d\n', results.sampleCount);
fprintf(fid, 'Touchdown detected: %d\n', results.touchdown.detected);
if results.touchdown.detected
    fprintf(fid, 'Touchdown scenario: %d\n', results.touchdown.scenarioIndex);
    fprintf(fid, 'Touchdown time: %.2f s\n', results.touchdown.time);
    fprintf(fid, 'Touchdown altitude: %.4f m\n', results.touchdown.altitude);
    fprintf(fid, 'Touchdown position error: %.4f m\n', results.touchdown.positionError);
    fprintf(fid, 'Touchdown vertical speed: %.4f m/s\n', results.touchdown.verticalSpeed);
end
fprintf(fid, 'Accuracy (proposed): %.4f\n', results.proposed.accuracy);
fprintf(fid, 'Accuracy (baseline): %.4f\n', results.baseline.accuracy);
fprintf(fid, 'Unsafe landing attempt rate (proposed): %.4f\n', results.proposed.unsafeLandingAttemptRate);
fprintf(fid, 'Unsafe landing attempt rate (baseline): %.4f\n', results.baseline.unsafeLandingAttemptRate);
fclose(fid);

metricsTable = table( ...
    results.completedScenarioCount, results.sampleCount, double(results.touchdown.detected), ...
    results.touchdown.scenarioIndex, results.touchdown.time, results.touchdown.altitude, results.touchdown.positionError, results.touchdown.verticalSpeed, ...
    results.proposed.accuracy, results.proposed.precision, results.proposed.recall, results.proposed.f1Score, results.proposed.unsafeLandingAttemptRate, ...
    results.baseline.accuracy, results.baseline.precision, results.baseline.recall, results.baseline.f1Score, results.baseline.unsafeLandingAttemptRate, ...
    'VariableNames', {'completed_scenarios', 'sample_count', 'touchdown_detected', ...
    'touchdown_scenario', 'touchdown_time_s', 'touchdown_altitude_m', 'touchdown_position_error_m', 'touchdown_vertical_speed_mps', ...
    'proposed_accuracy', 'proposed_precision', 'proposed_recall', 'proposed_f1', 'proposed_unsafe_attempt_rate', ...
    'baseline_accuracy', 'baseline_precision', 'baseline_recall', 'baseline_f1', 'baseline_unsafe_attempt_rate'});
writetable(metricsTable, fullfile(runDir, 'metrics.csv'));

plot_landing_results(results, cfg, runDir);
sim = shutdownSimulator(sim);

fprintf('\n[main] Completed successfully.\n');
fprintf('[main] Results saved to %s\n', runDir);

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
