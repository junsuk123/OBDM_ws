function pathResults = evaluate_path_generation(scenarioLogs, cfg)
%EVALUATE_PATH_GENERATION Compute path-generation metrics from scenario logs.

pathResults = struct();
pathResults.scenario = struct([]);
pathResults.summary = empty_summary();
pathResults.metricNames = {'meanTrackingError', 'finalTrackingError', 'pathEfficiency', ...
    'targetSmoothness', 'commandCoverage', 'markerCoverage'};

if isempty(scenarioLogs)
    return;
end

scenarioMetrics = [];
for idx = 1:numel(scenarioLogs)
    if ~isfield(scenarioLogs(idx), 'time') || isempty(scenarioLogs(idx).time)
        continue;
    end
    currentMetrics = evaluate_one_scenario(scenarioLogs(idx), cfg);
    if isempty(scenarioMetrics)
        scenarioMetrics = currentMetrics;
    else
        scenarioMetrics(end+1) = currentMetrics; %#ok<AGROW>
    end
end

pathResults.scenario = scenarioMetrics;
pathResults.summary = summarize_scenarios(scenarioMetrics);
end

function metrics = evaluate_one_scenario(logItem, cfg)
time = logItem.time(:);
sampleCount = numel(time);
dt = median(diff(time), 'omitnan');
if isempty(dt) || ~isfinite(dt) || dt <= 0
    dt = cfg.simulation.dt;
end

uav = nan(sampleCount, 3);
pad = nan(sampleCount, 3);
target = nan(sampleCount, 3);
trackingError = nan(sampleCount, 1);
approachStep = nan(sampleCount, 1);
maxStep = nan(sampleCount, 1);
navGain = nan(sampleCount, 1);
trackQuality = nan(sampleCount, 1);
risk = nan(sampleCount, 1);
descentAuthority = nan(sampleCount, 1);
hasMarker = false(sampleCount, 1);
finalLand = false(sampleCount, 1);

for k = 1:sampleCount
    if isfield(logItem, 'state') && numel(logItem.state) >= k && isfield(logItem.state(k), 'uav')
        uav(k, :) = get_vector(logItem.state(k).uav, 'position', [nan, nan, nan]);
    end
    if isfield(logItem, 'observation') && numel(logItem.observation) >= k && isfield(logItem.observation(k), 'value')
        obs = logItem.observation(k).value;
        pad(k, :) = get_vector(obs, 'markerPosition', [nan, nan, nan]);
        if isfield(obs, 'positionError') && ~isempty(obs.positionError)
            trackingError(k) = double(obs.positionError);
        end
    end
    if isfield(logItem, 'onlineDecision') && numel(logItem.onlineDecision) >= k && ...
            isstruct(logItem.onlineDecision{k}) && isfield(logItem.onlineDecision{k}, 'trajectory')
        traj = logItem.onlineDecision{k}.trajectory;
        target(k, :) = get_vector(traj, 'target', [nan, nan, nan]);
        markerFromTrajectory = get_vector(traj, 'markerWorldPosition', [nan, nan, nan]);
        if any(isfinite(markerFromTrajectory)) && ~any(isfinite(pad(k, :)))
            pad(k, :) = markerFromTrajectory;
        end
        if isfield(traj, 'distanceToMarker') && ~isempty(traj.distanceToMarker)
            trackingError(k) = double(traj.distanceToMarker);
        end
        approachStep(k) = get_scalar(traj, 'approachStep', nan);
        maxStep(k) = get_scalar(traj, 'maxHorizontalStep', nan);
        navGain(k) = get_scalar(traj, 'navigationGain', nan);
        trackQuality(k) = get_scalar(traj, 'trackQuality', nan);
        risk(k) = get_scalar(traj, 'risk', nan);
        hasMarker(k) = logical(get_scalar(traj, 'hasMarker', false));
        finalLand(k) = logical(get_scalar(traj, 'finalLand', false));
        if isfield(traj, 'ontologyGraph') && isfield(traj.ontologyGraph, 'values')
            descentAuthority(k) = get_scalar(traj.ontologyGraph.values, 'descentAuthority', nan);
        end
    end
    if ~isfinite(trackingError(k)) && all(isfinite(uav(k, 1:2))) && all(isfinite(pad(k, 1:2)))
        trackingError(k) = norm(uav(k, 1:2) - pad(k, 1:2));
    end
end

validTarget = all(isfinite(target), 2);
validPad = all(isfinite(pad(:, 1:2)), 2);
validUav = all(isfinite(uav), 2);
validError = isfinite(trackingError);

uavPathLength = path_length(uav(validUav, :));
targetPathLength = path_length(target(validTarget, :));
padPathLength = path_length(pad(validPad, :));
straightDistance = straight_line_distance(uav, pad);
targetSmoothness = mean_second_difference(target(validTarget, :), dt);

captureRadius = get_guidance_value(cfg, 'alignRadius', 0.75);
finalRadius = get_guidance_value(cfg, 'finalLandRadius', 0.35);

metrics = struct();
metrics.scenarioIndex = get_scalar(logItem, 'scenarioIndex', numel(logItem));
metrics.sampleCount = sampleCount;
metrics.duration = time(end) - time(1);
metrics.commandCoverage = mean(validTarget);
metrics.markerCoverage = mean(hasMarker | validPad);
metrics.meanTrackingError = mean_or_nan(trackingError(validError));
metrics.medianTrackingError = median_or_nan(trackingError(validError));
metrics.maxTrackingError = max_or_nan(trackingError(validError));
metrics.finalTrackingError = last_finite(trackingError);
metrics.minTrackingError = min_or_nan(trackingError(validError));
metrics.uavPathLength = uavPathLength;
metrics.targetPathLength = targetPathLength;
metrics.padPathLength = padPathLength;
metrics.straightDistance = straightDistance;
metrics.pathEfficiency = bounded_ratio(straightDistance, uavPathLength);
metrics.targetPathEfficiency = bounded_ratio(straightDistance, targetPathLength);
metrics.targetSmoothness = targetSmoothness;
metrics.meanApproachStep = mean_or_nan(approachStep);
metrics.maxApproachStep = max_or_nan(approachStep);
metrics.meanMaxStep = mean_or_nan(maxStep);
metrics.meanNavigationGain = mean_or_nan(navGain);
metrics.meanTrackQuality = mean_or_nan(trackQuality);
metrics.meanRisk = mean_or_nan(risk);
metrics.meanDescentAuthority = mean_or_nan(descentAuthority);
metrics.timeToFirstMarker = first_time(time, hasMarker | validPad);
metrics.timeToCaptureRadius = first_time(time, trackingError <= captureRadius);
metrics.timeToFinalRadius = first_time(time, trackingError <= finalRadius);
metrics.finalLandCommanded = any(finalLand);
metrics.timeToFinalLandCommand = first_time(time, finalLand);
metrics.successLabel = get_scalar(logItem, 'finalLabel', nan);
metrics.timeSeries = struct('time', time, 'uav', uav, 'pad', pad, 'target', target, ...
    'trackingError', trackingError, 'approachStep', approachStep, 'maxStep', maxStep, ...
    'navigationGain', navGain, 'trackQuality', trackQuality, 'risk', risk, ...
    'descentAuthority', descentAuthority, 'hasMarker', hasMarker, 'finalLand', finalLand);
end

function summary = summarize_scenarios(metrics)
summary = empty_summary();
if isempty(metrics)
    return;
end
fields = {'meanTrackingError', 'finalTrackingError', 'maxTrackingError', 'pathEfficiency', ...
    'targetPathEfficiency', 'targetSmoothness', 'commandCoverage', 'markerCoverage', ...
    'uavPathLength', 'targetPathLength', 'padPathLength', 'meanApproachStep', ...
    'meanNavigationGain', 'meanTrackQuality', 'meanRisk', 'meanDescentAuthority'};
for idx = 1:numel(fields)
    values = [metrics.(fields{idx})];
    summary.(fields{idx}) = mean(values, 'omitnan');
end
summary.scenarioCount = numel(metrics);
summary.successRate = mean([metrics.successLabel] == 1, 'omitnan');
summary.finalLandCommandRate = mean([metrics.finalLandCommanded]);
summary.meanTimeToCaptureRadius = mean([metrics.timeToCaptureRadius], 'omitnan');
summary.meanTimeToFinalRadius = mean([metrics.timeToFinalRadius], 'omitnan');
end

function summary = empty_summary()
summary = struct('scenarioCount', 0, 'successRate', NaN, 'finalLandCommandRate', NaN, ...
    'meanTrackingError', NaN, 'finalTrackingError', NaN, 'maxTrackingError', NaN, ...
    'pathEfficiency', NaN, 'targetPathEfficiency', NaN, 'targetSmoothness', NaN, ...
    'commandCoverage', NaN, 'markerCoverage', NaN, 'uavPathLength', NaN, ...
    'targetPathLength', NaN, 'padPathLength', NaN, 'meanApproachStep', NaN, ...
    'meanNavigationGain', NaN, 'meanTrackQuality', NaN, 'meanRisk', NaN, ...
    'meanDescentAuthority', NaN, 'meanTimeToCaptureRadius', NaN, ...
    'meanTimeToFinalRadius', NaN);
end

function len = path_length(points)
if size(points, 1) < 2
    len = NaN;
    return;
end
deltas = diff(points, 1, 1);
len = sum(vecnorm(deltas, 2, 2), 'omitnan');
end

function dist = straight_line_distance(uav, pad)
validUav = find(all(isfinite(uav), 2), 1, 'first');
validPad = find(all(isfinite(pad(:, 1:2)), 2), 1, 'last');
if isempty(validUav) || isempty(validPad)
    dist = NaN;
    return;
end
dist = norm(uav(validUav, 1:2) - pad(validPad, 1:2));
end

function value = mean_second_difference(points, dt)
if size(points, 1) < 3
    value = NaN;
    return;
end
accelLike = diff(points, 2, 1) ./ max(dt^2, eps);
value = mean(vecnorm(accelLike, 2, 2), 'omitnan');
end

function value = bounded_ratio(numerator, denominator)
if ~isfinite(numerator) || ~isfinite(denominator) || denominator <= eps
    value = NaN;
else
    value = max(0.0, min(1.0, numerator / denominator));
end
end

function value = mean_or_nan(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = mean(values);
end
end

function value = median_or_nan(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = median(values);
end
end

function value = max_or_nan(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = max(values);
end
end

function value = min_or_nan(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = min(values);
end
end

function value = last_finite(vector)
idx = find(isfinite(vector), 1, 'last');
if isempty(idx)
    value = NaN;
else
    value = vector(idx);
end
end

function t = first_time(time, mask)
idx = find(mask(:), 1, 'first');
if isempty(idx)
    t = NaN;
else
    t = time(idx);
end
end

function value = get_vector(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && isnumeric(item.(fieldName)) && ...
        numel(item.(fieldName)) >= numel(defaultValue)
    candidate = item.(fieldName);
    value = candidate(1:numel(defaultValue));
end
value = value(:)';
end

function value = get_scalar(item, fieldName, defaultValue)
value = defaultValue;
if isstruct(item) && isfield(item, fieldName) && ~isempty(item.(fieldName))
    value = double(item.(fieldName));
end
end

function value = get_guidance_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end
