function rlData = build_reinforcement_training_data(scenarioLogs, pathGeneration, cfg)
%BUILD_REINFORCEMENT_TRAINING_DATA Build smoothness-aware RL reward episodes.

rlData = struct();
rlData.enabled = isfield(cfg, 'reinforcement') && isfield(cfg.reinforcement, 'enabled') && cfg.reinforcement.enabled;
rlData.episodes = struct([]);
rlData.summary = empty_summary();
rlData.rewardComponents = {'tracking', 'smoothness', 'efficiency', 'coverage', 'trackQuality', 'risk', 'terminal'};
if ~rlData.enabled || isempty(scenarioLogs) || ~isfield(pathGeneration, 'scenario') || isempty(pathGeneration.scenario)
    return;
end

episodes = [];
for idx = 1:numel(pathGeneration.scenario)
    episode = build_episode(scenarioLogs(idx), pathGeneration.scenario(idx), cfg);
    if isempty(episodes)
        episodes = episode;
    else
        episodes(end+1) = episode; %#ok<AGROW>
    end
end

rlData.episodes = episodes;
rlData.summary = summarize_episodes(episodes);
end

function episode = build_episode(logItem, pathMetrics, cfg)
ts = pathMetrics.timeSeries;
time = ts.time(:);
sampleCount = numel(time);
dt = median(diff(time), 'omitnan');
if isempty(dt) || ~isfinite(dt) || dt <= 0
    dt = cfg.simulation.dt;
end

targetSmoothness = per_step_smoothness(ts.target, dt);
trackingScore = 1.0 - normalize_series(ts.trackingError, cfg.reinforcement.trackingErrorReference);
smoothnessScore = 1.0 - normalize_series(targetSmoothness, cfg.reinforcement.targetSmoothnessReference);
efficiencyScore = constant_score(pathMetrics.targetPathEfficiency, sampleCount, 0.5);
coverageScore = double(ts.hasMarker(:));
trackQualityScore = fill_missing_score(ts.trackQuality(:), 0.5);
riskScore = 1.0 - fill_missing_score(ts.risk(:), 0.5);
terminalScore = zeros(sampleCount, 1);
if sampleCount > 0
    if isfinite(pathMetrics.successLabel) && pathMetrics.successLabel == 1
        terminalScore(end) = cfg.reinforcement.successBonus;
    elseif isfinite(pathMetrics.successLabel) && pathMetrics.successLabel == 0
        terminalScore(end) = -cfg.reinforcement.failurePenalty;
    end
end

reward = cfg.reinforcement.trackingWeight * trackingScore + ...
    cfg.reinforcement.smoothnessWeight * smoothnessScore + ...
    cfg.reinforcement.efficiencyWeight * efficiencyScore + ...
    cfg.reinforcement.coverageWeight * coverageScore + ...
    cfg.reinforcement.trackQualityWeight * trackQualityScore + ...
    cfg.reinforcement.riskWeight * riskScore + terminalScore;

episode = struct();
episode.scenarioIndex = pathMetrics.scenarioIndex;
episode.sampleCount = sampleCount;
episode.return = sum(reward, 'omitnan');
episode.meanReward = mean(reward, 'omitnan');
episode.meanSmoothnessScore = mean(smoothnessScore, 'omitnan');
episode.meanTrackingScore = mean(trackingScore, 'omitnan');
episode.targetSmoothness = pathMetrics.targetSmoothness;
episode.successLabel = pathMetrics.successLabel;
episode.stateFeatureNames = [cfg.featureFusion.rawNames, cfg.featureFusion.semanticNames];
episode.semanticNames = cfg.featureFusion.semanticNames;
episode.time = time;
episode.reward = reward;
episode.components = struct('tracking', trackingScore, 'smoothness', smoothnessScore, ...
    'efficiency', efficiencyScore, 'coverage', coverageScore, 'trackQuality', trackQualityScore, ...
    'risk', riskScore, 'terminal', terminalScore, 'targetSmoothness', targetSmoothness);
episode.actions = extract_actions(logItem, sampleCount);
episode.semantic = extract_semantic(logItem, sampleCount, numel(cfg.featureFusion.semanticNames));
episode.features = extract_features(logItem, sampleCount, numel(episode.stateFeatureNames));
end

function summary = summarize_episodes(episodes)
summary = empty_summary();
if isempty(episodes)
    return;
end
summary.episodeCount = numel(episodes);
summary.meanReturn = mean([episodes.return], 'omitnan');
summary.meanReward = mean([episodes.meanReward], 'omitnan');
summary.meanSmoothnessScore = mean([episodes.meanSmoothnessScore], 'omitnan');
summary.meanTrackingScore = mean([episodes.meanTrackingScore], 'omitnan');
summary.meanTargetSmoothness = mean([episodes.targetSmoothness], 'omitnan');
summary.successRate = mean([episodes.successLabel] == 1, 'omitnan');
end

function summary = empty_summary()
summary = struct('episodeCount', 0, 'meanReturn', NaN, 'meanReward', NaN, ...
    'meanSmoothnessScore', NaN, 'meanTrackingScore', NaN, ...
    'meanTargetSmoothness', NaN, 'successRate', NaN);
end

function smoothness = per_step_smoothness(target, dt)
smoothness = nan(size(target, 1), 1);
valid = all(isfinite(target), 2);
if nnz(valid) < 3
    return;
end
validIdx = find(valid);
targetValid = target(validIdx, :);
secondDiff = diff(targetValid, 2, 1) ./ max(dt^2, eps);
smoothness(validIdx(2:end-1)) = vecnorm(secondDiff, 2, 2);
smoothness = fill_missing_score(smoothness, nanmedian(smoothness));
end

function score = normalize_series(values, reference)
score = fill_missing_score(values(:), reference);
score = min(max(score ./ max(reference, eps), 0.0), 1.0);
end

function values = fill_missing_score(values, defaultValue)
values = values(:);
if ~isfinite(defaultValue)
    defaultValue = 0.0;
end
values(~isfinite(values)) = defaultValue;
end

function score = constant_score(value, count, defaultValue)
if ~isfinite(value)
    value = defaultValue;
end
score = repmat(max(0.0, min(1.0, value)), count, 1);
end

function actions = extract_actions(logItem, sampleCount)
actions = repmat({'HOVER'}, sampleCount, 1);
if ~isfield(logItem, 'onlineDecision')
    return;
end
for idx = 1:min(sampleCount, numel(logItem.onlineDecision))
    item = logItem.onlineDecision{idx};
    if isstruct(item) && isfield(item, 'action') && ~isempty(item.action)
        actions{idx} = char(item.action);
    end
end
end

function semantic = extract_semantic(logItem, sampleCount, featureCount)
semantic = nan(sampleCount, featureCount);
if ~isfield(logItem, 'semantic')
    return;
end
for idx = 1:min(sampleCount, numel(logItem.semantic))
    if isfield(logItem.semantic(idx), 'value') && isfield(logItem.semantic(idx).value, 'vector')
        vector = logItem.semantic(idx).value.vector(:)';
        semantic(idx, 1:min(featureCount, numel(vector))) = vector(1:min(featureCount, numel(vector)));
    end
end
end

function features = extract_features(logItem, sampleCount, featureCount)
features = nan(sampleCount, featureCount);
if ~isfield(logItem, 'observation') || ~isfield(logItem, 'semantic')
    return;
end
for idx = 1:min([sampleCount, numel(logItem.observation), numel(logItem.semantic)])
    rawVector = [];
    semanticVector = [];
    if isfield(logItem.observation(idx), 'value') && isfield(logItem.observation(idx).value, 'continuous')
        rawVector = logItem.observation(idx).value.continuous(:)';
    end
    if isfield(logItem.semantic(idx), 'value') && isfield(logItem.semantic(idx).value, 'vector')
        semanticVector = logItem.semantic(idx).value.vector(:)';
    end
    vector = [rawVector, semanticVector];
    if ~isempty(vector)
        features(idx, 1:min(featureCount, numel(vector))) = vector(1:min(featureCount, numel(vector)));
    end
end
end

function value = nanmedian(values)
values = values(isfinite(values));
if isempty(values)
    value = NaN;
else
    value = median(values);
end
end
