function plot_landing_results(results, cfg, runDir)
%PLOT_LANDING_RESULTS Generate and save plots for the landing decision run.

if ~cfg.visualization.enabled
    return;
end

visibleFlag = 'off';
if cfg.visualization.visible
    visibleFlag = 'on';
end

timeVector = results.timeVector;
if isempty(timeVector)
    warning('[plot_landing_results] No samples were collected; skipping result plots.');
    plot_no_data_figure(runDir, cfg, visibleFlag, 'No samples were collected for this run.');
    return;
end
traceStyle = plot_trace_style(timeVector);

figure('Name', 'Landing Scenario Summary', 'Color', 'w', 'Visible', visibleFlag);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Landing scenario result summary');

nexttile;
plot(timeVector, results.altitudeVector, traceStyle, 'LineWidth', 1.4); hold on;
if isfield(results, 'positionErrorVector') && ~isempty(results.positionErrorVector)
    plot(timeVector, results.positionErrorVector, traceStyle, 'LineWidth', 1.2);
end
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('State value');
legend({'Altitude (m)', 'Position error (m)'}, 'Location', 'best');
title('Final approach state');
grid on;

nexttile;
stairs(timeVector, results.ontologyLabels, 'LineWidth', 1.4); hold on;
stairs(timeVector, results.trueLabels, 'LineWidth', 1.0);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Class label');
yticks([0 1]);
yticklabels({'HOVER', 'LAND'});
legend({'Ontology', 'True'}, 'Location', 'best');
title('Decision trace');
grid on;
ylim([-0.1 1.1]);

nexttile;
metricNames = categorical({'Accuracy', 'Precision', 'Recall', 'F1', 'Unsafe'});
metricNames = reordercats(metricNames, {'Accuracy', 'Precision', 'Recall', 'F1', 'Unsafe'});
metricValues = [
    results.ontology.accuracy;
    results.ontology.precision;
    results.ontology.recall;
    results.ontology.f1Score;
    results.ontology.unsafeLandingAttemptRate];
bar(metricNames, metricValues);
ylabel('Score');
legend({'Ontology'}, 'Location', 'best');
title('Decision performance');
grid on;
ylim([0 1]);

nexttile;
axis off;
summaryLines = build_summary_lines(results, cfg);
text(0, 1, strjoin(summaryLines, newline), ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'FontName', 'monospaced', ...
    'FontSize', 10);
title('Touchdown and run summary');
save_figure_if_needed(gcf, runDir, 'scenario_result_summary', cfg);

figure('Name', 'Wind Profile', 'Color', 'w', 'Visible', visibleFlag);
plot(timeVector, results.windSpeedVector, traceStyle, 'LineWidth', 1.4);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Wind speed (m/s)');
title('Wind profile over time');
grid on;
save_figure_if_needed(gcf, runDir, 'wind_profile', cfg);

figure('Name', 'UAV Attitude and Altitude', 'Color', 'w', 'Visible', visibleFlag);
subplot(2, 1, 1);
plot(timeVector, results.altitudeVector, traceStyle, 'LineWidth', 1.4);
add_touchdown_marker(results);
ylabel('Altitude (m)');
title('Altitude');
grid on;
subplot(2, 1, 2);
plot(timeVector, rad2deg(results.rollVector), traceStyle, 'LineWidth', 1.2); hold on;
plot(timeVector, rad2deg(results.pitchVector), traceStyle, 'LineWidth', 1.2);
plot(timeVector, rad2deg(results.yawVector), traceStyle, 'LineWidth', 1.2);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Angle (deg)');
legend({'Roll', 'Pitch', 'Yaw'}, 'Location', 'best');
title('Attitude');
grid on;
save_figure_if_needed(gcf, runDir, 'altitude_attitude', cfg);

figure('Name', 'Semantic Features', 'Color', 'w', 'Visible', visibleFlag);
semanticCount = size(results.semanticMatrix, 2);
semanticNames = cfg.featureFusion.semanticNames(1:min(numel(cfg.featureFusion.semanticNames), semanticCount));
plot(timeVector, results.semanticMatrix(:, 1:semanticCount), traceStyle, 'LineWidth', 1.2);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Normalized score');
if ~isempty(semanticNames)
    legend(semanticNames, 'Location', 'best');
end
title('Ontology-based semantic features');
grid on;
ylim([0 1]);
save_figure_if_needed(gcf, runDir, 'semantic_features', cfg);

figure('Name', 'Decision Timeline', 'Color', 'w', 'Visible', visibleFlag);
stairs(timeVector, results.ontologyLabels, 'LineWidth', 1.4); hold on;
stairs(timeVector, results.trueLabels, 'LineWidth', 1.0);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Class label');
legend({'Ontology', 'True'}, 'Location', 'best');
title('Decision timeline');
grid on;
ylim([-0.1 1.1]);
save_figure_if_needed(gcf, runDir, 'decision_timeline', cfg);

figure('Name', 'Confusion Matrix', 'Color', 'w', 'Visible', visibleFlag);
plot_confusion_matrix(results.ontology.confusionMatrix, 'Ontology');
save_figure_if_needed(gcf, runDir, 'confusion_matrix', cfg);

figure('Name', 'Wind Bin Analysis', 'Color', 'w', 'Visible', visibleFlag);
binCount = numel(results.windBinAnalysis);
ontologyLandingRate = zeros(binCount, 1);
binLabels = cell(binCount, 1);
for binIdx = 1:binCount
    binLabels{binIdx} = sprintf('%.0f-%.0f', results.windBinAnalysis(binIdx).lowerEdge, results.windBinAnalysis(binIdx).upperEdge);
    ontologyLandingRate(binIdx) = results.windBinAnalysis(binIdx).ontology.recall;
end
bar(ontologyLandingRate);
xlabel('Wind-speed bin');
ylabel('Landing recall');
legend({'Ontology'}, 'Location', 'best');
title('Landing performance by wind-speed bin');
grid on;
set(gca, 'XTickLabel', binLabels);
    save_figure_if_needed(gcf, runDir, 'wind_bin_analysis', cfg);

plot_path_generation_results(results, cfg, runDir, visibleFlag);
plot_reinforcement_reward_results(results, cfg, runDir, visibleFlag);

function summaryLines = build_summary_lines(results, cfg)
    summaryLines = {
        sprintf('Completed scenarios : %d / %d', results.completedScenarioCount, cfg.simulation.numScenarios)
        sprintf('Samples evaluated   : %d', results.sampleCount)
        sprintf('Touchdown detected  : %d', logical_to_double(results.touchdown.detected))
        sprintf('Ontology accuracy   : %.3f', results.ontology.accuracy)
        sprintf('Ontology unsafe rate: %.3f', results.ontology.unsafeLandingAttemptRate)
        };
    if isfield(results, 'touchdown') && results.touchdown.detected
        summaryLines = [summaryLines; {
            sprintf('Touchdown scenario  : %d', results.touchdown.scenarioIndex)
            sprintf('Touchdown time      : %.2f s', results.touchdown.time)
            sprintf('Touchdown altitude  : %.3f m', results.touchdown.altitude)
            sprintf('Touchdown pos error : %.3f m', results.touchdown.positionError)
            sprintf('Touchdown Vz        : %.3f m/s', results.touchdown.verticalSpeed)
            }];
    end
    if isfield(results, 'pathGeneration') && isfield(results.pathGeneration, 'summary')
        pathSummary = results.pathGeneration.summary;
        summaryLines = [summaryLines; {
            sprintf('Path mean error    : %.3f m', pathSummary.meanTrackingError)
            sprintf('Path final error   : %.3f m', pathSummary.finalTrackingError)
            sprintf('Path efficiency    : %.3f', pathSummary.pathEfficiency)
            sprintf('Command coverage   : %.3f', pathSummary.commandCoverage)
            }];
    end
end

function value = logical_to_double(inputValue)
    value = double(logical(inputValue));
end

function add_touchdown_marker(results)
    if ~isfield(results, 'touchdown') || ~results.touchdown.detected || isnan(results.touchdown.time)
        return;
    end
    xline(results.touchdown.time, '--r', 'Touchdown', ...
        'LabelOrientation', 'horizontal', ...
        'LabelVerticalAlignment', 'bottom', ...
        'HandleVisibility', 'off');
end

function plot_confusion_matrix(confusionMatrix, plotTitle)
    imagesc(confusionMatrix);
    axis equal tight;
    colormap(parula);
    colorbar;
    xticks([1 2]);
    yticks([1 2]);
    xticklabels({'HOVER', 'LAND'});
    yticklabels({'HOVER', 'LAND'});
    xlabel('Predicted');
    ylabel('Actual');
    title(sprintf('%s confusion matrix', plotTitle));
    for rowIdx = 1:2
        for colIdx = 1:2
            text(colIdx, rowIdx, num2str(confusionMatrix(rowIdx, colIdx)), ...
                'HorizontalAlignment', 'center', 'Color', 'w', 'FontWeight', 'bold');
        end
    end
end

function save_figure_if_needed(figHandle, runDir, baseName, cfg)
    if ~cfg.visualization.saveFigures
        return;
    end
    if ~exist(runDir, 'dir')
        mkdir(runDir);
    end
    pngPath = fullfile(runDir, [baseName '.png']);
    saveas(figHandle, pngPath);
end

function style = plot_trace_style(timeVector)
    if numel(timeVector) < 2
        style = 'o-';
    else
        style = '-';
    end
end

function plot_no_data_figure(runDir, cfg, visibleFlag, message)
    if ~cfg.visualization.enabled
        return;
    end
    figure('Name', 'No Result Data', 'Color', 'w', 'Visible', visibleFlag);
    axis off;
    text(0.5, 0.5, message, 'Units', 'normalized', ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
    save_figure_if_needed(gcf, runDir, 'no_result_data', cfg);
end

function plot_path_generation_results(results, cfg, runDir, visibleFlag)
    if ~isfield(results, 'pathGeneration') || ~isfield(results.pathGeneration, 'scenario') || ...
            isempty(results.pathGeneration.scenario)
        return;
    end

    scenarioMetrics = results.pathGeneration.scenario;
    scenarioIds = [scenarioMetrics.scenarioIndex];

    figure('Name', 'Path Generation Metric Dashboard', 'Color', 'w', 'Visible', visibleFlag);
    layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, 'Path generation performance metrics');

    nexttile;
    bar(scenarioIds, [[scenarioMetrics.meanTrackingError]' [scenarioMetrics.finalTrackingError]']);
    xlabel('Scenario');
    ylabel('Error (m)');
    legend({'Mean tracking error', 'Final tracking error'}, 'Location', 'best');
    title('Pad tracking accuracy');
    grid on;

    nexttile;
    bar(scenarioIds, [[scenarioMetrics.pathEfficiency]' [scenarioMetrics.targetPathEfficiency]']);
    xlabel('Scenario');
    ylabel('Efficiency');
    ylim([0 1]);
    legend({'UAV path', 'Generated target path'}, 'Location', 'best');
    title('Straight-line efficiency');
    grid on;

    nexttile;
    bar(scenarioIds, [[scenarioMetrics.commandCoverage]' [scenarioMetrics.markerCoverage]']);
    xlabel('Scenario');
    ylabel('Coverage');
    ylim([0 1]);
    legend({'Command target', 'Marker/pad track'}, 'Location', 'best');
    title('Guidance availability');
    grid on;

    nexttile;
    yyaxis left;
    bar(scenarioIds, [scenarioMetrics.targetSmoothness]');
    ylabel('Target smoothness (m/s^2)');
    yyaxis right;
    plot(scenarioIds, [scenarioMetrics.meanApproachStep]', '-o', 'LineWidth', 1.4);
    ylabel('Mean approach step (m)');
    xlabel('Scenario');
    title('Command smoothness and step size');
    grid on;
    save_figure_if_needed(gcf, runDir, 'path_generation_metric_dashboard', cfg);

    figure('Name', 'Path Tracking Time Series', 'Color', 'w', 'Visible', visibleFlag);
    layout = tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, 'Path tracking and ontology guidance signals');
    for scenarioIdx = 1:numel(scenarioMetrics)
        ts = scenarioMetrics(scenarioIdx).timeSeries;
        label = sprintf('S%d', scenarioMetrics(scenarioIdx).scenarioIndex);
        style = plot_trace_style(ts.time);
        nexttile(1);
        plot(ts.time, ts.trackingError, style, 'DisplayName', label, 'LineWidth', 1.1); hold on;
        nexttile(2);
        plot(ts.time, ts.approachStep, style, 'DisplayName', label, 'LineWidth', 1.1); hold on;
        nexttile(3);
        plot(ts.time, ts.trackQuality, style, 'DisplayName', [label ' track'], 'LineWidth', 1.1); hold on;
        plot(ts.time, ts.risk, marker_style('--', style), 'DisplayName', [label ' risk'], 'LineWidth', 1.0);
    end
    nexttile(1);
    yline(cfg.guidance.alignRadius, ':', 'Align radius', 'HandleVisibility', 'off');
    yline(cfg.guidance.finalLandRadius, '--', 'Final radius', 'HandleVisibility', 'off');
    ylabel('Error (m)');
    title('Distance to moving landing pad');
    grid on;
    legend('Location', 'best');
    nexttile(2);
    ylabel('Step (m)');
    title('Generated approach step');
    grid on;
    legend('Location', 'best');
    nexttile(3);
    xlabel('Time (s)');
    ylabel('Ontology score');
    title('Track quality and operational risk');
    ylim([0 1]);
    grid on;
    legend('Location', 'best');
    save_figure_if_needed(gcf, runDir, 'path_tracking_time_series', cfg);

    plot_topdown_paths(scenarioMetrics, cfg, runDir, visibleFlag);
end

function plot_topdown_paths(scenarioMetrics, cfg, runDir, visibleFlag)
    scenarioCount = min(numel(scenarioMetrics), 4);
    if scenarioCount == 0
        return;
    end
    figure('Name', 'Generated Path Top-down View', 'Color', 'w', 'Visible', visibleFlag);
    rows = ceil(scenarioCount / 2);
    cols = min(2, scenarioCount);
    layout = tiledlayout(rows, cols, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, 'Top-down UAV, landing pad, and generated target paths');
    for scenarioIdx = 1:scenarioCount
        ts = scenarioMetrics(scenarioIdx).timeSeries;
        nexttile;
        plot_xy(ts.pad, 'g-', 'Landing pad'); hold on;
        plot_xy(ts.target, 'm--', 'Generated target');
        plot_xy(ts.uav, 'b-', 'UAV');
        mark_endpoint(ts.pad, 'go');
        mark_endpoint(ts.target, 'mo');
        mark_endpoint(ts.uav, 'bo');
        xlabel('X (m)');
        ylabel('Y (m)');
        title(sprintf('Scenario %d', scenarioMetrics(scenarioIdx).scenarioIndex));
        axis equal;
        grid on;
        legend('Location', 'best');
    end
    save_figure_if_needed(gcf, runDir, 'path_topdown_comparison', cfg);
end

function plot_reinforcement_reward_results(results, cfg, runDir, visibleFlag)
    if ~isfield(results, 'reinforcementTraining') || ...
            ~isfield(results.reinforcementTraining, 'episodes') || isempty(results.reinforcementTraining.episodes)
        return;
    end
    episodes = results.reinforcementTraining.episodes;
    figure('Name', 'Reinforcement Reward Smoothness', 'Color', 'w', 'Visible', visibleFlag);
    layout = tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(layout, 'Smoothness-aware reinforcement reward');
    for episodeIdx = 1:numel(episodes)
        episode = episodes(episodeIdx);
        label = sprintf('S%d', episode.scenarioIndex);
        style = plot_trace_style(episode.time);
        nexttile(1);
        plot(episode.time, episode.reward, style, 'DisplayName', label, 'LineWidth', 1.1); hold on;
        nexttile(2);
        plot(episode.time, episode.components.smoothness, style, 'DisplayName', [label ' smoothness'], 'LineWidth', 1.1); hold on;
        plot(episode.time, episode.components.tracking, marker_style('--', style), 'DisplayName', [label ' tracking'], 'LineWidth', 1.0);
    end
    nexttile(1);
    ylabel('Reward');
    title('Per-step RL reward');
    grid on;
    legend('Location', 'best');
    nexttile(2);
    xlabel('Time (s)');
    ylabel('Component score');
    ylim([0 1]);
    title('Smoothness and tracking reward components');
    grid on;
    legend('Location', 'best');
    save_figure_if_needed(gcf, runDir, 'reinforcement_reward_smoothness', cfg);
end

function plot_xy(points, style, labelText)
    if isempty(points) || size(points, 2) < 2
        return;
    end
    valid = all(isfinite(points(:, 1:2)), 2);
    if ~any(valid)
        return;
    end
    plot(points(valid, 1), points(valid, 2), style, 'DisplayName', labelText, 'LineWidth', 1.2);
end

function style = marker_style(baseStyle, traceStyle)
    style = baseStyle;
    if contains(traceStyle, 'o')
        style = [baseStyle 'o'];
    end
end

function mark_endpoint(points, style)
    if isempty(points) || size(points, 2) < 2
        return;
    end
    validIdx = find(all(isfinite(points(:, 1:2)), 2), 1, 'last');
    if isempty(validIdx)
        return;
    end
    plot(points(validIdx, 1), points(validIdx, 2), style, ...
        'MarkerFaceColor', style(1), 'HandleVisibility', 'off');
end

end
