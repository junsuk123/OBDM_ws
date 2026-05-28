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
    return;
end

figure('Name', 'Landing Scenario Summary', 'Color', 'w', 'Visible', visibleFlag);
layout = tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
title(layout, 'Landing scenario result summary');

nexttile;
plot(timeVector, results.altitudeVector, 'LineWidth', 1.4); hold on;
if isfield(results, 'positionErrorVector') && ~isempty(results.positionErrorVector)
    plot(timeVector, results.positionErrorVector, 'LineWidth', 1.2);
end
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('State value');
legend({'Altitude (m)', 'Position error (m)'}, 'Location', 'best');
title('Final approach state');
grid on;

nexttile;
stairs(timeVector, results.proposedLabels, 'LineWidth', 1.4); hold on;
stairs(timeVector, results.trueLabels, 'LineWidth', 1.0);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Class label');
yticks([0 1]);
yticklabels({'HOVER', 'LAND'});
legend({'Proposed', 'True'}, 'Location', 'best');
title('Decision trace');
grid on;
ylim([-0.1 1.1]);

nexttile;
metricNames = categorical({'Accuracy', 'Precision', 'Recall', 'F1', 'Unsafe'});
metricNames = reordercats(metricNames, {'Accuracy', 'Precision', 'Recall', 'F1', 'Unsafe'});
metricValues = [
    results.proposed.accuracy, results.baseline.accuracy;
    results.proposed.precision, results.baseline.precision;
    results.proposed.recall, results.baseline.recall;
    results.proposed.f1Score, results.baseline.f1Score;
    results.proposed.unsafeLandingAttemptRate, results.baseline.unsafeLandingAttemptRate];
bar(metricNames, metricValues);
ylabel('Score');
legend({'Proposed', 'Baseline'}, 'Location', 'best');
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
plot(timeVector, results.windSpeedVector, 'LineWidth', 1.4);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Wind speed (m/s)');
title('Wind profile over time');
grid on;
save_figure_if_needed(gcf, runDir, 'wind_profile', cfg);

figure('Name', 'UAV Attitude and Altitude', 'Color', 'w', 'Visible', visibleFlag);
subplot(2, 1, 1);
plot(timeVector, results.altitudeVector, 'LineWidth', 1.4);
add_touchdown_marker(results);
ylabel('Altitude (m)');
title('Altitude');
grid on;
subplot(2, 1, 2);
plot(timeVector, rad2deg(results.rollVector), 'LineWidth', 1.2); hold on;
plot(timeVector, rad2deg(results.pitchVector), 'LineWidth', 1.2);
plot(timeVector, rad2deg(results.yawVector), 'LineWidth', 1.2);
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
plot(timeVector, results.semanticMatrix(:, 1:semanticCount), 'LineWidth', 1.2);
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
stairs(timeVector, results.proposedLabels, 'LineWidth', 1.4); hold on;
stairs(timeVector, results.baselineLabels, 'LineWidth', 1.2);
stairs(timeVector, results.trueLabels, 'LineWidth', 1.0);
add_touchdown_marker(results);
xlabel('Time (s)');
ylabel('Class label');
legend({'Proposed', 'Baseline', 'True'}, 'Location', 'best');
title('Decision timeline');
grid on;
ylim([-0.1 1.1]);
save_figure_if_needed(gcf, runDir, 'decision_timeline', cfg);

figure('Name', 'Confusion Matrix', 'Color', 'w', 'Visible', visibleFlag);
subplot(1, 2, 1);
plot_confusion_matrix(results.proposed.confusionMatrix, 'Proposed');
subplot(1, 2, 2);
plot_confusion_matrix(results.baseline.confusionMatrix, 'Baseline');
save_figure_if_needed(gcf, runDir, 'confusion_matrix', cfg);

figure('Name', 'Wind Bin Analysis', 'Color', 'w', 'Visible', visibleFlag);
binCount = numel(results.windBinAnalysis);
proposedLandingRate = zeros(binCount, 1);
baselineLandingRate = zeros(binCount, 1);
binLabels = cell(binCount, 1);
for idx = 1:binCount
    binLabels{idx} = sprintf('%.0f-%.0f', results.windBinAnalysis(idx).lowerEdge, results.windBinAnalysis(idx).upperEdge);
    proposedLandingRate(idx) = results.windBinAnalysis(idx).proposed.recall;
    baselineLandingRate(idx) = results.windBinAnalysis(idx).baseline.recall;
end
bar([proposedLandingRate, baselineLandingRate]);
xlabel('Wind-speed bin');
ylabel('Landing recall');
legend({'Proposed', 'Baseline'}, 'Location', 'best');
title('Landing performance by wind-speed bin');
grid on;
set(gca, 'XTickLabel', binLabels);
    save_figure_if_needed(gcf, runDir, 'wind_bin_analysis', cfg);

function summaryLines = build_summary_lines(results, cfg)
    summaryLines = {
        sprintf('Completed scenarios : %d / %d', results.completedScenarioCount, cfg.simulation.numScenarios)
        sprintf('Samples evaluated   : %d', results.sampleCount)
        sprintf('Touchdown detected  : %d', logical_to_double(results.touchdown.detected))
        sprintf('Proposed accuracy   : %.3f', results.proposed.accuracy)
        sprintf('Baseline accuracy   : %.3f', results.baseline.accuracy)
        sprintf('Proposed unsafe rate: %.3f', results.proposed.unsafeLandingAttemptRate)
        sprintf('Baseline unsafe rate: %.3f', results.baseline.unsafeLandingAttemptRate)
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
    pngPath = fullfile(runDir, [baseName '.png']);
    saveas(figHandle, pngPath);
end

end
