function plotGuidanceAlgorithmRealtime(ontologyGraph, cfg)
%PLOTGUIDANCEALGORITHMREALTIME Show live guidance-law inference activations.

persistent figHandle layoutHandle trendAxesHandle barAxesHandle parameterAxesHandle history

if nargin < 2
    cfg = struct();
end
if isempty(ontologyGraph) || ~isstruct(ontologyGraph) || ...
        ~isfield(ontologyGraph, 'values') || ~isstruct(ontologyGraph.values)
    return;
end
if ~isfield(ontologyGraph.values, 'guidanceAlgorithmScores')
    return;
end
if ~usejava('desktop')
    return;
end

visibleFlag = 'off';
if isfield(cfg, 'visualization') && isfield(cfg.visualization, 'visible') && cfg.visualization.visible
    visibleFlag = 'on';
end

[algorithmNames, activations] = guidance_activations(ontologyGraph.values.guidanceAlgorithmScores);
if isempty(algorithmNames)
    return;
end
selected = selected_algorithm(ontologyGraph.values, algorithmNames, activations);
timestamp = graph_timestamp(ontologyGraph);

if isempty(history) || ~isstruct(history) || ~isfield(history, 'activation') || ~isequal(history.names, algorithmNames)
    history = struct();
    history.names = algorithmNames;
    history.time = [];
    history.activation = zeros(0, numel(algorithmNames));
    history.selected = strings(0, 1);
end

history.time(end + 1, 1) = timestamp;
history.activation(end + 1, :) = activations(:)';
history.selected(end + 1, 1) = string(selected);
maxSamples = 240;
if numel(history.time) > maxSamples
    keepIdx = (numel(history.time) - maxSamples + 1):numel(history.time);
    history.time = history.time(keepIdx);
    history.activation = history.activation(keepIdx, :);
    history.selected = history.selected(keepIdx);
end

if isempty(figHandle) || ~isvalid(figHandle) || isempty(trendAxesHandle) || ~isvalid(trendAxesHandle)
    figHandle = figure( ...
        'Name', 'Guidance Algorithm Inference Realtime', ...
        'Color', 'w', ...
        'Visible', visibleFlag, ...
        'NumberTitle', 'off');
    layoutHandle = tiledlayout(figHandle, 3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    trendAxesHandle = nexttile(layoutHandle);
    barAxesHandle = nexttile(layoutHandle);
    parameterAxesHandle = nexttile(layoutHandle);
else
    if strcmpi(figHandle.Visible, 'off') && strcmpi(visibleFlag, 'on')
        figHandle.Visible = 'on';
    end
end

plot_activation_history(trendAxesHandle, history, selected);
plot_current_activations(barAxesHandle, algorithmNames, activations, selected);
plot_selected_parameters(parameterAxesHandle, ontologyGraph.values);
title(layoutHandle, sprintf('Guidance selection: %s   t = %.2f s', readable_name(selected), timestamp));
drawnow limitrate nocallbacks;
end

function [names, activations] = guidance_activations(scores)
names = string(fieldnames(scores));
if isempty(names)
    activations = [];
    return;
end
raw = zeros(numel(names), 1);
for idx = 1:numel(names)
    value = scores.(char(names(idx)));
    if isnumeric(value) && isscalar(value) && isfinite(value)
        raw(idx) = max(0.0, double(value));
    end
end
if sum(raw) <= eps
    activations = ones(numel(raw), 1) ./ max(numel(raw), 1);
else
    activations = raw ./ sum(raw);
end
[names, order] = sort(names);
activations = activations(order);
end

function selected = selected_algorithm(values, names, activations)
if isfield(values, 'selectedGuidanceAlgorithm') && ~isempty(values.selectedGuidanceAlgorithm)
    selected = char(string(values.selectedGuidanceAlgorithm));
    return;
end
[~, idx] = max(activations);
selected = char(names(idx));
end

function timestamp = graph_timestamp(ontologyGraph)
timestamp = 0.0;
if isfield(ontologyGraph, 'timestamp') && ~isempty(ontologyGraph.timestamp) && isfinite(double(ontologyGraph.timestamp))
    timestamp = double(ontologyGraph.timestamp);
end
end

function plot_activation_history(ax, history, selected)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
if isempty(history.time)
    return;
end
time = history.time - history.time(1);
colors = lines(numel(history.names));
selectedIdx = find(history.names == string(selected), 1);
for idx = 1:numel(history.names)
    lineWidth = 1.0;
    if ~isempty(selectedIdx) && idx == selectedIdx
        lineWidth = 2.8;
    end
    plot(ax, time, history.activation(:, idx), ...
        'Color', colors(idx, :), ...
        'LineWidth', lineWidth, ...
        'DisplayName', readable_name(history.names(idx)));
end
ylim(ax, [0.0, 1.0]);
xlabel(ax, 'time [s]');
ylabel(ax, 'inference activation');
title(ax, 'Guidance algorithm activation history');
legend(ax, 'Location', 'eastoutside', 'Interpreter', 'none');
end

function plot_current_activations(ax, names, activations, selected)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
selectedIdx = find(names == string(selected), 1);
colors = repmat([0.32 0.42 0.52], numel(names), 1);
if ~isempty(selectedIdx)
    colors(selectedIdx, :) = [0.10 0.55 0.85];
end
barHandle = bar(ax, activations, 'FaceColor', 'flat');
barHandle.CData = colors;
ylim(ax, [0.0, max(0.25, min(1.0, max(activations) + 0.08))]);
xticks(ax, 1:numel(names));
xticklabels(ax, arrayfun(@readable_name, names, 'UniformOutput', false));
xtickangle(ax, 35);
ylabel(ax, 'current activation');
title(ax, 'Current guidance inference distribution');
for idx = 1:numel(names)
    text(ax, idx, activations(idx) + 0.015, sprintf('%.2f', activations(idx)), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 8);
end
end

function plot_selected_parameters(ax, values)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
if ~isfield(values, 'selectedGuidanceParameters') || ~isstruct(values.selectedGuidanceParameters)
    axis(ax, 'off');
    return;
end
params = values.selectedGuidanceParameters;
names = ["lookahead", "horizon", "control", "safety", "wind"];
fields = ["lookaheadActivation", "horizonActivation", "controlActivation", "safetyActivation", "windActivation"];
activations = zeros(numel(fields), 1);
for idx = 1:numel(fields)
    if isfield(params, char(fields(idx))) && isnumeric(params.(char(fields(idx))))
        activations(idx) = max(0.0, min(1.0, double(params.(char(fields(idx))))));
    end
end
barHandle = bar(ax, activations, 'FaceColor', 'flat');
barHandle.CData = [
    0.12 0.48 0.74
    0.25 0.58 0.42
    0.78 0.46 0.18
    0.70 0.22 0.28
    0.42 0.36 0.70
];
ylim(ax, [0.0, 1.0]);
xticks(ax, 1:numel(names));
xticklabels(ax, names);
ylabel(ax, 'activation');
title(ax, 'Selected guidance parameter setting');
for idx = 1:numel(names)
    text(ax, idx, activations(idx) + 0.03, sprintf('%.2f', activations(idx)), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 8);
end
end

function textValue = readable_name(name)
name = char(string(name));
name = regexprep(name, 'Guidance$', '');
name = strrep(name, 'NMPC_CBF', 'NMPC+CBF');
name = regexprep(name, '([a-z])([A-Z])', '$1 $2');
textValue = name;
end
