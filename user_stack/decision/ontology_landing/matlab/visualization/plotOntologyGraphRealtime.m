function plotOntologyGraphRealtime(ontologyGraph, cfg)
%PLOTONTOLOGYGRAPHRUNTIME Visualize the live ontology relation graph in MATLAB.

persistent figHandle layoutHandle graphAxesHandle xyAxesHandle

if nargin < 2
    cfg = struct();
end
if isempty(ontologyGraph) || ~isstruct(ontologyGraph)
    return;
end
if ~isfield(ontologyGraph, 'nodes') || ~isfield(ontologyGraph, 'edges')
    return;
end
if ~usejava('desktop')
    return;
end

visibleFlag = 'off';
if isfield(cfg, 'visualization') && isfield(cfg.visualization, 'visible') && cfg.visualization.visible
    visibleFlag = 'on';
end

timestamp = NaN;
if isfield(ontologyGraph, 'timestamp')
    timestamp = ontologyGraph.timestamp;
end

if isempty(figHandle) || ~isvalid(figHandle) || isempty(graphAxesHandle) || ~isvalid(graphAxesHandle)
    figHandle = figure( ...
        'Name', 'Ontology Graph Realtime', ...
        'Color', 'w', ...
        'Visible', visibleFlag, ...
        'NumberTitle', 'off');
    layoutHandle = tiledlayout(figHandle, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    graphAxesHandle = nexttile(layoutHandle);
    xyAxesHandle = nexttile(layoutHandle);
else
    if strcmpi(figHandle.Visible, 'off') && strcmpi(visibleFlag, 'on')
        figHandle.Visible = 'on';
    end
end

nodeNames = string({ontologyGraph.nodes.name});
if isempty(nodeNames)
    return;
end

nodeKinds = string({ontologyGraph.nodes.kind});
activations = [ontologyGraph.nodes.activation];

edgeSources = string({ontologyGraph.edges.source});
edgeTargets = string({ontologyGraph.edges.target});
edgeLabels = build_edge_labels(ontologyGraph.edges);

nodeLabels = arrayfun(@(idx) sprintf('%s\n%s\n%.2f', nodeNames(idx), nodeKinds(idx), activations(idx)), ...
    1:numel(nodeNames), 'UniformOutput', false);

cla(graphAxesHandle);
hold(graphAxesHandle, 'on');
grid(graphAxesHandle, 'on');
axis(graphAxesHandle, 'off');

if isempty(edgeSources)
    graph = digraph([], [], [], cellstr(nodeNames));
else
    graph = digraph(cellstr(edgeSources), cellstr(edgeTargets), [], cellstr(nodeNames));
end

graphHandle = plot(graphAxesHandle, graph, ...
    'Layout', 'layered', ...
    'Direction', 'right', ...
    'NodeLabel', nodeLabels, ...
    'EdgeLabel', edgeLabels, ...
    'MarkerSize', 7, ...
    'LineWidth', 1.2);

if ~isempty(activations)
    graphHandle.NodeCData = activations(:);
    colormap(graphAxesHandle, turbo);
    existingColorbars = findall(figHandle, 'Type', 'ColorBar');
    delete(existingColorbars);
    colorbar(graphAxesHandle);
end

subtitleText = sprintf('nodes: %d   edges: %d', numel(nodeNames), numel(edgeSources));
if isfinite(timestamp)
    subtitleText = sprintf('%s   timestamp: %.2f', subtitleText, timestamp);
end
title(graphAxesHandle, {'Live ontology graph relations', subtitleText});

if isfield(ontologyGraph, 'values') && isstruct(ontologyGraph.values)
    infoText = sprintf( ...
        'intent: %.2f   risk: %.2f   track: %.2f   eff: %.2f   lead: %.2f s', ...
        get_value(ontologyGraph.values, 'landingIntent'), ...
        get_value(ontologyGraph.values, 'operationalRisk'), ...
        get_value(ontologyGraph.values, 'trackQuality'), ...
        get_value(ontologyGraph.values, 'efficiency'), ...
        get_value(ontologyGraph.values, 'leadTime'));
    text(graphAxesHandle, 0.01, 0.01, infoText, ...
        'Units', 'normalized', ...
        'VerticalAlignment', 'bottom', ...
        'FontName', 'monospaced', ...
        'FontSize', 9, ...
        'Color', [0.25 0.25 0.25]);
end

plot_xy_motion_panel(xyAxesHandle, ontologyGraph);

drawnow limitrate nocallbacks;
end

function labels = build_edge_labels(edges)
labels = cell(1, numel(edges));
for idx = 1:numel(edges)
    relation = '';
    weight = 1.0;
    if isfield(edges(idx), 'relation') && ~isempty(edges(idx).relation)
        relation = char(string(edges(idx).relation));
    end
    if isfield(edges(idx), 'weight') && ~isempty(edges(idx).weight)
        weight = double(edges(idx).weight);
    end
    labels{idx} = sprintf('%s (%.2f)', relation, weight);
end
end

function plot_xy_motion_panel(ax, ontologyGraph)
cla(ax);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');

uavXY = get_xy_pair(ontologyGraph, 'uavPosition');
markerXY = get_xy_pair(ontologyGraph, 'markerPosition');
futureXY = get_xy_pair(ontologyGraph, 'approachAimPoint');
if any(isnan(uavXY)) || any(isnan(markerXY))
    title(ax, 'XY motion view unavailable');
    axis(ax, 'off');
    return;
end

span = max(1.0, norm(markerXY - uavXY));
pad = 0.9 * span;
xlim(ax, [min(uavXY(1), markerXY(1)) - pad, max(uavXY(1), markerXY(1)) + pad]);
ylim(ax, [min(uavXY(2), markerXY(2)) - pad, max(uavXY(2), markerXY(2)) + pad]);

plot(ax, uavXY(1), uavXY(2), 'o', 'MarkerSize', 9, 'MarkerFaceColor', [0.20 0.45 0.90], 'MarkerEdgeColor', 'k');
plot(ax, markerXY(1), markerXY(2), 's', 'MarkerSize', 9, 'MarkerFaceColor', [0.15 0.70 0.30], 'MarkerEdgeColor', 'k');
plot(ax, [uavXY(1) markerXY(1)], [uavXY(2) markerXY(2)], '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 1.2);

if all(isfinite(futureXY))
    plot(ax, [markerXY(1) futureXY(1)], [markerXY(2) futureXY(2)], '--', 'Color', [0.90 0.35 0.10], 'LineWidth', 1.4);
    quiver(ax, markerXY(1), markerXY(2), futureXY(1) - markerXY(1), futureXY(2) - markerXY(2), 0, ...
        'Color', [0.90 0.35 0.10], 'LineWidth', 1.6, 'MaxHeadSize', 0.45);
end

quiver(ax, uavXY(1), uavXY(2), markerXY(1) - uavXY(1), markerXY(2) - uavXY(2), 0, ...
    'Color', [0.20 0.45 0.90], 'LineWidth', 1.7, 'MaxHeadSize', 0.45);

text(ax, uavXY(1), uavXY(2), '  UAV', 'FontWeight', 'bold', 'Color', [0.05 0.20 0.55]);
text(ax, markerXY(1), markerXY(2), '  PAD', 'FontWeight', 'bold', 'Color', [0.05 0.45 0.15]);
if all(isfinite(futureXY))
    text(ax, futureXY(1), futureXY(2), '  AIM', 'FontWeight', 'bold', 'Color', [0.65 0.20 0.05]);
end

title(ax, 'XY movement direction');
xlabel(ax, 'X');
ylabel(ax, 'Y');
legend(ax, {'UAV', 'Pad', 'UAV to Pad', 'Pad to Aim'}, 'Location', 'bestoutside');
end

function xy = get_xy_pair(ontologyGraph, fieldName)
xy = [NaN, NaN];
if ~isfield(ontologyGraph, fieldName) || isempty(ontologyGraph.(fieldName))
    return;
end
value = ontologyGraph.(fieldName);
if numel(value) >= 2
    xy = double(value(1:2));
end
end

function value = get_value(values, fieldName)
value = NaN;
if isfield(values, fieldName) && ~isempty(values.(fieldName))
    value = values.(fieldName);
end
end
