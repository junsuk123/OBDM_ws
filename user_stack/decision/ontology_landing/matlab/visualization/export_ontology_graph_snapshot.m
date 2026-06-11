function export_ontology_graph_snapshot(ontologyGraph, cfg, trajectory, action, classProbability, confidence)
%EXPORT_ONTOLOGY_GRAPH_SNAPSHOT Append one web-viewer ontology graph snapshot.

if nargin < 2 || ~isstruct(cfg) || ~isfield(cfg, 'visualization')
    return;
end
if ~isfield(cfg.visualization, 'web3dEnabled') || ~cfg.visualization.web3dEnabled
    return;
end
if ~isfield(cfg.visualization, 'web3dStreamPath') || isempty(cfg.visualization.web3dStreamPath)
    return;
end
if nargin < 3
    trajectory = struct();
end
if nargin < 4
    action = '';
end
if nargin < 5 || isempty(classProbability)
    classProbability = [1.0, 0.0];
end
if nargin < 6 || isempty(confidence)
    confidence = max(classProbability);
end
if isempty(ontologyGraph) || ~isstruct(ontologyGraph) || ~isfield(ontologyGraph, 'nodes') || ~isfield(ontologyGraph, 'edges')
    return;
end

snapshot = build_snapshot(ontologyGraph, trajectory, action, classProbability, confidence);
streamPath = char(string(cfg.visualization.web3dStreamPath));
[streamDir, ~, ~] = fileparts(streamPath);
if ~exist(streamDir, 'dir')
    mkdir(streamDir);
end

fid = fopen(streamPath, 'a');
if fid < 0
    warning('[ontology_viewer] Could not write stream: %s', streamPath);
    return;
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(snapshot));
end

function snapshot = build_snapshot(ontologyGraph, trajectory, action, classProbability, confidence)
nodes = empty_node_array();
for idx = 1:numel(ontologyGraph.nodes)
    source = ontologyGraph.nodes(idx);
    name = char(string(source.name));
    activation = clamp01(double(source.activation));
    kind = char(string(source.kind));
    degreeCount = node_degree(name, ontologyGraph.edges);
    item = struct();
    item.id = name;
    item.label = name;
    item.type = kind;
    item.confidence = activation;
    item.score = activation;
    item.importance = clamp01(0.70 * activation + 0.30 * min(1.0, degreeCount / 6.0));
    item.active = activation >= 0.72 || is_terminal_active(name, ontologyGraph);
    item.attributes = struct();
    item.attributes.activation = activation;
    item.attributes.importance = item.importance;
    item.attributes.degree = degreeCount;
    item.attributes.description = char(string(source.description));
    nodes(end + 1, 1) = item; %#ok<AGROW>
end

edges = empty_edge_array();
for idx = 1:numel(ontologyGraph.edges)
    source = ontologyGraph.edges(idx);
    relation = char(string(source.relation));
    sourceName = char(string(source.source));
    targetName = char(string(source.target));
    weight = clamp01(double(source.weight));
    item = struct();
    item.id = sprintf('%s__%s__%s', sourceName, relation, targetName);
    item.source = sourceName;
    item.target = targetName;
    item.label = relation;
    item.type = relation;
    item.confidence = weight;
    item.active = weight >= 0.45 && (node_activation(sourceName, ontologyGraph.nodes) >= 0.65 || ...
        node_activation(targetName, ontologyGraph.nodes) >= 0.65);
    item.attributes = struct();
    item.attributes.weight = weight;
    item.attributes.source_activation = node_activation(sourceName, ontologyGraph.nodes);
    item.attributes.target_activation = node_activation(targetName, ontologyGraph.nodes);
    edges(end + 1, 1) = item; %#ok<AGROW>
end

values = struct();
if isfield(ontologyGraph, 'values') && isstruct(ontologyGraph.values)
    values = ontologyGraph.values;
end
values.action = char(string(action));
values.classProbability = classProbability;
values.confidence = confidence;

selectedNode = choose_selected_node(ontologyGraph, trajectory);
snapshot = struct();
snapshot.stamp = get_graph_timestamp(ontologyGraph);
snapshot.graph_id = get_graph_name(ontologyGraph);
snapshot.nodes = struct_array_to_cell(nodes);
snapshot.edges = struct_array_to_cell(edges);
snapshot.values = values;
snapshot.selected = struct('node_id', selectedNode, 'reason', 'Current most relevant landing ontology state.');
snapshot.reasoning_trace = build_reasoning_trace(ontologyGraph);
snapshot.topdown = build_topdown(ontologyGraph, trajectory);
end

function nodes = empty_node_array()
nodes = struct( ...
    'id', {}, ...
    'label', {}, ...
    'type', {}, ...
    'confidence', {}, ...
    'score', {}, ...
    'importance', {}, ...
    'active', {}, ...
    'attributes', {});
end

function edges = empty_edge_array()
edges = struct( ...
    'id', {}, ...
    'source', {}, ...
    'target', {}, ...
    'label', {}, ...
    'type', {}, ...
    'confidence', {}, ...
    'active', {}, ...
    'attributes', {});
end

function values = struct_array_to_cell(items)
values = cell(1, numel(items));
for idx = 1:numel(items)
    values{idx} = items(idx);
end
end

function trace = build_reasoning_trace(ontologyGraph)
trace = [
    trace_step(1, 'Risk evidence updates OperationalRisk.', ...
        {'WindRisk', 'DragRisk', 'EnergyEfficiency', 'DroneMotionStability', 'RoverMotionStability', 'OperationalRisk'}, ontologyGraph)
    trace_step(2, 'Alignment and visual evidence update TrackQuality.', ...
        {'AlignmentState', 'RelativeDistanceState', 'VisualStability', 'DroneMotionStability', 'TrackQuality'}, ontologyGraph)
    trace_step(3, 'Pad prediction selects the current approach aim point.', ...
        {'PadTrack', 'FuturePadPrediction', 'ApproachAimPoint'}, ontologyGraph)
    trace_step(4, 'Intent, risk, and track quality gate guidance authority.', ...
        {'LandingIntent', 'OperationalRisk', 'TrackQuality', 'HorizontalGuidance', 'DescentAuthority'}, ontologyGraph)
    trace_step(5, 'Terminal readiness combines alignment, descent authority, and risk.', ...
        {'LandingAlignment', 'DescentAuthority', 'TrackQuality', 'OperationalRisk', 'FinalLandReadiness'}, ontologyGraph)
];
end

function item = trace_step(step, description, nodeIds, ontologyGraph)
nodeIds = existing_node_ids(nodeIds, ontologyGraph.nodes);
edgeIds = {};
for idx = 1:numel(ontologyGraph.edges)
    edge = ontologyGraph.edges(idx);
    if any(strcmp(nodeIds, char(string(edge.source)))) && any(strcmp(nodeIds, char(string(edge.target))))
        edgeIds{end + 1} = sprintf('%s__%s__%s', char(string(edge.source)), char(string(edge.relation)), char(string(edge.target))); %#ok<AGROW>
    end
end
confidence = mean_node_activation(nodeIds, ontologyGraph.nodes);
item = struct('step', step, 'description', description, 'nodes', {nodeIds}, 'edges', {edgeIds}, 'confidence', confidence);
end

function filtered = existing_node_ids(nodeIds, nodes)
filtered = {};
for idx = 1:numel(nodeIds)
    if node_exists(nodeIds{idx}, nodes)
        filtered{end + 1} = nodeIds{idx}; %#ok<AGROW>
    end
end
end

function topdown = build_topdown(ontologyGraph, trajectory)
topdown = struct();
topdown.drone = xy_struct(get_vector_field(ontologyGraph, 'uavPosition', [NaN, NaN]));
topdown.pad = xy_struct(get_vector_field(ontologyGraph, 'markerPosition', [NaN, NaN]));
topdown.target = xy_struct(get_vector_field(ontologyGraph, 'approachAimPoint', [NaN, NaN]));
if isfield(trajectory, 'target') && numel(trajectory.target) >= 2
    topdown.target = xy_struct(trajectory.target(1:2));
end
topdown.min_span = 6.0;
topdown.grid_m = 1.0;
end

function item = xy_struct(value)
item = struct('x', value(1), 'y', value(2));
end

function selected = choose_selected_node(ontologyGraph, trajectory)
selected = 'LandingIntent';
if isfield(trajectory, 'finalLand') && trajectory.finalLand
    selected = 'FinalLandReadiness';
elseif isfield(ontologyGraph, 'values') && isfield(ontologyGraph.values, 'descentAuthority') && ...
        ontologyGraph.values.descentAuthority >= 0.5
    selected = 'DescentAuthority';
elseif isfield(ontologyGraph, 'values') && isfield(ontologyGraph.values, 'operationalRisk') && ...
        ontologyGraph.values.operationalRisk >= 0.55
    selected = 'OperationalRisk';
end
if ~node_exists(selected, ontologyGraph.nodes) && ~isempty(ontologyGraph.nodes)
    selected = char(string(ontologyGraph.nodes(1).name));
end
end

function exists = node_exists(name, nodes)
exists = false;
for idx = 1:numel(nodes)
    if strcmp(char(string(nodes(idx).name)), name)
        exists = true;
        return;
    end
end
end

function value = node_activation(name, nodes)
value = 0.0;
for idx = 1:numel(nodes)
    if strcmp(char(string(nodes(idx).name)), name)
        value = clamp01(double(nodes(idx).activation));
        return;
    end
end
end

function value = mean_node_activation(nodeIds, nodes)
if isempty(nodeIds)
    value = 0.0;
    return;
end
values = zeros(1, numel(nodeIds));
for idx = 1:numel(nodeIds)
    values(idx) = node_activation(nodeIds{idx}, nodes);
end
value = clamp01(mean(values));
end

function count = node_degree(name, edges)
count = 0;
for idx = 1:numel(edges)
    if strcmp(char(string(edges(idx).source)), name) || strcmp(char(string(edges(idx).target)), name)
        count = count + 1;
    end
end
end

function active = is_terminal_active(name, ontologyGraph)
active = strcmp(name, 'FinalLandReadiness') && ...
    isfield(ontologyGraph, 'values') && isfield(ontologyGraph.values, 'landingAlignment') && ...
    ontologyGraph.values.landingAlignment >= 0.72;
end

function value = get_graph_timestamp(ontologyGraph)
value = posixtime(datetime('now'));
if isfield(ontologyGraph, 'timestamp') && ~isempty(ontologyGraph.timestamp) && isfinite(double(ontologyGraph.timestamp))
    value = double(ontologyGraph.timestamp);
end
end

function value = get_graph_name(ontologyGraph)
value = 'LandingOntologyTrajectoryGraph';
if isfield(ontologyGraph, 'name') && ~isempty(ontologyGraph.name)
    value = char(string(ontologyGraph.name));
end
end

function value = get_vector_field(source, fieldName, defaultValue)
value = defaultValue;
if isfield(source, fieldName) && numel(source.(fieldName)) >= numel(defaultValue)
    fieldValue = source.(fieldName);
    value = fieldValue(1:numel(defaultValue));
end
end

function value = clamp01(value)
value = max(0.0, min(1.0, value));
end
