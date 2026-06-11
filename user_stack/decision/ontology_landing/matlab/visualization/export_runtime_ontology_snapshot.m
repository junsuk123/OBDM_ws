function export_runtime_ontology_snapshot(uavState, ~, observation, semantic, classProbability, confidence, action, cfg)
%EXPORT_RUNTIME_ONTOLOGY_SNAPSHOT Stream one ontology snapshot from the main loop.

if nargin < 8 || ~isstruct(cfg) || ~isfield(cfg, 'visualization') || ...
        ~isfield(cfg.visualization, 'web3dEnabled') || ~cfg.visualization.web3dEnabled
    return;
end

try
    markerPosition = get_vector_field(observation, 'markerPosition', [0.0, 0.0, 0.0]);
    if any(isnan(markerPosition(1:2)))
        markerPosition = [0.0, 0.0, 0.0];
    end
    markerVelocity = get_vector_field(observation, 'markerVelocity', [0.0, 0.0, 0.0]);
    if any(isnan(markerVelocity))
        markerVelocity = [0.0, 0.0, 0.0];
    end

    uavVelocity = get_vector_field(uavState, 'velocity', [0.0, 0.0, 0.0]);
    rel = markerPosition(1:2) - uavState.position(1:2);
    relVel = markerVelocity(1:2) - uavVelocity(1:2);
    range = norm(rel);
    closingSpeed = max(0.2, -dot(rel, relVel) / max(range, eps));

    ontologyGraph = evaluate_landing_ontology_graph(markerPosition, markerVelocity, uavState, ...
        semantic, classProbability, confidence, range, closingSpeed, cfg);
    if isfield(uavState, 'timestamp') && ~isempty(uavState.timestamp)
        ontologyGraph.timestamp = uavState.timestamp;
    end

    trajectory = struct();
    trajectory.distanceToMarker = range;
    trajectory.target = [ontologyGraph.values.approachAimPoint, uavState.position(3)];
    trajectory.finalLand = false;
    export_ontology_graph_snapshot(ontologyGraph, cfg, trajectory, action, classProbability, confidence);
catch exportError
    warning('ontology_viewer:RuntimeExportFailed', ...
        '[ontology_viewer] Failed to export runtime graph snapshot: %s', exportError.message);
end
end

function value = get_vector_field(source, fieldName, defaultValue)
value = defaultValue;
if isstruct(source) && isfield(source, fieldName) && numel(source.(fieldName)) >= numel(defaultValue)
    fieldValue = source.(fieldName);
    value = fieldValue(1:numel(defaultValue));
end
end
