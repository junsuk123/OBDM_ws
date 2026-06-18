function selection = select_guidance_algorithms(ontologyGraph, cfg)
%SELECT_GUIDANCE_ALGORITHMS Select modular XY and altitude guidance laws.

selection = struct();
selection.xyAlgorithm = get_cfg_string(cfg, 'defaultXYGuidanceAlgorithm', 'Vision_FOV_Constrained_Guidance');
selection.altitudeAlgorithm = get_cfg_string(cfg, 'defaultAltitudeGuidanceAlgorithm', 'Ontology_Gated_PD_Descent');

if isstruct(ontologyGraph) && isfield(ontologyGraph, 'values')
    values = ontologyGraph.values;
    if isfield(values, 'selectedXYAlgorithm') && ~isempty(values.selectedXYAlgorithm)
        selection.xyAlgorithm = char(string(values.selectedXYAlgorithm));
    end
    if isfield(values, 'selectedAltitudeAlgorithm') && ~isempty(values.selectedAltitudeAlgorithm)
        selection.altitudeAlgorithm = char(string(values.selectedAltitudeAlgorithm));
    end
    if isfield(values, 'xyGuidanceScores')
        selection.xyScores = values.xyGuidanceScores;
    else
        selection.xyScores = struct();
    end
    if isfield(values, 'altitudeGuidanceScores')
        selection.altitudeScores = values.altitudeGuidanceScores;
    else
        selection.altitudeScores = struct();
    end
else
    selection.xyScores = struct();
    selection.altitudeScores = struct();
end

selection.applyAltitudeSafetyFilter = get_cfg_value(cfg, 'altitudeSafetyFilterEnabled', true);
end

function value = get_cfg_string(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = char(string(cfg.guidance.(fieldName)));
end
end

function value = get_cfg_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'guidance') && isfield(cfg.guidance, fieldName) && ~isempty(cfg.guidance.(fieldName))
    value = cfg.guidance.(fieldName);
end
end
