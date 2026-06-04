function [predictedLabel, score, predictedAction] = threshold_baseline_decision(semanticInput, cfg)
%THRESHOLD_BASELINE_DECISION Simple ontology threshold baseline.

if isstruct(semanticInput)
    semanticMatrix = semanticInput.vector;
else
    semanticMatrix = semanticInput;
end

if isvector(semanticMatrix)
    semanticMatrix = semanticMatrix(:)';
end

windRisk = semanticMatrix(:, 1);
visualStability = semanticMatrix(:, 2);
alignmentState = semanticMatrix(:, 3);
attitudeStability = semanticMatrix(:, 4);
descentStability = semanticMatrix(:, 5);
energyEfficiency = get_semantic_column(semanticMatrix, 8, 0.5);
relativeDistanceState = get_semantic_column(semanticMatrix, 9, alignmentState);
altitudeState = get_semantic_column(semanticMatrix, 10, 0.5);
droneMotionStability = get_semantic_column(semanticMatrix, 11, attitudeStability);
roverMotionStability = get_semantic_column(semanticMatrix, 12, 0.5);

score = 0.18 * (1.0 - windRisk) + 0.14 * visualStability + ...
    0.14 * alignmentState + 0.10 * attitudeStability + 0.10 * descentStability + ...
    0.14 * relativeDistanceState + 0.08 * altitudeState + ...
    0.06 * droneMotionStability + 0.06 * roverMotionStability + 0.04 * energyEfficiency;
predictedLabel = double(score >= cfg.decision.baselineThreshold);

predictedAction = cell(numel(predictedLabel), 1);
for idx = 1:numel(predictedLabel)
    if predictedLabel(idx) == 1
        predictedAction{idx} = 'LAND';
    else
        predictedAction{idx} = 'HOVER';
    end
end

if isscalar(predictedLabel)
    predictedLabel = predictedLabel(1);
    score = score(1);
    predictedAction = predictedAction{1};
end
end

function value = get_semantic_column(semanticMatrix, columnIndex, defaultValue)
if size(semanticMatrix, 2) >= columnIndex
    value = semanticMatrix(:, columnIndex);
elseif isscalar(defaultValue)
    value = repmat(defaultValue, size(semanticMatrix, 1), 1);
else
    value = defaultValue;
end
end
