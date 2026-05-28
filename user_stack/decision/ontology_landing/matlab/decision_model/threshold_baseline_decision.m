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

score = 0.30 * (1.0 - windRisk) + 0.20 * visualStability + 0.20 * alignmentState + 0.15 * attitudeStability + 0.15 * descentStability;
predictedLabel = double(score >= cfg.decision.baselineThreshold);

predictedAction = cell(numel(predictedLabel), 1);
for idx = 1:numel(predictedLabel)
    if predictedLabel(idx) == 1
        predictedAction{idx} = 'LAND';
    else
        predictedAction{idx} = 'HOVER';
    end
end

if numel(predictedLabel) == 1
    predictedLabel = predictedLabel(1);
    score = score(1);
    predictedAction = predictedAction{1};
end