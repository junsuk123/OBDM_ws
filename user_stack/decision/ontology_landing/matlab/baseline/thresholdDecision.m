function [decision, score] = thresholdDecision(sensorData, semanticFeatures, cfg)
%THRESHOLDDECISION Conventional threshold baseline for comparison.

if nargin < 2 || isempty(semanticFeatures)
    semanticFeatures = sensorData.semantic;
end

[label, scoreValue, action] = threshold_baseline_decision(semanticFeatures, cfg);
decision = struct('label', label, 'score', scoreValue, 'action', action);
score = scoreValue;