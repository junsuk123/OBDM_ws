function [predictedLabel, classProbability, confidence, predictedAction] = predict_gaussian_nb(model, featureMatrix)
%PREDICT_GAUSSIAN_NB Predict classes and posterior probabilities.

if ~isfield(model, 'isTrained') || ~model.isTrained
    sampleCount = size(featureMatrix, 1);
    predictedLabel = zeros(sampleCount, 1);
    classProbability = repmat([1.0, 0.0], sampleCount, 1);
    confidence = ones(sampleCount, 1);
    predictedAction = repmat({'HOVER'}, sampleCount, 1);
    return;
end

if isvector(featureMatrix)
    featureMatrix = featureMatrix(:)';
end

sampleCount = size(featureMatrix, 1);
if isfield(model, 'featureCount') && model.featureCount ~= size(featureMatrix, 2)
    predictedLabel = zeros(sampleCount, 1);
    classProbability = repmat([1.0, 0.0], sampleCount, 1);
    confidence = ones(sampleCount, 1);
    predictedAction = repmat({'HOVER'}, sampleCount, 1);
    if sampleCount == 1
        predictedLabel = predictedLabel(1);
        classProbability = classProbability(1, :);
        confidence = confidence(1);
        predictedAction = predictedAction{1};
    end
    return;
end
classCount = numel(model.classLabels);
logPosterior = zeros(sampleCount, classCount);

for classIdx = 1:classCount
    mu = model.mean(classIdx, :);
    sigma2 = model.variance(classIdx, :);
    centered = featureMatrix - mu;
    gaussianLogPdf = -0.5 * sum(log(2 * pi * sigma2)) - 0.5 * sum((centered .^ 2) ./ sigma2, 2);
    logPosterior(:, classIdx) = log(model.prior(classIdx) + eps) + gaussianLogPdf;
end

maxLogPosterior = max(logPosterior, [], 2);
stabilized = exp(logPosterior - maxLogPosterior);
classProbability = stabilized ./ sum(stabilized, 2);

[confidence, classIndex] = max(classProbability, [], 2);
predictedLabel = model.classLabels(classIndex)';

predictedAction = cell(sampleCount, 1);
for idx = 1:sampleCount
    if predictedLabel(idx) == 1
        predictedAction{idx} = 'LAND';
    else
        predictedAction{idx} = 'HOVER';
    end
end

if sampleCount == 1
    predictedLabel = predictedLabel(1);
    classProbability = classProbability(1, :);
    confidence = confidence(1);
    predictedAction = predictedAction{1};
end
