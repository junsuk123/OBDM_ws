function model = train_gaussian_nb(featureMatrix, labelVector, cfg)
%TRAIN_GAUSSIAN_NB Fit a Gaussian Naive Bayes classifier from labeled data.

if isempty(featureMatrix) || isempty(labelVector)
    model = struct('isTrained', false);
    return;
end

labelVector = labelVector(:);
classes = cfg.decision.classLabels(:)';
featureCount = size(featureMatrix, 2);
sampleCount = size(featureMatrix, 1);

meanMatrix = zeros(numel(classes), featureCount);
varianceMatrix = zeros(numel(classes), featureCount);
priorVector = zeros(1, numel(classes));

for classIdx = 1:numel(classes)
    classLabel = classes(classIdx);
    classMask = labelVector == classLabel;
    classSamples = featureMatrix(classMask, :);
    classCount = size(classSamples, 1);

    if classCount == 0
        meanMatrix(classIdx, :) = zeros(1, featureCount);
        varianceMatrix(classIdx, :) = ones(1, featureCount);
        priorVector(classIdx) = eps;
        continue;
    end

    meanMatrix(classIdx, :) = mean(classSamples, 1);
    varianceMatrix(classIdx, :) = var(classSamples, 0, 1);
    varianceMatrix(classIdx, varianceMatrix(classIdx, :) < cfg.decision.epsilon) = cfg.decision.epsilon;
    priorVector(classIdx) = classCount / sampleCount;
end

priorVector = priorVector / sum(priorVector);

model = struct();
model.isTrained = true;
model.classLabels = classes;
model.classNames = cfg.decision.classNames;
model.prior = priorVector;
model.mean = meanMatrix;
model.variance = varianceMatrix;
model.epsilon = cfg.decision.epsilon;
model.featureCount = featureCount;
model.sampleCount = sampleCount;