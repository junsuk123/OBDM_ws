function results = evaluate_landing_decisions(trueLabels, proposedLabels, baselineLabels, windSpeedVector, cfg)
%EVALUATE_LANDING_DECISIONS Compare the proposed classifier against a baseline.

trueLabels = trueLabels(:);
proposedLabels = proposedLabels(:);
baselineLabels = baselineLabels(:);
windSpeedVector = windSpeedVector(:);

results = struct();
results.proposed = compute_metrics(trueLabels, proposedLabels, cfg);
results.baseline = compute_metrics(trueLabels, baselineLabels, cfg);
results.windBinAnalysis = compute_wind_bin_analysis(trueLabels, proposedLabels, baselineLabels, windSpeedVector, cfg);
results.classOrder = cfg.decision.classLabels;
results.windBins = cfg.evaluation.windBins;
results.trueLabels = trueLabels;
results.proposedLabels = proposedLabels;
results.baselineLabels = baselineLabels;

function metrics = compute_metrics(actualLabels, predictedLabels, cfg)
    positiveLabel = cfg.evaluation.positiveClassLabel;
    [confusionMatrix, accuracy, precision, recall, f1Score] = compute_binary_metrics(actualLabels, predictedLabels, positiveLabel);

    falsePositive = confusionMatrix(1, 2);
    totalPredictedPositive = sum(confusionMatrix(:, 2));
    if totalPredictedPositive > 0
        unsafeLandingAttemptRate = falsePositive / totalPredictedPositive;
    else
        unsafeLandingAttemptRate = 0.0;
    end

    metrics = struct();
    metrics.confusionMatrix = confusionMatrix;
    metrics.accuracy = accuracy;
    metrics.precision = precision;
    metrics.recall = recall;
    metrics.f1Score = f1Score;
    metrics.unsafeLandingAttemptRate = unsafeLandingAttemptRate;
end

function analysis = compute_wind_bin_analysis(actualLabels, proposedLabels, baselineLabels, windVector, cfg)
    edges = cfg.evaluation.windBins;
    binCount = numel(edges) - 1;
    analysis = repmat(struct(), binCount, 1);

    for binIdx = 1:binCount
        lowerEdge = edges(binIdx);
        upperEdge = edges(binIdx + 1);
        if isinf(upperEdge)
            mask = windVector >= lowerEdge;
        else
            mask = windVector >= lowerEdge & windVector < upperEdge;
        end

        [proposedMetrics, baselineMetrics] = compute_bin_metrics(actualLabels(mask), proposedLabels(mask), baselineLabels(mask), cfg);
        analysis(binIdx).lowerEdge = lowerEdge;
        analysis(binIdx).upperEdge = upperEdge;
        analysis(binIdx).count = sum(mask);
        analysis(binIdx).proposed = proposedMetrics;
        analysis(binIdx).baseline = baselineMetrics;
    end
end

function [proposedMetrics, baselineMetrics] = compute_bin_metrics(actualLabels, proposedLabels, baselineLabels, cfg)
    if isempty(actualLabels)
        proposedMetrics = compute_metrics_for_empty();
        baselineMetrics = compute_metrics_for_empty();
        return;
    end
    proposedMetrics = compute_metrics(actualLabels, proposedLabels, cfg);
    baselineMetrics = compute_metrics(actualLabels, baselineLabels, cfg);
end

function metrics = compute_metrics_for_empty()
    metrics = struct();
    metrics.confusionMatrix = zeros(2, 2);
    metrics.accuracy = 0.0;
    metrics.precision = 0.0;
    metrics.recall = 0.0;
    metrics.f1Score = 0.0;
    metrics.unsafeLandingAttemptRate = 0.0;
end

function [confusionMatrix, accuracy, precision, recall, f1Score] = compute_binary_metrics(actualLabels, predictedLabels, positiveLabel)
    confusionMatrix = zeros(2, 2);
    for idx = 1:numel(actualLabels)
        actualIndex = actualLabels(idx) + 1;
        predictedIndex = predictedLabels(idx) + 1;
        confusionMatrix(actualIndex, predictedIndex) = confusionMatrix(actualIndex, predictedIndex) + 1;
    end

    total = sum(confusionMatrix(:));
    if total == 0
        accuracy = 0.0;
    else
        accuracy = trace(confusionMatrix) / total;
    end

    positiveIndex = positiveLabel + 1;
    truePositive = confusionMatrix(positiveIndex, positiveIndex);
    falsePositive = sum(confusionMatrix(:, positiveIndex)) - truePositive;
    falseNegative = sum(confusionMatrix(positiveIndex, :)) - truePositive;
    if (truePositive + falsePositive) > 0
        precision = truePositive / (truePositive + falsePositive);
    else
        precision = 0.0;
    end
    if (truePositive + falseNegative) > 0
        recall = truePositive / (truePositive + falseNegative);
    else
        recall = 0.0;
    end
    if (precision + recall) > 0
        f1Score = 2.0 * precision * recall / (precision + recall);
    else
        f1Score = 0.0;
    end
end

end