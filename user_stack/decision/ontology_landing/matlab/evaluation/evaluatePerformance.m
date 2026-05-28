function metrics = evaluatePerformance(predictions, labels, cfg)
%EVALUATEPERFORMANCE Compute performance metrics for the proposed method.

baseline = predictions;
metrics = evaluate_landing_decisions(labels, predictions, baseline, zeros(size(labels)), cfg).proposed;