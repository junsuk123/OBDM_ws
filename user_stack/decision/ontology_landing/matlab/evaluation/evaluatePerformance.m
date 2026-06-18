function metrics = evaluatePerformance(predictions, labels, cfg)
%EVALUATEPERFORMANCE Compute performance metrics for ontology decisions.

metrics = evaluate_landing_decisions(labels, predictions, zeros(size(labels)), cfg).ontology;
