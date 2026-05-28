function [predictedClass, posteriorProbabilities, confidenceScore, action] = predictLandingDecision(model, featureVector, cfg)
%PREDICTLANDINGDECISION CamelCase wrapper for Gaussian Naive Bayes prediction.

[predictedClass, posteriorProbabilities, confidenceScore, action] = predict_gaussian_nb(model, featureVector);