function semanticFeatures = computeOntologyFeatures(sensorData, windData, cfg)
%COMPUTEONTOLOGYFEATURES CamelCase wrapper for the ontology feature layer.

uavState = sensorData.uavState;
observation = sensorData.observation;
semanticFeatures = compute_ontology_features(uavState, windData, observation, cfg);