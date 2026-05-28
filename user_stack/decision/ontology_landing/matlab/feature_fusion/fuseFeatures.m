function featureVector = fuseFeatures(sensorFeatures, semanticFeatures, cfg)
%FUSEFEATURES Combine raw sensor features and ontology semantics.

fused = fuse_features(sensorFeatures, semanticFeatures, cfg);
featureVector = fused.vector;