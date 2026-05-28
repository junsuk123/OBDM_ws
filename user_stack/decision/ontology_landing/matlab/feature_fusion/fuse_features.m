function fused = fuse_features(rawContinuousVector, semantic, cfg)
%FUSE_FEATURES Normalize and concatenate observation and semantic features.

rawContinuousVector = rawContinuousVector(:)';
rawCount = numel(cfg.featureFusion.rawMin);
if numel(rawContinuousVector) ~= rawCount
    error('Raw feature length mismatch. Expected %d, got %d.', rawCount, numel(rawContinuousVector));
end

rawScaled = zeros(1, rawCount);
for idx = 1:rawCount
    rawScaled(idx) = normalize_feature(rawContinuousVector(idx), cfg.featureFusion.rawMin(idx), cfg.featureFusion.rawMax(idx));
end

semanticVector = semantic.vector(:)';
featureVector = [rawScaled, semanticVector];

fused = struct();
fused.raw = rawScaled;
fused.semantic = semanticVector;
fused.vector = featureVector;
fused.featureNames = [cfg.featureFusion.rawNames, cfg.featureFusion.semanticNames];
fused.ordering = struct('rawNames', {cfg.featureFusion.rawNames}, 'semanticNames', {cfg.featureFusion.semanticNames});