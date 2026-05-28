function normalizedValue = normalize_feature(value, minValue, maxValue)
%NORMALIZE_FEATURE Clamp a scalar value into the [0, 1] interval.

if nargin < 3 || maxValue == minValue
    normalizedValue = 0.0;
    return;
end

normalizedValue = (value - minValue) / (maxValue - minValue);
normalizedValue = max(0.0, min(1.0, normalizedValue));