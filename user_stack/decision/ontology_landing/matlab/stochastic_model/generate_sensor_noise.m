function noiseValue = generate_sensor_noise(cfg, noiseType)
%GENERATE_SENSOR_NOISE Return a scalar Gaussian noise sample for the given sensor channel.

switch lower(char(noiseType))
    case 'altitude'
        sigma = cfg.sensor.positionNoiseStd * 0.25;
    case 'attitude'
        sigma = deg2rad(cfg.sensor.attitudeNoiseStdDeg);
    case 'tag_center'
        sigma = cfg.sensor.tagCenterNoiseStd;
    case 'stability'
        sigma = cfg.sensor.stabilityNoiseStd;
    otherwise
        sigma = cfg.sensor.positionNoiseStd;
end

noiseValue = sigma * randn();