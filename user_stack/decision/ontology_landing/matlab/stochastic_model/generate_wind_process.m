function windProcess = generate_wind_process(cfg, scenarioIndex)
%GENERATE_WIND_PROCESS Create a time-varying stochastic wind sequence.

timeVector = cfg.simulation.timeVector;
sampleCount = numel(timeVector);

rng(cfg.simulation.seed + 1000 + scenarioIndex, 'twister');

meanVelocity = cfg.wind.meanVelocity;
gust = zeros(sampleCount, 3);
turbulence = zeros(sampleCount, 3);
headingDrift = zeros(sampleCount, 1);
speedDrift = zeros(sampleCount, 1);
windProcess = zeros(sampleCount, 3);

for idx = 1:sampleCount
    if idx == 1
        gust(idx, :) = cfg.wind.gustStd .* randn(1, 3);
        turbulence(idx, :) = 0.5 * cfg.wind.gustStd .* randn(1, 3);
        headingDrift(idx) = cfg.wind.directionChangeStd * randn();
        speedDrift(idx) = 0.08 * randn();
    else
        alpha = exp(-cfg.simulation.dt / max(cfg.wind.turbulenceTau, eps));
        gust(idx, :) = 0.85 * gust(idx - 1, :) + 0.15 * cfg.wind.gustStd .* randn(1, 3);
        turbulence(idx, :) = alpha * turbulence(idx - 1, :) + sqrt(max(0.0, 1.0 - alpha^2)) * cfg.wind.gustStd .* randn(1, 3);
        headingDrift(idx) = alpha * headingDrift(idx - 1) + ...
            sqrt(max(0.0, 1.0 - alpha^2)) * cfg.wind.directionChangeStd * randn();
        speedDrift(idx) = alpha * speedDrift(idx - 1) + ...
            sqrt(max(0.0, 1.0 - alpha^2)) * 0.08 * randn();
    end

    directionDrift = cfg.wind.directionChangeStd * sin(0.25 * timeVector(idx) + scenarioIndex) + headingDrift(idx);
    directionRotation = [cos(directionDrift), -sin(directionDrift), 0; sin(directionDrift), cos(directionDrift), 0; 0, 0, 1];
    speedScale = max(0.55, min(1.25, 1.0 + 0.10 * sin(0.35 * timeVector(idx) + 0.7 * scenarioIndex) + speedDrift(idx)));
    baseWind = speedScale * (directionRotation * meanVelocity(:))';
    windProcess(idx, :) = baseWind + gust(idx, :) + turbulence(idx, :);
end

speed = sqrt(sum(windProcess.^2, 2));
tooStrong = speed > cfg.wind.maxSpeed;
if any(tooStrong)
    windProcess(tooStrong, :) = windProcess(tooStrong, :) .* (cfg.wind.maxSpeed ./ speed(tooStrong));
end
