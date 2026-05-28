function xObs = applySensorNoise(xTrue, cfg)
%APPLYSENSORNOISE Model noisy observation x_obs(t) = x_true(t) + n(t).

xObs = xTrue + cfg.sensor.positionNoiseStd * randn(size(xTrue));