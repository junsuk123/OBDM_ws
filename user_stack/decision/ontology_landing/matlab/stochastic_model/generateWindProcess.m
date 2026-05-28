function windProcess = generateWindProcess(cfg, t)
%GENERATEWINDPROCESS CamelCase wrapper for the requested API.

if nargin < 2
    t = cfg.simulation.timeVector;
end
scenarioIndex = 1;
windProcess = generate_wind_process(cfg, scenarioIndex);
if numel(t) ~= size(windProcess, 1)
    windProcess = windProcess(1:min(numel(t), size(windProcess, 1)), :);
end