function stats = estimateWindStatistics(windLog, cfg)
%ESTIMATEWINDSTATISTICS Estimate stochastic wind characteristics from a log.

if isempty(windLog)
    stats = struct('mean', zeros(1,3), 'variance', zeros(1,3), 'acceleration', zeros(1,3), 'directionChange', 0);
    return;
end

stats = struct();
stats.mean = mean(windLog, 1, 'omitnan');
stats.variance = var(windLog, 0, 1, 'omitnan');
stats.acceleration = [0, 0, 0];
if size(windLog, 1) > 1
    dt = max(cfg.simulation.dt, eps);
    stats.acceleration = mean(diff(windLog, 1, 1), 1, 'omitnan') / dt;
end
speed = sqrt(sum(windLog.^2, 2));
if numel(speed) > 1
    dirVec = windLog ./ max(speed, eps);
    ang = 0;
    for k = 2:size(dirVec,1)
        ang = ang + acos(max(-1, min(1, dot(dirVec(k-1,:), dirVec(k,:)))));
    end
    stats.directionChange = ang / max(size(dirVec,1)-1, 1);
else
    stats.directionChange = 0;
end