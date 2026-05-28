function [windState, sim] = read_wind_condition(sim, timestamp, windVector, cfg)
%READ_WIND_CONDITION Build wind disturbance features from the current sample.

if isempty(windVector)
    windVector = cfg.wind.meanVelocity;
end

previousWind = sim.previousWind;
dt = max(cfg.simulation.dt, eps);

speed = norm(windVector);
previousSpeed = norm(previousWind);
acceleration = (speed - previousSpeed) / dt;

currentDirection = windVector / max(speed, eps);
previousDirection = previousWind / max(previousSpeed, eps);
directionChange = acos(max(-1.0, min(1.0, dot(currentDirection, previousDirection))));

windState = struct();
windState.timestamp = timestamp;
windState.vector = windVector;
windState.speed = speed;
windState.direction = currentDirection;
windState.acceleration = acceleration;
windState.directionChange = directionChange;
windState.previousVector = previousWind;
windState.isMock = true;
windState.source = 'generated';

if isfield(sim, 'isConnected') && sim.isConnected && ...
        isfield(cfg.simulator, 'publishWindToGazebo') && cfg.simulator.publishWindToGazebo
    sim = verify_gazebo_wind_endpoint(sim, cfg);
    [published, sim] = publish_gazebo_wind(sim, timestamp, windVector, cfg);
    if published
        windState.isMock = false;
        windState.source = 'generated_and_published_to_gazebo';
    end
end

sim.previousWind = windVector;

end

function sim = verify_gazebo_wind_endpoint(sim, cfg)
if isfield(sim, 'windEndpointChecked') && sim.windEndpointChecked
    return;
end
sim.windEndpointChecked = true;

worldName = 'runway';
if isfield(cfg.simulator, 'gazeboWorldName') && ~isempty(cfg.simulator.gazeboWorldName)
    worldName = cfg.simulator.gazeboWorldName;
end
gzVersion = 'harmonic';
if isfield(cfg.simulation, 'gzVersion') && ~isempty(cfg.simulation.gzVersion)
    gzVersion = cfg.simulation.gzVersion;
end

cmd = sprintf(['bash -lc ''export GZ_VERSION=%s; ' ...
    'timeout 2 gz service -s /world/%s/wind_info --reqtype gz.msgs.Empty ' ...
    '--reptype gz.msgs.Wind --timeout 1500 --req "" >/dev/null 2>&1'''], ...
    gzVersion, worldName);
[status, ~] = system(cmd);
if status == 0
    fprintf('[simulator_interface] Gazebo WindEffects/wind_info endpoint is available for world "%s".\n', worldName);
else
    fprintf('[simulator_interface] Warning: Gazebo wind_info endpoint did not respond for world "%s"; wind topic publish will still be attempted.\n', worldName);
end
end

function [published, sim] = publish_gazebo_wind(sim, timestamp, windVector, cfg)
published = false;

period = 1.0;
if isfield(cfg.simulator, 'windPublishPeriod')
    period = cfg.simulator.windPublishPeriod;
end

if isfield(sim, 'lastWindPublishTimestamp') && ...
        (timestamp - sim.lastWindPublishTimestamp) < period
    return;
end

worldName = 'runway';
if isfield(cfg.simulator, 'gazeboWorldName') && ~isempty(cfg.simulator.gazeboWorldName)
    worldName = cfg.simulator.gazeboWorldName;
end

topic = sprintf('/world/%s/wind', worldName);
payload = sprintf('linear_velocity: {x: %.8f, y: %.8f, z: %.8f}, enable_wind: true', ...
    windVector(1), windVector(2), windVector(3));
gzVersion = 'harmonic';
if isfield(cfg.simulation, 'gzVersion') && ~isempty(cfg.simulation.gzVersion)
    gzVersion = cfg.simulation.gzVersion;
end
cmd = sprintf(['bash -lc ''export GZ_VERSION=%s; ' ...
    'timeout 1 gz topic -t "%s" -m gz.msgs.Wind -p "%s" >/dev/null 2>&1'''], ...
    gzVersion, topic, payload);
[status, ~] = system(cmd);
published = status == 0;
if published
    sim.lastWindPublishTimestamp = timestamp;
end
end
