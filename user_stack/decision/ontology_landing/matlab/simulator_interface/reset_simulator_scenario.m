function sim = reset_simulator_scenario(sim, scenarioIndex, cfg)
%RESET_SIMULATOR_SCENARIO Reset the adapter state for a new scenario.

sim.currentScenario = scenarioIndex;
sim.currentStep = 0;
sim.previousWind = cfg.wind.meanVelocity;
sim.lastDecision = struct('action', 'HOVER', 'confidence', 0.0, 'probability', [1.0, 0.0]);
sim.guidance = struct('lastCommandTimestamp', -inf, 'landModeSent', false, ...
    'lastTarget', [nan, nan, nan], 'mode', 'HOVER');

rng(cfg.simulation.seed + scenarioIndex, 'twister');

sim.truth.position = [cfg.simulator.initialPosition(1), cfg.simulator.initialPosition(2), cfg.simulator.initialAltitude];
sim.truth.velocity = [0.0, 0.0, -cfg.simulator.nominalDescentRate];
sim.truth.attitude = [0.0, 0.0, 0.0];
sim.truth.altitude = cfg.simulator.initialAltitude;
sim.truth.timestamp = 0.0;

sim.state = sim.truth;
append_startup_log('scenario_reset', sprintf('scenario=%d', scenarioIndex));
end

function append_startup_log(eventName, message)
try
    fid = fopen('/tmp/obdm_startup_sequence.log', 'a');
    if fid < 0
        return;
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s event=%s message=%s\n', datestr(now, 'yyyy-mm-ddTHH:MM:SS.FFF'), eventName, message);
catch
end
end
