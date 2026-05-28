function sim = init_simulator_interface(cfg)
%INIT_SIMULATOR_INTERFACE Initialize a simulator adapter or mock backend.

sim = struct();
sim.backend = cfg.simulation.backend;
sim.namespace = cfg.simulator.namespace;
sim.isConnected = false;
sim.currentScenario = 0;
sim.currentStep = 0;
sim.lastDecision = struct('action', 'HOVER', 'confidence', 0.0, 'probability', [1.0, 0.0]);
sim.guidance = struct('lastCommandTimestamp', -inf, 'landModeSent', false, ...
    'lastTarget', [nan, nan, nan], 'mode', 'HOVER');
sim.previousWind = cfg.wind.meanVelocity;
sim.truth = struct();
sim.state = struct();

if strcmpi(sim.backend, 'mock')
    fprintf('[simulator_interface] Mock backend selected. Placeholder mode enabled.\n');
    return;
end

fprintf('[simulator_interface] Backend "%s" is configured, but no concrete API is wired yet.\n', sim.backend);
fprintf('[simulator_interface] Falling back to mock-style execution while keeping the interface stable.\n');
