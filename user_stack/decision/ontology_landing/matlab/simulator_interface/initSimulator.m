function sim = initSimulator(cfg)
%INITSIMULATOR Initialize simulator connectivity using the integrated_main flow.
% The existing simulator is not rewritten here; this wrapper reuses the
% project-level initialization path that mirrors integrated_main.m.

    persistent initialHardResetDone
    if isempty(initialHardResetDone)
        initialHardResetDone = false;
    end

    launchCfg = cfg;
    forceInitialHardReset = true;
    if isfield(cfg, 'simulation') && isfield(cfg.simulation, 'forceInitialHardReset')
        forceInitialHardReset = logical(cfg.simulation.forceInitialHardReset);
    end
    if forceInitialHardReset && ~initialHardResetDone
        launchCfg.simulation.resetMethod = 'hard';
        initialHardResetDone = true;
        fprintf('[initSimulator] First simulator startup in this MATLAB process: forcing hard reset.\n');
    end

sim = init_simulator_interface(cfg);
projectRoot = fileparts(fileparts(mfilename('fullpath')));
sim = initialize_landing_simulation(sim, launchCfg, projectRoot);
% Warm up wind endpoint/service check before takeoff so wind logs do not
% appear late inside the control loop.
if isfield(cfg, 'simulator') && isfield(cfg.simulator, 'publishWindToGazebo') && cfg.simulator.publishWindToGazebo
    try
        [~, sim] = read_wind_condition(sim, 0.0, cfg.wind.meanVelocity, cfg);
    catch ME
        fprintf('[initSimulator] wind endpoint warm-up skipped: %s\n', ME.message);
    end
end
sim.lastKnownState = struct('position', cfg.simulator.initialPosition, 'altitude', cfg.simulator.initialAltitude);
sim.padModule = [];
