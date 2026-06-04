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
sim.lastKnownState = struct('position', cfg.simulator.initialPosition, 'altitude', cfg.simulator.initialAltitude);
sim.padModule = [];
