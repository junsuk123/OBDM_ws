function sim = initSimulator(cfg)
%INITSIMULATOR Initialize simulator connectivity using the integrated_main flow.
% The existing simulator is not rewritten here; this wrapper reuses the
% project-level initialization path that mirrors integrated_main.m.

sim = init_simulator_interface(cfg);
projectRoot = fileparts(fileparts(mfilename('fullpath')));
sim = initialize_landing_simulation(sim, cfg, projectRoot);
sim.lastKnownState = struct('position', cfg.simulator.initialPosition, 'altitude', cfg.simulator.initialAltitude);
sim.padModule = [];
