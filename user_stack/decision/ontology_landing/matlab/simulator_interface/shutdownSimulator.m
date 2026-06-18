function sim = shutdownSimulator(sim, cfg, reason)
%SHUTDOWNSIMULATOR Safely terminate simulator communication and launchers.

if nargin < 1 || isempty(sim)
    sim = struct();
    return;
end
if nargin < 2
    cfg = [];
end
if nargin < 3 || isempty(reason)
    reason = 'shutdown';
end

if isfield(sim, 'runtimeShutdown') && sim.runtimeShutdown
    return;
end

if ~isempty(cfg) && (~isfield(sim, 'resetPrepared') || ~sim.resetPrepared)
    sim = prepare_simulation_reset(sim, cfg, reason, struct());
elseif isfield(sim, 'padModule') && ~isempty(sim.padModule)
    try
        sim.padModule.stop();
    catch ME
        fprintf('[simulator_interface] pad module cleanup warning: %s\n', ME.message);
    end
    sim.padModule = [];
end

sim = close_simulator_interface(sim);
sim.runtimeShutdown = true;
sim.isConnected = false;
