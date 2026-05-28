function sim = close_simulator_interface(sim)
%CLOSE_SIMULATOR_INTERFACE Placeholder cleanup hook for the simulator adapter.

if nargin < 1 || isempty(sim)
    sim = struct();
    return;
end

if isfield(sim, 'runtimeShutdown') && sim.runtimeShutdown
    return;
end

if isfield(sim, 'launcher') && ~isempty(sim.launcher)
    try
        if ismethod(sim.launcher, 'stop_simulation')
            sim.launcher = sim.launcher.stop_simulation();
        end
    catch ME
        fprintf('[simulator_interface] Warning: launcher cleanup failed: %s\n', ME.message);
    end
end

if isfield(sim, 'backend') && ~strcmpi(sim.backend, 'mock')
    fprintf('[simulator_interface] Closing backend connection for %s\n', sim.backend);
end

sim.runtimeShutdown = true;
sim.isConnected = false;
