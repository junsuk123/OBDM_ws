function sim = prepare_simulation_reset(sim, cfg, reason, contact)
%PREPARE_SIMULATION_RESET Give MATLAB control before any simulator reset.
% The ordering is deliberate: stop moving actors first, then terminate
% MATLAB-owned helper processes, and only then let SimulationManager reset.

if nargin < 1 || isempty(sim)
    sim = struct();
end
if nargin < 2 || isempty(cfg)
    cfg = landing_decision_config();
end
if nargin < 3 || isempty(reason)
    reason = 'unspecified';
end
if nargin < 4 || isempty(contact)
    contact = struct();
end

if isfield(sim, 'resetPrepared') && sim.resetPrepared
    return;
end

log_runtime_control('prepare_begin', reason, contact);
fprintf('[simulator_interface] MATLAB runtime control: preparing reset (%s)\n', reason);
try
    ProcessManager.mark_soft_reset_unhealthy();
catch
end

sim = stopLandingPadMotion(sim);
pause(get_reset_control_pause(cfg, 'padStopSettleSeconds', 0.20));

[sim, disarmOk] = stop_drone_motion_for_reset(sim, cfg, reason, contact);
pause(get_reset_control_pause(cfg, 'droneStopSettleSeconds', 0.20));

sim = cleanup_matlab_owned_runtime(sim, cfg);

sim.resetPrepared = true;
sim.resetPreparedReason = char(string(reason));
sim.resetPreparedDisarmOk = disarmOk;
log_runtime_control('prepare_complete', reason, contact);
fprintf('[simulator_interface] MATLAB runtime control: reset preparation complete (disarmOk=%d)\n', disarmOk);
end

function [sim, disarmOk] = stop_drone_motion_for_reset(sim, cfg, reason, contact)
disarmOk = false;
publish_guidance_stop(cfg);

try
    disarmOk = run_mavlink_command('disarm', 0, 0, 0, ...
        get_reset_control_pause(cfg, 'disarmTimeoutSeconds', 4.0), sim, cfg);
catch ME
    fprintf('[simulator_interface] drone disarm warning: %s\n', ME.message);
end

if ~disarmOk && is_pad_success(contact)
    try
        disarmOk = run_mavlink_command('land_disarm', 0, 0, 0, ...
            get_reset_control_pause(cfg, 'landDisarmTimeoutSeconds', 6.0), sim, cfg);
    catch ME
        fprintf('[simulator_interface] land_disarm warning: %s\n', ME.message);
    end
end

log_runtime_control(sprintf('drone_stop disarmOk=%d', disarmOk), reason, contact);
end

function publish_guidance_stop(cfg)
domainId = 0;
if isfield(cfg, 'simulation') && isfield(cfg.simulation, 'rosDomainId') && ~isempty(cfg.simulation.rosDomainId)
    domainId = cfg.simulation.rosDomainId;
end
cmd = sprintf(['timeout 2 bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=%d; ' ...
    'ros2 topic pub --once /obdm/guidance_land std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1 || true; ' ...
    'ros2 topic pub --once /model/r1_rover/cmd_vel geometry_msgs/msg/Twist ' ...
    '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true'''], ...
    domainId);
system(cmd);
end

function sim = cleanup_matlab_owned_runtime(sim, cfg)
grace_s = get_reset_control_pause(cfg, 'runtimeCleanupGraceSeconds', 0.35);
try
    if has_process_manager(sim)
        sim.launcher.process_manager = sim.launcher.process_manager.cleanup_ros_runtime_processes(grace_s);
    else
        pm = ProcessManager('verbose', false);
        pm = pm.cleanup_ros_runtime_processes(grace_s); %#ok<NASGU>
    end
catch ME
    fprintf('[simulator_interface] MATLAB-owned runtime cleanup warning: %s\n', ME.message);
end
end

function tf = has_process_manager(sim)
tf = false;
if ~isfield(sim, 'launcher') || isempty(sim.launcher)
    return;
end
if isobject(sim.launcher)
    tf = isprop(sim.launcher, 'process_manager') && ~isempty(sim.launcher.process_manager);
elseif isstruct(sim.launcher)
    tf = isfield(sim.launcher, 'process_manager') && ~isempty(sim.launcher.process_manager);
end
end

function tf = is_pad_success(contact)
tf = isstruct(contact) && isfield(contact, 'success') && logical(contact.success);
end

function value = get_reset_control_pause(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'simulator') && isfield(cfg.simulator, 'resetControl') && ...
        isfield(cfg.simulator.resetControl, fieldName) && ~isempty(cfg.simulator.resetControl.(fieldName))
    value = double(cfg.simulator.resetControl.(fieldName));
end
end

function log_runtime_control(eventName, reason, contact)
try
    fid = fopen('/tmp/obdm_matlab_runtime_control.log', 'a');
    if fid < 0
        return;
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
    xy = NaN;
    z = NaN;
    source = 'none';
    if isstruct(contact)
        if isfield(contact, 'positionError'), xy = contact.positionError; end
        if isfield(contact, 'altitude'), z = contact.altitude; end
        if isfield(contact, 'source'), source = char(string(contact.source)); end
    end
    fprintf(fid, '%s event=%s reason=%s source=%s xy=%.4f z=%.4f matlab_pid=%d\n', ...
        datestr(now, 'yyyy-mm-ddTHH:MM:SS.FFF'), eventName, char(string(reason)), source, xy, z, feature('getpid'));
catch
end
end
