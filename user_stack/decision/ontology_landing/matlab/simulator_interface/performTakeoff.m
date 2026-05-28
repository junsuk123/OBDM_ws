function sim = performTakeoff(sim, cfg)
%PERFORMTAKEOFF Execute the initial takeoff sequence.
% Reference integrated_main.m for the actual command timing and altitude.

sim.takeoff = struct();
sim.takeoff.requestedAltitude = cfg.simulator.takeoffAltitude;
sim.takeoff.hoverDuration = cfg.simulator.initialHoverDuration;
sim.takeoff.completed = false;

ns = cfg.simulator.namespace;
if isfield(sim, 'namespace') && ~isempty(sim.namespace)
    ns = sim.namespace;
end

if ~isfield(cfg.simulation, 'autoLaunch') || ~cfg.simulation.autoLaunch
    fprintf('[simulator_interface] performTakeoff skipped: auto launch disabled\n');
    return;
end

if (isfield(sim, 'backend') && strcmpi(strtrim(sim.backend), 'mock')) || ...
        (isfield(sim, 'isConnected') && ~sim.isConnected)
    fprintf('[simulator_interface] performTakeoff skipped: placeholder simulator backend is active\n');
    return;
end

if ~wait_for_takeoff_state_stream(ns)
    fprintf('[simulator_interface] takeoff skipped: %s odometry stream is not ready\n', ns);
    return;
end

if run_mavlink_command('arm_takeoff', sim.takeoff.requestedAltitude, 0, 0, 45, sim, cfg)
    pause(max(0.5, sim.takeoff.hoverDuration));
    sim.takeoff.completed = true;
    fprintf('[simulator_interface] takeoff command accepted for %s (altitude %.2f m)\n', ns, sim.takeoff.requestedAltitude);
else
    fprintf('[simulator_interface] takeoff command failed for %s\n', ns);
end
end

function ready = wait_for_takeoff_state_stream(ns)
ready = false;
odomTopic = sprintf('%s/odometry', ns);
fprintf('[simulator_interface] waiting for %s before takeoff...\n', odomTopic);

repoRoot = locate_repo_root_local(fileparts(mfilename('fullpath')));
arduWs = fullfile(repoRoot, 'ardu_ws');

for attempt = 1:12
    cmd = sprintf(['timeout 6 %s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
        'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
        'export ROS_DOMAIN_ID=0; ' ...
        'ros2 topic echo "%s" nav_msgs/msg/Odometry --once --qos-reliability best_effort --qos-durability volatile >/dev/null 2>&1'''], ...
        clean_env_command_prefix(), arduWs, arduWs, odomTopic);
    [status, ~] = system(cmd);
    if status == 0
        pause(2.0);
        ready = true;
        return;
    end
    fprintf('  %s not ready yet (%d/12)\n', odomTopic, attempt);
    pause(1.5);
end
end

function prefix = clean_env_command_prefix()
keys = {'HOME', 'USER', 'LOGNAME', 'SHELL', 'LANG', 'DISPLAY', ...
    'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', ...
    'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY'};
defaults = struct('SHELL', '/bin/bash', 'LANG', 'C.UTF-8');
parts = {'env -i'};
for i = 1:numel(keys)
    key = keys{i};
    value = getenv(key);
    if isempty(value) && isfield(defaults, key)
        value = defaults.(key);
    end
    if ~isempty(value)
        parts{end+1} = sprintf('%s="%s"', key, regexprep(value, '"', '\\"')); %#ok<AGROW>
    end
end
parts{end+1} = 'PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"';
if isempty(getenv('RMW_IMPLEMENTATION'))
    parts{end+1} = 'RMW_IMPLEMENTATION="rmw_fastrtps_cpp"';
end
if isempty(getenv('ROS_LOCALHOST_ONLY'))
    parts{end+1} = 'ROS_LOCALHOST_ONLY="0"';
end
prefix = strjoin(parts, ' ');
end

function repoRoot = locate_repo_root_local(startDir)
repoRoot = startDir;
for idx = 1:12
    if isfolder(fullfile(repoRoot, 'matlab', 'modules')) && isfolder(fullfile(repoRoot, 'user_stack'))
        return;
    end
    parentDir = fileparts(repoRoot);
    if strcmp(parentDir, repoRoot)
        break;
    end
    repoRoot = parentDir;
end
end
