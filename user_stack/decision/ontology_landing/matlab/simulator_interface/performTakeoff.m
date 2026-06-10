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

if isfield(sim, 'landingStackReady') && ~sim.landingStackReady
    fprintf('[simulator_interface] takeoff blocked: landing stack is not ready\n');
    append_startup_log('takeoff_blocked_stack_not_ready', sprintf('namespace=%s', ns));
    return;
end

if (isfield(sim, 'backend') && strcmpi(strtrim(sim.backend), 'mock')) || ...
        (isfield(sim, 'isConnected') && ~sim.isConnected)
    fprintf('[simulator_interface] performTakeoff skipped: placeholder simulator backend is active\n');
    return;
end

if ~wait_for_takeoff_state_stream(ns)
    fprintf('[simulator_interface] takeoff skipped: %s odometry stream is not ready\n', ns);
    append_startup_log('takeoff_skipped_odom_not_ready', sprintf('namespace=%s', ns));
    return;
end

append_startup_log('takeoff_request_begin', sprintf('namespace=%s altitude=%.2f', ns, sim.takeoff.requestedAltitude));
if run_mavlink_command('arm_takeoff', sim.takeoff.requestedAltitude, 0, 0, 45, sim, cfg)
    pause(min(0.5, max(0.1, sim.takeoff.hoverDuration)));
    sim.takeoff.completed = true;
    append_startup_log('takeoff_hover_complete', sprintf('namespace=%s hover_s=%.2f', ns, sim.takeoff.hoverDuration));
    fprintf('[simulator_interface] takeoff command accepted for %s (altitude %.2f m)\n', ns, sim.takeoff.requestedAltitude);
else
    if takeoff_airborne_by_odometry(ns, sim.takeoff.requestedAltitude)
        pause(min(0.5, max(0.1, sim.takeoff.hoverDuration)));
        sim.takeoff.completed = true;
        append_startup_log('takeoff_hover_complete_odometry_fallback', sprintf('namespace=%s hover_s=%.2f', ns, sim.takeoff.hoverDuration));
        fprintf('[simulator_interface] takeoff command accepted for %s via odometry fallback (altitude %.2f m)\n', ns, sim.takeoff.requestedAltitude);
        return;
    end
    append_startup_log('takeoff_request_failed', sprintf('namespace=%s altitude=%.2f', ns, sim.takeoff.requestedAltitude));
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

function airborne = takeoff_airborne_by_odometry(ns, requestedAltitude)
airborne = false;

odomTopic = sprintf('%s/odometry', ns);
minAltitude = max(0.8, requestedAltitude * 0.08);
cmd = sprintf(['timeout 4 bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; timeout 2 ros2 topic echo --no-daemon --spin-time 0.4 ' ...
    '--qos-reliability best_effort --once --field "pose.pose.position.z" "%s" nav_msgs/msg/Odometry 2>/dev/null'''], ...
    odomTopic);
[status, output] = system(cmd);
if status ~= 0 || isempty(strtrim(output))
    return;
end

altitude = str2double(strtrim(output));
if isnan(altitude)
    return;
end

airborne = altitude >= minAltitude;
if ~airborne && altitude <= -minAltitude
    airborne = true;
end
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

function prefix = clean_env_command_prefix()
keys = {'HOME', 'USER', 'LOGNAME', 'SHELL', 'LANG', 'DISPLAY', ...
    'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', ...
    'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI', ...
    'FASTRTPS_DEFAULT_PROFILES_FILE'};
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
