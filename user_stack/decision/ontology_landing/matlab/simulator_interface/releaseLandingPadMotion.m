function sim = releaseLandingPadMotion(sim, cfg)
%RELEASELANDINGPADMOTION Allow the rover target to move after startup/takeoff gates.

if nargin < 1 || isempty(sim)
    sim = struct();
    return;
end
if nargin < 2
    cfg = struct();
end

topicName = '/obdm/start_pad_motion';
if isfield(sim, 'padModule') && ~isempty(sim.padModule) && ...
        isprop(sim.padModule, 'motion_enable_topic') && ~isempty(sim.padModule.motion_enable_topic)
    topicName = sim.padModule.motion_enable_topic;
end

if ~isfield(sim, 'takeoff') || ~isfield(sim.takeoff, 'completed') || ~sim.takeoff.completed
    append_startup_log('rover_motion_release_blocked', sprintf('topic=%s reason=takeoff_not_completed', topicName));
    fprintf('[simulator_interface] rover motion release blocked: takeoff has not completed\n');
    if isfield(sim, 'padMotion')
        sim.padMotion.released = false;
    end
    return;
end

if isfield(sim, 'padModule') && ~isempty(sim.padModule)
    try
        sim.padModule.release_motion();
        append_startup_log('rover_motion_released', sprintf('topic=%s', topicName));
        if isfield(sim, 'padMotion')
            sim.padMotion.released = true;
        end
        return;
    catch ME
        fprintf('[simulator_interface] padModule release warning: %s\n', ME.message);
    end
end

arduWs = locate_ardu_ws_local();
cmd = sprintf(['timeout 4 bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
    'export ROS_DOMAIN_ID=%d; ros2 topic pub --once "%s" std_msgs/msg/Bool "{data: true}" ' ...
    '--qos-reliability reliable --qos-durability transient_local >/tmp/obdm_rover_motion_release.log 2>&1'''], ...
    arduWs, arduWs, get_ros_domain(cfg), topicName);
[status, ~] = system(cmd);
if status == 0 || status == 124
    append_startup_log('rover_motion_released', sprintf('topic=%s fallback=1', topicName));
    fprintf('[simulator_interface] rover motion released on %s\n', topicName);
else
    append_startup_log('rover_motion_release_failed', sprintf('topic=%s status=%d', topicName, status));
    fprintf('[simulator_interface] rover motion release failed; check /tmp/obdm_rover_motion_release.log\n');
end

if isfield(sim, 'padMotion')
    sim.padMotion.released = status == 0 || status == 124;
end
end

function value = get_ros_domain(cfg)
value = 0;
if isfield(cfg, 'simulation') && isfield(cfg.simulation, 'rosDomainId') && ~isempty(cfg.simulation.rosDomainId)
    value = cfg.simulation.rosDomainId;
end
end

function arduWs = locate_ardu_ws_local()
repoRoot = locate_repo_root_local(fileparts(mfilename('fullpath')));
arduWs = fullfile(repoRoot, 'ardu_ws');
end

function repoRoot = locate_repo_root_local(startDir)
repoRoot = startDir;
for idx = 1:12
    if isfolder(fullfile(repoRoot, 'matlab')) && isfolder(fullfile(repoRoot, 'user_stack'))
        return;
    end
    parentDir = fileparts(repoRoot);
    if strcmp(parentDir, repoRoot)
        break;
    end
    repoRoot = parentDir;
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
