function isTouchdown = is_touchdown_on_marker(uavState, observation, cfg)
%IS_TOUCHDOWN_ON_MARKER True when the vehicle has landed on the marker.

isTouchdown = read_touchdown_topic();
if isTouchdown
    return;
end

touchdownAltitude = cfg.labeling.safeAltitude;
if isfield(cfg.labeling, 'touchdownAltitude')
    touchdownAltitude = cfg.labeling.touchdownAltitude;
end

touchdownPositionError = cfg.labeling.safePositionError;
if isfield(cfg.labeling, 'touchdownPositionError')
    touchdownPositionError = cfg.labeling.touchdownPositionError;
end

altitudeOk = uavState.altitude <= touchdownAltitude;
positionOk = observation.positionError <= touchdownPositionError;
verticalOk = abs(observation.verticalSpeed) <= max(cfg.labeling.safeVerticalSpeed, 0.05);

isTouchdown = altitudeOk && positionOk && verticalOk;
end

function isTouchdown = read_touchdown_topic()
isTouchdown = false;
thisDir = fileparts(mfilename('fullpath'));
workspaceRoot = locate_repo_root_local(thisDir);
reader = fullfile(workspaceRoot, 'scripts', 'diagnostics', 'read_ros_bool_once.py');
if ~exist(reader, 'file')
    return;
end
cmd = sprintf(['timeout 1.2 bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; python3 "%s" --topic "/landing_marker/touchdown" --timeout 0.7 2>/dev/null'''], reader);
[status, output] = system(cmd);
if status ~= 0 || isempty(strtrim(output))
    return;
end
text = lower(strtrim(output));
isTouchdown = contains(text, 'true') || strcmp(text, '1');
end

function repoRoot = locate_repo_root_local(startDir)
repoRoot = startDir;
for idx = 1:12
    if isfolder(fullfile(repoRoot, 'scripts', 'diagnostics')) && isfolder(fullfile(repoRoot, 'user_stack'))
        return;
    end
    parentDir = fileparts(repoRoot);
    if strcmp(parentDir, repoRoot)
        break;
    end
    repoRoot = parentDir;
end
end
