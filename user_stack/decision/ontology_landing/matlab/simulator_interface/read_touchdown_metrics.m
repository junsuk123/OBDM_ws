function [metrics, ok] = read_touchdown_metrics()
%READ_TOUCHDOWN_METRICS Read touchdown altitude, horizontal error, and vertical speed.

metrics = [NaN, NaN, NaN];
ok = false;

thisDir = fileparts(mfilename('fullpath'));
workspaceRoot = locate_repo_root_local(thisDir);
reader = fullfile(workspaceRoot, 'scripts', 'diagnostics', 'read_ros_vector3_once.py');
if ~exist(reader, 'file')
    return;
end

cmd = sprintf(['timeout 1.2 bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; python3 "%s" --topic "/landing_marker/touchdown_metrics" --timeout 0.7 2>/dev/null'''], reader);
[status, output] = system(cmd);
if status ~= 0 || isempty(strtrim(output))
    return;
end

values = sscanf(output, '%f %f %f');
if numel(values) ~= 3 || any(isnan(values))
    return;
end

metrics = values(:)';
ok = true;
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
