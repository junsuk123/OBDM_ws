function contact = detect_landing_contact(uavState, observation, cfg, takeoffConfirmed)
%DETECT_LANDING_CONTACT Classify pad contact success vs ground contact failure.

if nargin < 4
    takeoffConfirmed = true;
end

contact = default_contact();
contact.altitude = contact_altitude(uavState, observation);
contact.positionError = observation.positionError;
contact.verticalSpeed = observation.verticalSpeed;

if ~takeoffConfirmed
    contact.source = 'pre_takeoff_guard';
    return;
end

touchdownAltitude = get_labeling_value(cfg, 'touchdownAltitude', cfg.labeling.safeAltitude);
padPositionError = get_labeling_value(cfg, 'touchdownPositionError', cfg.labeling.safePositionError);
groundAltitude = get_labeling_value(cfg, 'touchdownGroundAltitude', touchdownAltitude);
groundPositionError = get_labeling_value(cfg, 'touchdownGroundPositionError', padPositionError);
groundResetPositionError = get_labeling_value(cfg, 'touchdownGroundResetPositionError', groundPositionError);
maxAbsVz = get_labeling_value(cfg, 'touchdownMaxAbsVz', cfg.labeling.safeVerticalSpeed);
allowPadThresholdFallback = get_labeling_value(cfg, 'allowTouchdownThresholdFallback', false);
if ~allowPadThresholdFallback
    fallbackEnv = getenv('OBDM_ALLOW_TOUCHDOWN_THRESHOLD_FALLBACK');
    allowPadThresholdFallback = strcmpi(strtrim(fallbackEnv), '1') || strcmpi(strtrim(fallbackEnv), 'true');
end

padContact = contact.altitude <= touchdownAltitude && ...
    contact.positionError <= padPositionError && ...
    abs(contact.verticalSpeed) <= max(maxAbsVz, 0.05);
groundContact = contact.altitude <= groundAltitude && ...
    contact.positionError <= groundResetPositionError;

if allowPadThresholdFallback && padContact
    contact.detected = true;
    contact.type = 'pad';
    contact.success = true;
    contact.source = 'pad_threshold_current';
    return;
end

if groundContact && ~padContact
    contact.detected = true;
    contact.type = 'ground';
    contact.success = false;
    contact.source = 'ground_altitude_immediate';
    return;
end

if ~padContact && ~groundContact
    contact.source = 'none';
    return;
end

[metrics, metricsOk] = read_touchdown_metrics();
touchdownTopic = read_touchdown_bool();
metricPadContact = metricsOk && ...
    metrics(1) <= touchdownAltitude && ...
    metrics(2) <= padPositionError && ...
    abs(metrics(3)) <= max(maxAbsVz, 0.05);
metricGroundContact = metricsOk && metrics(1) <= groundAltitude && ...
    metrics(2) <= groundResetPositionError;
metricsMatchCurrent = metricsOk && ...
    abs(metrics(1) - contact.altitude) <= 0.35 && ...
    abs(metrics(2) - contact.positionError) <= max(0.50, 0.5 * padPositionError);

if touchdownTopic && padContact
    contact.detected = true;
    contact.type = 'pad';
    contact.success = true;
    contact.source = 'touchdown_topic_verified_pad';
    if metricPadContact && metricsMatchCurrent
        contact.altitude = metrics(1);
        contact.positionError = metrics(2);
        contact.verticalSpeed = metrics(3);
    end
elseif touchdownTopic && groundContact
    contact.detected = true;
    contact.type = 'ground';
    contact.success = false;
    contact.source = 'touchdown_topic_verified_ground';
    if metricGroundContact && metricsMatchCurrent
        contact.altitude = metrics(1);
        contact.positionError = metrics(2);
        contact.verticalSpeed = metrics(3);
    end
elseif groundContact
    contact.detected = true;
    contact.type = 'ground';
    contact.success = false;
    contact.source = 'ground_altitude_candidate';
else
    contact.source = 'none';
end
end

function altitude = contact_altitude(uavState, observation)
altitude = NaN;
if isstruct(uavState) && isfield(uavState, 'localAltitude') && isfinite_scalar(uavState.localAltitude)
    altitude = uavState.localAltitude;
elseif isstruct(uavState) && isfield(uavState, 'altitude') && isfinite_scalar(uavState.altitude)
    altitude = uavState.altitude;
elseif isstruct(observation) && isfield(observation, 'altitude') && isfinite_scalar(observation.altitude)
    altitude = observation.altitude;
end
end

function tf = isfinite_scalar(value)
tf = isnumeric(value) && isscalar(value) && isfinite(value);
end

function contact = default_contact()
contact = struct( ...
    'detected', false, ...
    'type', 'none', ...
    'success', false, ...
    'source', 'none', ...
    'scenarioIndex', NaN, ...
    'time', NaN, ...
    'altitude', NaN, ...
    'positionError', NaN, ...
    'verticalSpeed', NaN);
end

function isTouchdown = read_touchdown_bool()
isTouchdown = false;
thisDir = fileparts(mfilename('fullpath'));
workspaceRoot = locate_repo_root_local(thisDir);
reader = fullfile(workspaceRoot, 'scripts', 'diagnostics', 'read_ros_bool_once.py');
if ~exist(reader, 'file')
    return;
end
cmd = sprintf(['timeout 0.45 bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
    'export ROS_DOMAIN_ID=0; python3 "%s" --topic "/landing_marker/touchdown" --timeout 0.20 2>/dev/null'''], reader);
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

function value = get_labeling_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'labeling') && isfield(cfg.labeling, fieldName) && ~isempty(cfg.labeling.(fieldName))
    value = cfg.labeling.(fieldName);
end
end
