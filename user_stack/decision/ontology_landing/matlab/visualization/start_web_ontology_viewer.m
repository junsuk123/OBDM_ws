function cfg = start_web_ontology_viewer(cfg)
%START_WEB_ONTOLOGY_VIEWER Start the local 3D ontology viewer and open Chrome.

if nargin < 1 || ~isstruct(cfg)
    cfg = struct();
end
if ~isfield(cfg, 'visualization') || ~isstruct(cfg.visualization)
    cfg.visualization = struct();
end
if isfield(cfg.visualization, 'web3dEnabled') && ~cfg.visualization.web3dEnabled
    return;
end

repoRoot = get_cfg_string(cfg, {'project', 'root'}, pwd);
outputDir = get_cfg_string(cfg, {'results', 'outputDir'}, fullfile(repoRoot, 'results'));
streamDir = fullfile(outputDir, 'ontology_viz');
if ~exist(streamDir, 'dir')
    mkdir(streamDir);
end

streamPath = fullfile(streamDir, 'live_graph.jsonl');
fid = fopen(streamPath, 'w');
if fid >= 0
    fclose(fid);
end

host = get_visualization_value(cfg, 'web3dHost', '127.0.0.1');
port = get_visualization_value(cfg, 'web3dPort', 8765);
pythonCmd = get_visualization_value(cfg, 'web3dPythonCommand', 'python3');
backendPath = fullfile(repoRoot, 'ontology_viz', 'backend', 'graph_provider.py');
frontendPath = fullfile(repoRoot, 'ontology_viz', 'frontend');
logPath = fullfile(streamDir, 'viewer_server.log');
viewerUrl = sprintf('http://%s:%d/viewer', host, port);

stop_existing_viewer_server(port);

command = sprintf(['cd "%s" && nohup %s "%s" --snapshot "%s" --frontend "%s" ' ...
    '--host "%s" --port %d --watch-interval 0.2 > "%s" 2>&1 &'], ...
    repoRoot, pythonCmd, backendPath, streamPath, frontendPath, host, port, logPath);
[status, message] = system(command);
if status ~= 0
    warning('[ontology_viewer] Failed to start web server: %s', strtrim(message));
else
    fprintf('[ontology_viewer] 3D ontology stream: %s\n', streamPath);
    fprintf('[ontology_viewer] 3D ontology viewer: %s\n', viewerUrl);
end

if get_visualization_value(cfg, 'web3dOpenChrome', true)
    pause(0.8);
    open_browser(viewerUrl);
end

cfg.visualization.web3dEnabled = true;
cfg.visualization.web3dStreamPath = streamPath;
cfg.visualization.web3dViewerUrl = viewerUrl;
cfg.visualization.web3dServerLog = logPath;
end

function value = get_visualization_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'visualization') && isfield(cfg.visualization, fieldName) && ~isempty(cfg.visualization.(fieldName))
    value = cfg.visualization.(fieldName);
end
end

function value = get_cfg_string(cfg, pathParts, defaultValue)
value = defaultValue;
cursor = cfg;
for idx = 1:numel(pathParts)
    key = pathParts{idx};
    if ~isstruct(cursor) || ~isfield(cursor, key) || isempty(cursor.(key))
        return;
    end
    cursor = cursor.(key);
end
value = char(string(cursor));
end

function open_browser(url)
if ispc
    commands = {sprintf('start chrome "%s"', url), sprintf('start "" "%s"', url)};
elseif ismac
    commands = {sprintf('open -a "Google Chrome" "%s"', url), sprintf('open "%s"', url)};
else
    commands = {sprintf('google-chrome "%s" >/dev/null 2>&1 &', url), ...
        sprintf('google-chrome-stable "%s" >/dev/null 2>&1 &', url), ...
        sprintf('chromium-browser "%s" >/dev/null 2>&1 &', url), ...
        sprintf('chromium "%s" >/dev/null 2>&1 &', url), ...
        sprintf('xdg-open "%s" >/dev/null 2>&1 &', url)};
end

for idx = 1:numel(commands)
    [status, ~] = system(commands{idx});
    if status == 0
        return;
    end
end
warning('[ontology_viewer] Could not open Chrome automatically. Open this URL manually: %s', url);
end

function stop_existing_viewer_server(port)
if ispc
    return;
end

port = round(double(port));
commands = { ...
    sprintf('fuser -k %d/tcp >/dev/null 2>&1', port), ...
    sprintf('pids=$(lsof -ti tcp:%d 2>/dev/null); if [ -n "$pids" ]; then kill $pids >/dev/null 2>&1; fi', port)};
for idx = 1:numel(commands)
    system(commands{idx});
end
pause(0.2);
end
