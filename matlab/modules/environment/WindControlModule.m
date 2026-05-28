classdef WindControlModule < handle
    % WindControlModule Publish simple Gazebo wind commands for runtime testing.
    % Usage:
    %  wc = WindControlModule('nodeName','/wind_ctrl');
    %  wc.setPreset('safe'); wc.publishOnce();
    %  wc.startPeriodic('moderate',1.0); % publish every 1s

    properties
        Node
        Pub
        LastPreset
        TimerObj
        Namespace = ''
        WorldName = 'runway'
        IsPublishing = false
        ActivePublishPids = []
        Verbose = false
        LogFile = '/tmp/obdm_wind_control_module.log'
    end

    methods
        function obj = WindControlModule(varargin)
            p = inputParser;
            addParameter(p,'nodeName','/wind_control',@ischar);
            addParameter(p,'namespace','/iris',@ischar);
            addParameter(p,'worldName','runway',@ischar);
            parse(p,varargin{:});

            nodeName = p.Results.nodeName;
            obj.Namespace = p.Results.namespace;
            obj.WorldName = p.Results.worldName;

            % normalize namespace to have leading slash and no trailing slash
            if isempty(obj.Namespace)
                obj.Namespace = '/iris';
            end
            if obj.Namespace(1) ~= '/'
                obj.Namespace = ['/' obj.Namespace];
            end
            if obj.Namespace(end) == '/'
                obj.Namespace = obj.Namespace(1:end-1);
            end

            obj.Node = [];
            obj.Pub = [];
            obj.Verbose = strcmpi(getenv('OBDM_WIND_VERBOSE'), '1');
            topic = sprintf('/world/%s/wind', obj.WorldName);
            if obj.Verbose
                fprintf('[WindControlModule] Using Gazebo CLI wind publishing on topic: %s\n', topic);
            end

            obj.LastPreset = '';
        end

        function publishOnce(obj, windVector)
            if nargin < 2
                windVector = obj.presetToMsg('safe');
            end
            if obj.IsPublishing
                return;
            end
            obj.IsPublishing = true;
            cleanup_publish = onCleanup(@() set_publish_flag(obj, false)); %#ok<NASGU>

            windVector = double(windVector);
            if numel(windVector) < 3
                warning('[WindControlModule] Wind vector must contain [x y z] in m/s');
                return;
            end

            try
                topic = sprintf('/world/%s/wind', obj.WorldName);
                payload = sprintf('linear_velocity: {x: %.9g, y: %.9g, z: %.9g} enable_wind: true', ...
                    windVector(1), windVector(2), windVector(3));
                % Publish in the background so MATLAB timers do not block on
                % the Gazebo CLI. The helper returns immediately after spawn.
                cmd = sprintf([ ...
                    'bash -lc ''setsid env GZ_VERSION=harmonic gz topic -t "%s" -m gz.msgs.Wind -p "%s" -n 1 ' ...
                    '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                    topic, payload, obj.LogFile);
                if ispc
                    % Windows not supported for gz CLI in this environment
                    warning('[WindControlModule] Direct gz publish not supported on Windows');
                else
                    [status, output] = system(cmd);
                    if status == 0
                        obj.LastPreset = payload;
                        if ~isempty(output)
                            pid_value = str2double(strtrim(output));
                            if ~isnan(pid_value) && pid_value > 0
                                obj.ActivePublishPids(end+1) = pid_value; %#ok<AGROW>
                            end
                            if obj.Verbose
                                fprintf('[WindControlModule] wind publish spawned (PID: %s, log: %s)\n', strtrim(output), obj.LogFile);
                            end
                        end
                    else
                        warning('[WindControlModule] gz wind publish spawn failed (status %d)', status);
                    end
                end
            catch ME
                if contains(lower(string(ME.message)), 'operation terminated by user')
                    return;
                end
                warning('[WindControlModule] Fallback gz publish failed: %s', ME.message);
            end
        end

        function set_publish_flag(obj, value)
            obj.IsPublishing = logical(value);
        end

        function startPeriodic(obj, preset, period_s)
            if nargin < 3, period_s = 1.0; end
            if ~isempty(obj.TimerObj)
                stop(obj.TimerObj); delete(obj.TimerObj); obj.TimerObj = []; end
            obj.TimerObj = timer('ExecutionMode','fixedRate','Period',period_s,'TimerFcn',@(~,~)obj.publishOnce(obj.presetToMsg(preset)));
            start(obj.TimerObj);
        end

        function stopPeriodic(obj)
            if ~isempty(obj.TimerObj)
                stop(obj.TimerObj); delete(obj.TimerObj); obj.TimerObj = []; end
            obj.stopSpawnedPublishers();
        end

        function stopSpawnedPublishers(obj)
            if isempty(obj.ActivePublishPids)
                return;
            end
            pids = unique(obj.ActivePublishPids(:)');
            obj.ActivePublishPids = [];
            for pid = pids
                try
                    system(sprintf('kill -TERM %d >/dev/null 2>&1 || true', pid));
                catch
                end
            end
        end

        function msg = presetToMsg(~, preset)
            % Return steady wind vector [x y z] in world-frame m/s.
            switch lower(preset)
                case {'calm','none'}
                    msg = single([0, 0, 0]);
                case {'safe','light'}
                    msg = single([1.5, 0, 0]);
                case {'boundary','gust'}
                    msg = single([2.0, 0, 0]);
                case {'dangerous','extreme'}
                    msg = single([3.5, 0, 0]);
                otherwise
                    msg = single([0, 0, 0]);
            end
        end

        function delete(obj)
            try
                obj.stopPeriodic();
            catch
            end
            try
                obj.stopSpawnedPublishers();
            catch
            end
        end
    end
end
