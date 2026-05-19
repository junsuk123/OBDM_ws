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
            topic = sprintf('/world/%s/wind', obj.WorldName);
            fprintf('[WindControlModule] Using Gazebo CLI wind publishing on topic: %s\n', topic);

            obj.LastPreset = '';
        end

        function publishOnce(obj, windVector)
            if nargin < 2
                windVector = obj.presetToMsg('safe');
            end
            windVector = double(windVector);
            if numel(windVector) < 3
                warning('[WindControlModule] Wind vector must contain [x y z] in m/s');
                return;
            end

            try
                topic = sprintf('/world/%s/wind', obj.WorldName);
                payload = sprintf('linear_velocity: {x: %.9g, y: %.9g, z: %.9g} enable_wind: true', ...
                    windVector(1), windVector(2), windVector(3));
                cmd = sprintf('GZ_VERSION=harmonic gz topic -t "%s" -m gz.msgs.Wind -p "%s"', topic, payload);
                if ispc
                    % Windows not supported for gz CLI in this environment
                    warning('[WindControlModule] Direct gz publish not supported on Windows');
                else
                    status = system(cmd);
                    if status ~= 0
                        warning('[WindControlModule] gz topic publish failed (status %d)', status);
                    end
                end
            catch ME
                warning('[WindControlModule] Fallback gz publish failed: %s', ME.message);
            end
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
        end

        function msg = presetToMsg(~, preset)
            % Return steady wind vector [x y z] in world-frame m/s.
            switch lower(preset)
                case {'calm','none'}
                    msg = single([0.5, 0, 0]);
                case 'safe'
                    msg = single([1.5, 0, 0]);
                case 'boundary'
                    msg = single([4.0, 0, 0]);
                case 'dangerous'
                    msg = single([6.5, 0, 0]);
                otherwise
                    msg = single([0, 0, 0]);
            end
        end

        function delete(obj)
            try
                obj.stopPeriodic();
            catch
            end
        end
    end
end
