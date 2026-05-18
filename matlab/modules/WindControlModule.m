classdef WindControlModule < handle
    % WindControlModule Publish simple wind commands for runtime testing.
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

            obj.Node = ros2node(nodeName);
            % Publish directly using ros_gz_interfaces/Float32Array so the
            % ros_gz_bridge can map it to Gazebo messages without an extra node.
            topic = sprintf('%s/wind_command_az', obj.Namespace);
            try
                obj.Pub = ros2publisher(obj.Node, topic, 'ros_gz_interfaces/Float32Array');
                fprintf('[WindControlModule] Publisher created on topic: %s\n', topic);
            catch ME
                warning('[WindControlModule] Failed to create publisher on %s: %s', topic, ME.message);
                % Fallback to global topic
                try
                    obj.Pub = ros2publisher(obj.Node, '/wind_command_az', 'ros_gz_interfaces/Float32Array');
                    fprintf('[WindControlModule] Fallback publisher created on /wind_command_az\n');
                catch
                    obj.Pub = [];
                end
            end

            obj.LastPreset = '';
        end

        function publishOnce(obj, msgArray)
            if nargin < 2
                msgArray = obj.presetToMsg('safe');
            end
            % Ensure numeric array (double/single)
            arr = double(msgArray);

            % Try ROS2 publisher first if available
            if ~isempty(obj.Pub)
                try
                    m = ros2message('ros_gz_interfaces/Float32Array');
                    m.data = arr;
                    send(obj.Pub,m);
                    return;
                catch ME
                    warning('[WindControlModule] ROS publish failed: %s', ME.message);
                end
            end

            % Fallback: publish directly to gz topic using the bridge-matched
            % gz.msgs.Float_V payload expected by /world/<world>/wind.
            try
                payload = sprintf('data: [%s]', strjoin(arrayfun(@(v) num2str(v, '%.9g'), arr, 'UniformOutput', false), ', '));
                topic = sprintf('/world/%s/wind', obj.WorldName);
                cmd = sprintf('GZ_VERSION=harmonic gz topic -t "%s" -m gz.msgs.Float_V -p "%s"', topic, payload);
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
            % Return Float32 array: [enable, wind_x, wind_y, wind_z, horiz_noise, vert_noise, dir_var, mag_var, period, rise_time]
            switch lower(preset)
                case {'calm','none'}
                    msg = single([1, 0.5, 0, 0, 0.01, 0.0, 0.01, 0.01, 30, 10]);
                case 'safe'
                    msg = single([1, 1.5, 0, 0, 0.02, 0.01, 0.05, 0.05, 30, 10]);
                case 'boundary'
                    msg = single([1, 4.0, 0, 0, 0.07, 0.04, 0.17, 0.15, 15, 5]);
                case 'dangerous'
                    msg = single([1, 6.5, 0, 0, 0.15, 0.08, 0.35, 0.25, 6, 3]);
                otherwise
                    msg = single([1, 0, 0, 0, 0.0, 0.0, 0, 0, 0, 0]);
            end
        end

        function delete(obj)
            try
                obj.stopPeriodic();
            catch
            end
            try delete(obj.Node); catch; end
        end
    end
end
