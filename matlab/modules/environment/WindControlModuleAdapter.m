classdef WindControlModuleAdapter < ModuleBase
    % WindControlModuleAdapter - ModuleBase wrapper around WindControlModule

    properties
        wind_ctrl
    end

    methods
        function obj = WindControlModuleAdapter()
            obj@ModuleBase('WindControlModuleAdapter');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            args = {};
            if isfield(config, 'namespace'), args{end+1} = 'namespace'; args{end+1} = config.namespace; end
            if isfield(config, 'worldName'), args{end+1} = 'worldName'; args{end+1} = config.worldName; end
            obj.wind_ctrl = WindControlModule(args{:});
        end

        function publish_once(obj, preset)
            if nargin < 2, preset = 'safe'; end
            obj.wind_ctrl.publishOnce(obj.wind_ctrl.presetToMsg(preset));
        end

        function start(obj)
            % optional periodic start is available via wind_ctrl.startPeriodic
        end

        function stop(obj)
            try
                obj.wind_ctrl.stopPeriodic();
            catch
            end
        end

        function cleanup(obj)
            try
                obj.wind_ctrl.stopPeriodic();
            catch
            end
        end
    end
end
