classdef DataLoggerModule < ModuleBase
    % DataLoggerModule - ModuleBase wrapper around existing DataLogger

    properties
        logger % instance of DataLogger
        bg_interval = 0.5
    end

    methods
        function obj = DataLoggerModule()
            obj@ModuleBase('DataLoggerModule');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            % pass base_dir and namespace if provided
            args = {};
            if isfield(config, 'base_dir'), args{end+1} = 'base_dir'; args{end+1} = config.base_dir; end
            if isfield(config, 'namespace'), args{end+1} = 'namespace'; args{end+1} = config.namespace; end
            if isfield(config, 'verbose'), args{end+1} = 'verbose'; args{end+1} = config.verbose; end
            obj.logger = DataLogger(args{:});
            if isfield(config, 'bg_interval'), obj.bg_interval = config.bg_interval; end
        end

        function start(obj)
            try
                obj.logger.start_background(obj.bg_interval);
            catch
            end
        end

        function stop(obj)
            try
                obj.logger.stop_background();
            catch
            end
        end

        function cleanup(obj)
            try
                obj.logger.cleanup();
            catch
            end
        end
    end
end
