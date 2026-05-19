classdef RealtimeVisualizerModule < ModuleBase
    % RealtimeVisualizerModule - ModuleBase wrapper around RealtimeVisualizer

    properties
        vis % instance of RealtimeVisualizer
        interval = 0.5
    end

    methods
        function obj = RealtimeVisualizerModule()
            obj@ModuleBase('RealtimeVisualizerModule');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            args = {};
            if isfield(config, 'namespace'), args{end+1} = 'namespace'; args{end+1} = config.namespace; end
            if isfield(config, 'verbose'), args{end+1} = 'verbose'; args{end+1} = config.verbose; end
            obj.vis = RealtimeVisualizer(args{:});
            if isfield(config, 'interval'), obj.interval = config.interval; end
        end

        function start(obj)
            try
                obj.vis.start_background(obj.interval);
            catch
            end
        end

        function stop(obj)
            try
                obj.vis.stop();
            catch
            end
        end

        function cleanup(obj)
            try
                obj.vis.stop();
            catch
            end
        end
    end
end
