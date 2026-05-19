classdef DataCollectorModuleAdapter < ModuleBase
    % Adapter exposing standard lifecycle around DataCollectionModule

    properties
        collector % instance of DataCollectionModule
    end

    methods
        function obj = DataCollectorModuleAdapter()
            obj@ModuleBase('DataCollectorModuleAdapter');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            args = {};
            if isfield(config, 'namespace'), args{end+1} = 'namespace'; args{end+1} = config.namespace; end
            if isfield(config, 'verbose'), args{end+1} = 'verbose'; args{end+1} = config.verbose; end
            obj.collector = DataCollectionModule(args{:});
        end

        function start(obj)
            try
                obj.collector = obj.collector.subscribe_to_topics();
            catch
            end
        end

        function data = collect_data(obj, duration)
            data = obj.collector.collect_data(duration);
        end

        function stop(obj)
            try
                obj.collector = obj.collector.unsubscribe_from_topics();
            catch
            end
        end

        function cleanup(obj)
            try
                obj.collector.unsubscribe_from_topics();
            catch
            end
        end
    end
end
