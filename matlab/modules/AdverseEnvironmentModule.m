classdef AdverseEnvironmentModule < ModuleBase
    % AdverseEnvironmentModule applies configurable wind disturbance presets.

    properties
        wind_ctrl
        timer_obj
        preset = 'dangerous'
        world_name = 'runway'
        namespace = '/iris'
        rng_stream
    end

    methods
        function obj = AdverseEnvironmentModule()
            obj@ModuleBase('AdverseEnvironmentModule');
            obj.rng_stream = RandStream('mt19937ar', 'Seed', 'shuffle');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            if isfield(config, 'namespace'), obj.namespace = config.namespace; end
            if isfield(config, 'worldName'), obj.world_name = config.worldName; end
            if isfield(config, 'adversePreset'), obj.preset = config.adversePreset; end
            obj.wind_ctrl = WindControlModule('namespace', obj.namespace, 'worldName', obj.world_name);
        end

        function start(obj)
            obj.apply_once();
            if ~isempty(obj.timer_obj)
                stop(obj.timer_obj); delete(obj.timer_obj);
            end
            obj.timer_obj = timer('ExecutionMode','fixedRate','BusyMode','drop','Period',1.0, ...
                'TimerFcn',@(~,~)obj.apply_once());
            start(obj.timer_obj);
            fprintf('[AdverseEnvironmentModule] started preset=%s\n', obj.preset);
        end

        function apply_once(obj)
            base = obj.base_wind(obj.preset);
            gust = obj.gust_vector(obj.preset);
            obj.wind_ctrl.publishOnce(base + gust);
        end

        function stop(obj)
            if ~isempty(obj.timer_obj) && isvalid(obj.timer_obj)
                stop(obj.timer_obj); delete(obj.timer_obj);
            end
            obj.timer_obj = [];
        end

        function cleanup(obj)
            obj.stop();
        end

        function v = base_wind(~, preset)
            switch lower(preset)
                case {'safe','light'}
                    v = [1.5, 0.2, 0.0];
                case {'boundary','gust'}
                    v = [4.0, 0.8, 0.1];
                case {'dangerous','extreme'}
                    v = [6.5, 1.6, 0.25];
                otherwise
                    v = [0.5, 0.0, 0.0];
            end
        end

        function g = gust_vector(obj, preset)
            switch lower(preset)
                case {'safe','light'}
                    sigma = [0.15, 0.08, 0.02];
                    burst = 0.2;
                case {'boundary','gust'}
                    sigma = [0.55, 0.35, 0.08];
                    burst = 0.8;
                case {'dangerous','extreme'}
                    sigma = [1.2, 0.8, 0.18];
                    burst = 1.8;
                otherwise
                    sigma = [0.05, 0.03, 0.0];
                    burst = 0.0;
            end
            g = randn(obj.rng_stream, 1, 3) .* sigma;
            if rand(obj.rng_stream) < 0.15
                g(1) = g(1) + burst * sign(rand(obj.rng_stream) - 0.5);
                g(2) = g(2) + 0.5 * burst * sign(rand(obj.rng_stream) - 0.5);
            end
        end
    end
end
