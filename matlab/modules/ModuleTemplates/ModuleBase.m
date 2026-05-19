classdef ModuleBase < handle
    % ModuleBase - 공통 모듈 베이스 클래스
    % 서브클래스는 init/start/stop/cleanup을 구현합니다.

    properties
        config
        name
        dependencies % cell array of required module names or topics
    end

    methods
        function obj = ModuleBase(name)
            obj.name = name;
            obj.dependencies = {};
        end

        function init(obj, config)
            obj.config = config;
        end

        function start(obj)
            % override in subclass
        end

        function stop(obj)
            % override in subclass
        end

        function cleanup(obj)
            % override in subclass
        end
    end
end
