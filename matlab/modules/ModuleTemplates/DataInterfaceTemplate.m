classdef DataInterfaceTemplate < ModuleBase
    % DataInterfaceTemplate - ROS/Gazebo 통신 래퍼 예시

    properties
        node
        subs = struct();
        pubs = struct();
    end

    methods
        function obj = DataInterfaceTemplate()
            obj@ModuleBase('DataInterface');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            % 예: ros2node 생성, QoS 설정 등
            % obj.node = ros2node('/matlab_node');
        end

        function subscribe(obj, topic, callback)
            % 토픽 구독 래퍼
            % obj.subs.(sanitize(topic)) = obj.node.subscribe(topic, callback);
        end

        function publish(obj, topic, msg)
            % 발행 래퍼
        end

        function start(obj)
            % 필요한 경우 백그라운드 처리 시작
        end

        function stop(obj)
            % 구독 취소 등
        end
    end
end
