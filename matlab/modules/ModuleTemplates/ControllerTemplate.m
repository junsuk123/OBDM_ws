classdef ControllerTemplate < ModuleBase
    % ControllerTemplate - 제어기 템플릿 예시

    properties
        state
    end

    methods
        function obj = ControllerTemplate()
            obj@ModuleBase('Controller');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            obj.state = struct();
        end

        function step(obj, input)
            % 제어 루틴: input은 센서/목표 데이터
            % 출력 명령 생성 예시
            cmd = struct('throttle',0,'roll',0,'pitch',0,'yaw',0);
            % 실제 발행은 DataInterface를 통해 수행
            % obj.data_interface.publish('/iris/cmd', cmd);
        end
    end
end
