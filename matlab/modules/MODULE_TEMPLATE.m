%% MODULE_TEMPLATE - New Module Template
% 새로운 모듈을 만들 때 이 템플릿을 복사해서 사용하세요.
% 
% 단계:
% 1. 이 파일을 matlab/modules/YourModuleName.m으로 복사
% 2. 다음을 수정:
%    - 클래스명 (classdef MyModule)
%    - 파일명 (MODULE_TEMPLATE -> YourModuleName)
%    - 주석과 구현
% 3. matlab/tests/test_your_module_name.m 생성
% 4. 독립 테스트로 검증
% 5. integrated_main.m에서 호출

classdef MODULE_TEMPLATE < handle
    % 모듈 설명을 여기 작성하세요
    %
    % 용도: 이 모듈이 하는 일
    %
    % 사용법:
    %   obj = MODULE_TEMPLATE('param', value);
    %   obj = obj.initialize();
    %   result = obj.execute_main_operation();
    %   obj.cleanup();
    
    properties (Access = private)
        % === 내부 상태 (Private) ===
        namespace
        verbose
        % ... 다른 internal properties 추가
    end
    
    properties (Access = public)
        % === 공개 상태 (Public) ===
        is_initialized = false
        % ... 다른 public properties 추가
    end
    
    methods
        %% ========== 공개 메서드 (Public Methods) ==========
        
        function obj = MODULE_TEMPLATE(varargin)
            % 생성자
            %
            % 입력:
            %   'namespace' - ROS2 네임스페이스 (기본값: '/alti_transition')
            %   'verbose' - 상세 출력 (기본값: true)
            %
            % 예:
            %   obj = MODULE_TEMPLATE('namespace', '/alti_transition', 'verbose', true);
            
            p = inputParser;
            addParameter(p, 'namespace', '/alti_transition', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            % 다른 파라미터 추가:
            % addParameter(p, 'param_name', default_value, @validation_func);
            parse(p, varargin{:});
            
            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
            
            if obj.verbose
                fprintf('[MODULE_TEMPLATE] Initialized (namespace: %s)\n', obj.namespace);
            end
        end
        
        function obj = initialize(obj)
            % 초기화 - ROS2 연결, 서비스/구독자 생성 등
            %
            % 출력:
            %   obj - self (메서드 체인 가능)
            
            if obj.verbose
                fprintf('[MODULE_TEMPLATE] Initializing...\n');
            end
            
            try
                % === 초기화 코드 작성 ===
                % 예: ROS2 서비스 클라이언트 생성
                % service = ros2serviceclient(...);
                
                obj.is_initialized = true;
                
                if obj.verbose
                    fprintf('[MODULE_TEMPLATE] Initialization complete\n');
                end
            catch ME
                fprintf('[MODULE_TEMPLATE] Initialization failed: %s\n', ME.message);
                rethrow(ME);
            end
        end
        
        function result = execute_main_operation(obj)
            % 주요 동작 - 모듈의 핵심 기능 실행
            %
            % 출력:
            %   result - 동작 결과 (자료형은 필요에 따라)
            
            if ~obj.is_initialized
                error('MODULE_TEMPLATE: initialize() 호출 필수');
            end
            
            if obj.verbose
                fprintf('[MODULE_TEMPLATE] Executing main operation...\n');
            end
            
            try
                % === 주요 작업 코드 작성 ===
                % 예: 데이터 처리, 제어 명령 전송 등
                
                result = true;  % 성공/실패 표시
                
                if obj.verbose
                    fprintf('[MODULE_TEMPLATE] Main operation complete\n');
                end
            catch ME
                fprintf('[MODULE_TEMPLATE] Operation failed: %s\n', ME.message);
                result = false;
                rethrow(ME);
            end
        end
        
        function cleanup(obj)
            % 정리 - 리소스 해제, 구독 취소 등
            
            if obj.verbose
                fprintf('[MODULE_TEMPLATE] Cleaning up...\n');
            end
            
            try
                % === 정리 코드 작성 ===
                % 예: 구독자 해제, 노드 제거 등
                
                if obj.verbose
                    fprintf('[MODULE_TEMPLATE] Cleanup complete\n');
                end
            catch ME
                if obj.verbose
                    fprintf('[MODULE_TEMPLATE] Cleanup warning: %s\n', ME.message);
                end
            end
        end
        
    end
    
    methods (Access = private)
        %% ========== 비공개 메서드 (Private Helper Methods) ==========
        
        function helper_result = private_helper_func(obj, input_arg)
            % 내부 도우미 함수
            % 이 메서드는 외부에서 호출할 수 없음
            
            helper_result = input_arg;  % 처리
        end
        
    end
    
end
