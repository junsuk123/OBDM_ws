%% TEST_MODULE_TEMPLATE - Test Script Template
% 새로운 모듈의 독립 테스트 스크립트 템플릿입니다.
%
% 단계:
% 1. 이 파일을 matlab/tests/test_your_module_name.m으로 복사
% 2. MODULE_NAME을 실제 모듈명으로 변경
% 3. 초기화 파라미터 수정
% 4. 테스트 코드 추가
% 5. 실행: addpath(genpath(pwd)); test_your_module_name;

clear; clc;

fprintf('\n╔════════════════════════════════════════╗\n');
fprintf('║  Test: MODULE_NAME                     ║\n');
fprintf('║  Independent Module Test              ║\n');
fprintf('╚════════════════════════════════════════╝\n\n');

setenv('ROS_DOMAIN_ID', '0');

try
    % ========== 1단계: 모듈 생성 ==========
    fprintf('1️⃣ Creating MODULE_NAME...\n');
    module = MODULE_NAME(...
        'namespace', '/alti_transition', ...
        'verbose', true);
    fprintf('   ✓ Module created\n\n');
    
    % ========== 2단계: 초기화 ==========
    fprintf('2️⃣ Initializing module...\n');
    module = module.initialize();
    fprintf('   ✓ Initialized\n\n');
    
    % ========== 3단계: 주요 동작 ==========
    fprintf('3️⃣ Executing main operation...\n');
    result = module.execute_main_operation();
    fprintf('   ✓ Operation complete\n\n');
    
    % ========== 4단계: 결과 확인 ==========
    fprintf('4️⃣ Result\n');
    if result
        fprintf('   ✓ Operation SUCCESSFUL\n\n');
    else
        fprintf('   ⚠️  Operation completed with issues\n\n');
    end
    
    % ========== 5단계: 정리 ==========
    fprintf('5️⃣ Cleanup\n');
    module.cleanup();
    fprintf('   ✓ Module cleaned up\n\n');
    
    fprintf('═════════════════════════════════════════\n');
    fprintf('✓ Test complete\n');
    fprintf('═════════════════════════════════════════\n\n');
    
catch ME
    fprintf('\n✗ Error: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    disp(ME.stack);
    
    % 비상 정리
    try
        if exist('module', 'var')
            module.cleanup();
        end
    catch
    end
end
