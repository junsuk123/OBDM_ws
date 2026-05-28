%% Test CircleControlModule - Independent Module Test  
% 이 스크립트는 CircleControlModule을 독립적으로 테스트합니다.
% 시뮬레이터가 실행 중일 때만 작동합니다.
%
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   test_circle_control_module

clear; clc;

fprintf('\n╔════════════════════════════════════════╗\n');
fprintf('║  Test: CircleControlModule            ║\n');
fprintf('║  Independent Module Test              ║\n');
fprintf('╚════════════════════════════════════════╝\n\n');

setenv('ROS_DOMAIN_ID', '0');

try
    % 1. 모듈 생성
    fprintf('1️⃣ Creating CircleControlModule...\n');
    controller = CircleControlModule('namespace', '/alti_transition', 'verbose', true);
    fprintf('   ✓ Module created\n\n');
    
    % 2. 초기화
    fprintf('2️⃣ Initializing module...\n');
    controller = controller.initialize();
    fprintf('   ✓ Initialized\n\n');
    
    % 3. 원형 비행 실행
    fprintf('3️⃣ Executing circle flight...\n');
    success = controller.execute_circle_flight(...
        'radius_m', 100, ...
        'altitude_agl_m', 30, ...
        'num_points', 36, ...
        'tolerance_m', 30, ...
        'waypoint_timeout_s', 20);
    fprintf('   ✓ Execution complete\n\n');
    
    % 4. 결과 출력
    fprintf('4️⃣ Result\n');
    if success
        fprintf('   ✓ Circle flight SUCCESSFUL\n\n');
    else
        fprintf('   ⚠️  Circle flight completed with some failures\n\n');
    end
    
    % 5. 정리
    fprintf('5️⃣ Cleanup\n');
    controller.cleanup();
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
        if exist('controller', 'var')
            controller.cleanup();
        end
    catch
    end
end
