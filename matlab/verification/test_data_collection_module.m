%% Test DataCollectionModule - Independent Module Test
% 이 스크립트는 DataCollectionModule을 독립적으로 테스트합니다.
% 시뮬레이터가 실행 중일 때만 작동합니다.
%
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   test_data_collection_module

clear; clc;

fprintf('\n╔════════════════════════════════════════╗\n');
fprintf('║  Test: DataCollectionModule           ║\n');
fprintf('║  Independent Module Test              ║\n');
fprintf('╚════════════════════════════════════════╝\n\n');

setenv('ROS_DOMAIN_ID', '0');

try
    % 1. 모듈 생성
    fprintf('1️⃣ Creating DataCollectionModule...\n');
    collector = DataCollectionModule('namespace', '/alti_transition', 'verbose', true);
    fprintf('   ✓ Module created\n\n');
    
    % 2. 토픽 구독
    fprintf('2️⃣ Subscribing to topics...\n');
    collector = collector.subscribe_to_topics();
    fprintf('   ✓ Subscribed\n\n');
    
    % 3. 데이터 수집
    fprintf('3️⃣ Collecting data (10 seconds)...\n');
    data = collector.collect_data(10);
    fprintf('   ✓ Data collection complete\n\n');
    
    % 4. 결과 출력
    fprintf('4️⃣ Summary\n');
    collector.print_summary(data);
    
    % 5. 정리
    fprintf('5️⃣ Cleanup\n');
    collector.unsubscribe_from_topics();
    fprintf('   ✓ Module cleaned up\n\n');
    
    fprintf('═════════════════════════════════════════\n');
    fprintf('✓ Test complete\n');
    fprintf('═════════════════════════════════════════\n\n');
    
catch ME
    fprintf('\n✗ Error: %s\n', ME.message);
    fprintf('\nStack trace:\n');
    disp(ME.stack);
end
