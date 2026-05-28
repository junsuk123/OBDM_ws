%% Test DataCollectionModule GPS Publisher
clear; clc;
repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(genpath(fullfile(repoRoot, 'matlab')));
addpath(genpath(fullfile(repoRoot, 'ardu_ws', 'src', 'ardupilot', 'Tools', 'ros2', 'matlab_msg_gen')));

fprintf('╔════════════════════════════════╗\n');
fprintf('║  DataCollectionModule Test     ║\n');
fprintf('╚════════════════════════════════╝\n\n');

try
    % 모듈 생성
    collector = DataCollectionModule('namespace', '/alti_transition', 'verbose', true);
    
    % 구독 및 퍼블리셔 시작
    fprintf('\nInitializing subscriptions and publishers...\n');
    collector = collector.subscribe_to_topics();
    
    % 5초 동안 데이터 수집 (GPS 퍼블리셔도 함께 실행)
    fprintf('\nCollecting data for 5 seconds...\n');
    data = collector.collect_data(5);
    
    % 요약 출력
    collector.print_summary(data);
    
    % 정리
    collector.unsubscribe_from_topics();
    
catch ME
    fprintf('\nError: %s\n', ME.message);
    disp(ME);
end

fprintf('Test complete.\n');
