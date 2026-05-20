%% GPS Data Publisher - Test Simulator
% GPS 토픽에 테스트 데이터를 발행하는 노드
clear; clc;

fprintf('╔════════════════════════════════╗\n');
fprintf('║   GPS Test Data Publisher      ║\n');
fprintf('╚════════════════════════════════╝\n\n');

addpath(genpath(fullfile(pwd, 'ardu_ws', 'src', 'ardupilot', 'Tools', 'ros2', 'matlab_msg_gen')));

try
    % ROS2 노드 생성
    node = ros2node('/gps_test_publisher');
    fprintf('✓ ROS2 node created: /gps_test_publisher\n\n');
    
    % GPS 퍼블리셔 생성
    pub = ros2publisher(node, '/alti_transition/navsat', 'sensor_msgs/NavSatFix');
    fprintf('✓ GPS publisher created for: /alti_transition/navsat\n\n');
    
    % GPS 메시지 생성
    gps_msg = ros2message('sensor_msgs/NavSatFix');
    
    % 테스트 GPS 데이터 설정 (서울 강남역 근처)
    gps_msg.latitude = 37.4979;      % 위도
    gps_msg.longitude = 127.0276;    % 경도
    gps_msg.altitude = 45.0;         % 고도 (미터)
    gps_msg.position_covariance = diag([0.01, 0.01, 0.01]);
    gps_msg.position_covariance_type = 2; % COVARIANCE_TYPE_DIAGONAL_KNOWN
    gps_msg.status.status = 0; % STATUS_FIX (fix acquired)
    
    fprintf('Publishing GPS test data:\n');
    fprintf('  Latitude:  %.4f°\n', gps_msg.latitude);
    fprintf('  Longitude: %.4f°\n', gps_msg.longitude);
    fprintf('  Altitude:  %.1f m\n\n', gps_msg.altitude);
    fprintf('Publishing every 0.5 seconds for 60 seconds...\n\n');
    
    % 60초 동안 1주기마다 GPS 데이터 발행
    start_time = tic;
    msg_count = 0;
    
    while toc(start_time) < 60
        % 타임스탐프 업데이트
        gps_msg.header.stamp = rostime('now');
        gps_msg.header.frame_id = 'gps_link';
        
        % 메시지 발행
        send(pub, gps_msg);
        msg_count = msg_count + 1;
        
        % 진행 상황 출력
        if mod(msg_count, 2) == 0
            elapsed = toc(start_time);
            fprintf('  [%5.1fs] %4d messages published\n', elapsed, msg_count);
        end
        
        pause(0.5);
    end
    
    fprintf('\n✓ GPS publisher completed\n');
    fprintf('  Total messages published: %d\n', msg_count);
    
catch ME
    fprintf('✗ Error: %s\n', ME.message);
    disp(ME);
end

fprintf('\nTest complete.\n');
