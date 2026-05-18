%% GPS Direct Subscriber Test
clear; clc;
addpath(genpath(fullfile(pwd, 'ardu_ws_local', 'src', 'ardupilot', 'Tools', 'ros2', 'matlab_msg_gen')));

fprintf('=== GPS ROS2 Subscriber Test ===\n\n');

try
    node = ros2node('/test_gps_sub');
    fprintf('ROS2 node created\n');
    
    % GPS subscriber 생성
    fprintf('Creating GPS subscriber...\n');
    sub = ros2subscriber(node, '/alti_transition/navsat', 'sensor_msgs/NavSatFix');
    fprintf('✓ GPS subscriber created successfully\n\n');
    
    % 5초 동안 메시지 대기
    fprintf('Waiting 5 seconds for GPS messages...\n');
    pause(5);
    
    % 수신된 메시지 확인
    if ~isempty(sub.LatestMessage)
        fprintf('✓ Received GPS message!\n');
        fprintf('  Latitude: %f\n', sub.LatestMessage.latitude);
        fprintf('  Longitude: %f\n', sub.LatestMessage.longitude);
        fprintf('  Altitude: %f\n', sub.LatestMessage.altitude);
    else
        fprintf('✗ No GPS message received (LatestMessage is empty)\n');
    end
    
catch ME
    fprintf('ERROR: %s\n', ME.message);
    fprintf('\nFull error:\n');
    disp(ME);
end

fprintf('\nTest complete.\n');
