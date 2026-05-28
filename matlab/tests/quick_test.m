%% Quick ROS2 Connection 
% detect namespace subscribe

clc;
fprintf('\n╔════════════════════════════════╗\n');
fprintf('║ Quick ROS2 Connection ║\n');
fprintf('╚════════════════════════════════╝\n\n');

% set add
addpath(genpath(fileparts(pwd)));

% namespace detect
fprintf('ROS2 topic detect ...\n');
[status, out] = system('ros2 topic list 2>/dev/null');
if status == 0 && ~isempty(out)
 lines = strsplit(out, newline);
 detected_ns = '/ap'; % default
 for i = 1:length(lines)
 line = strtrim(lines{i});
 if contains(line, '/mavros/') && ~contains(line, '/gz')
 detected_ns = '/mavros';
 break;
 elseif contains(line, '/imu') && ~contains(line, '/gz') && length(line) > 5
 parts = strsplit(line, '/');
 if length(parts) >= 2
 detected_ns = ['/' parts{2}];
 break;
 end
 end
 end
else
 detected_ns = '/plane1';
end

fprintf('detect namespace: %s\n\n', detected_ns);

% topic 
topics = struct(...
 'tf', SimulationConfig.resolve_topic('tf', detected_ns), ...
 'imu', SimulationConfig.resolve_topic('imu', detected_ns), ...
 'odometry', SimulationConfig.resolve_topic('odometry', detected_ns), ...
 'clock', sprintf('%s/clock', detected_ns) ...
);

fprintf('subscribe topic:\n');
fprintf(' TF: %s\n', topics.tf);
fprintf(' IMU: %s\n', topics.imu);
fprintf(' Odometry: %s\n', topics.odometry);
fprintf(' Clock: %s\n\n', topics.clock);

% subscribe 
try
 fprintf('subscribe Start...\n');
 sub_tf = ros2subscriber(topics.tf);
 fprintf(' ✓ TF subscribe success\n');
 
 sub_imu = ros2subscriber(topics.imu);
 fprintf(' ✓ IMU subscribe success\n');
 
 sub_odom = ros2subscriber(topics.odometry);
 fprintf(' ✓ Odometry subscribe success\n');
 
 sub_clock = ros2subscriber(topics.clock);
 fprintf(' ✓ Clock subscribe success\n\n');
 
 % data and receive check
 fprintf('data receive (10)...\n\n');
 pause(2);
 
 % data check
 data_count = 0;
 for i = 1:10
 try
 msg_imu = receive(sub_imu, 1);
 if ~isempty(msg_imu)
 fprintf('[%d] IMU data receive: accel_x=%.3f\n', i, msg_imu.LinearAcceleration.X);
 data_count = data_count + 1;
 end
 catch
 % 
 end
 pause(1);
 end
 
 fprintf('\n✓ %d message receive\n\n', data_count);
 
 if data_count > 0
 fprintf('🟢 success! MATLAB ROS2 topic receive .\n');
 fprintf(': run(''verification/verify_ros2_connection.m'') verify \n');
 else
 fprintf('🟡 : message receive .\n');
 end
 
 % cleanup
 clear sub_tf sub_imu sub_odom sub_clock;
 
catch ME
 fprintf('✗ error: %s\n', ME.message);
 fprintf('\ncheck:\n');
 fprintf(' • ros2 topic list topic ?\n');
 fprintf(' • ROS2/Gazebo simulation ?\n');
end

fprintf('\n complete/done\n\n');
