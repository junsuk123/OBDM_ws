addpath(genpath(pwd));
setenv('ROS_DOMAIN_ID','0');

fprintf('Starting test_sub_simple...\n');
node = ros2node('/test_sub');
fprintf('Node created\n');
pause(1);

try
    fprintf('Creating IMU subscriber...\n');
    sub_imu = ros2subscriber(node, '/alti_transition/imu', 'sensor_msgs/Imu');
    fprintf('IMU subscriber created\n');
    
    for i=1:5
        fprintf('Waiting for message %d...\n', i);
        [msg, status] = receive(sub_imu, 2.0);
        if status
            fprintf('  ✓ IMU message %d received\n', i);
        else
            fprintf('  ✗ No message received at iteration %d (timeout)\n', i);
        end
    end
catch ME
    fprintf('Error: %s\n', ME.message);
end
