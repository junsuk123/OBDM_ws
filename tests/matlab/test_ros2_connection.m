%% Quick ROS2 Topic Connection Test
clear; clc;

setenv('ROS_DOMAIN_ID', '0');
fprintf('Creating ROS2 node...\n');
node = ros2node("/matlab_test");

% Test IMU
fprintf('Subscribing to /alti_transition/imu...\n');
try
    sub_imu = ros2subscriber(node, "/alti_transition/imu", "sensor_msgs/Imu");
    fprintf('  ✓ Successfully created IMU subscriber\n');
    
    % Try to receive one message with a timeout
    [msg, status] = receive(sub_imu, 2.0);
    if status && ~isempty(msg)
        fprintf('  ✓ Received IMU message:\n');
        fprintf('    accel: [%.3f, %.3f, %.3f]\n', ...
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
        fprintf('    gyro: [%.3f, %.3f, %.3f]\n', ...
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    else
        fprintf('  ✗ No IMU message received\n');
    end
catch ME
    fprintf('  ✗ IMU subscription failed: %s\n', ME.message);
end

% Test Odometry
fprintf('\nSubscribing to /alti_transition/odometry...\n');
try
    sub_odom = ros2subscriber(node, "/alti_transition/odometry", "nav_msgs/Odometry");
    fprintf('  ✓ Successfully created odometry subscriber\n');
    
    [msg, status] = receive(sub_odom, 2.0);
    if status && ~isempty(msg)
        fprintf('  ✓ Received odometry message:\n');
        fprintf('    position: [%.3f, %.3f, %.3f]\n', ...
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    else
        fprintf('  ✗ No odometry message received\n');
    end
catch ME
    fprintf('  ✗ Odometry subscription failed: %s\n', ME.message);
end

fprintf('\nDone.\n');
