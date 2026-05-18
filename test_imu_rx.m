%% Quick test to verify message reception
clear; clc;

setenv('ROS_DOMAIN_ID', '0');
fprintf('Testing message reception...\n');
node = ros2node("/matlab_rx_test");

sub = ros2subscriber(node, "/alti_transition/imu", "sensor_msgs/Imu");
fprintf('Subscriber created. Waiting for message...\n');

count = 0;
for i = 1:5
    [msg, status] = receive(sub, 2.0);
    if status && ~isempty(msg)
        count = count + 1;
        fprintf('[%d] IMU received: accel=[%.4f, %.4f, %.4f]\n', i, ...
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    else
        fprintf('[%d] No message\n', i);
    end
end

fprintf('\nTotal received: %d/5\n', count);
