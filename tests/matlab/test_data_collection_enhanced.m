%% Integrated Flight Control System - Enhanced Data Collection Test
% Purpose: Collect sensor data from Gazebo-ROS2 integration
% Sensors: IMU, Odometry
% Duration: 10 seconds (adjusted for Gazebo stability)
% Author: MATLAB Integration Task
% Date: 2026-05-08

clear; clc; close all;

fprintf('\n');
fprintf('╔════════════════════════════════════╗\n');
fprintf('║  Enhanced Sensor Data Collection  ║\n');
fprintf('║  (ODOM + IMU Bridging Test)       ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

% Create ROS2 node
fprintf('Creating ROS2 node...\n');
try
    node = ros2node("/matlab_data_collection");
    fprintf('✓ ROS2 node created\n\n');
catch ME
    fprintf('✗ Failed to create ROS2 node: %s\n', ME.message);
    return;
end

% Define sensor topics and types
sensors = struct(...
    'name', {'IMU', 'ODOM'}, ...
    'topic', {'/alti_transition/imu', '/alti_transition/odometry'}, ...
    'type', {'sensor_msgs/Imu', 'nav_msgs/Odometry'}, ...
    'count', {0, 0} ...
);

% Subscribe to sensors
fprintf('Subscribing to sensors:\n');
subscribers = {};
for i = 1:length(sensors)
    try
        sub = ros2subscriber(node, sensors(i).topic, sensors(i).type);
        subscribers{i} = sub;
        fprintf('  ✓ %s: %s\n', sensors(i).name, sensors(i).topic);
    catch ME
        fprintf('  ✗ %s: Failed - %s\n', sensors(i).name, ME.message);
        subscribers{i} = [];
    end
end

fprintf('\n%s\n', repmat('=', 1, 37));
fprintf('Collecting data (10 seconds)\n');
fprintf('%s\n', repmat('=', 1, 37));
fprintf('\n');

% Data collection parameters
collection_duration = 10;  % seconds
poll_interval = 0.1;       % seconds
message_counts = zeros(1, length(sensors));
start_time = tic;

% Main collection loop
iteration = 0;
while toc(start_time) < collection_duration
    iteration = iteration + 1;
    
    % Check each subscriber
    for i = 1:length(subscribers)
        if ~isempty(subscribers{i})
            try
                msgs = subscribers{i}.LatestMessage;
                if ~isempty(msgs)
                    message_counts(i) = message_counts(i) + 1;
                    % Display brief update on detection
                    if mod(message_counts(i), 5) == 0  % Update every 5 messages
                        elapsed = toc(start_time);
                        fprintf('[%.1fs] %s messages: %d\n', elapsed, sensors(i).name, message_counts(i));
                    end
                end
            catch
                % Silent catch - subscriber may not have latest message
            end
        end
    end
    
    % Small delay before next poll
    pause(poll_interval);
end

elapsed = toc(start_time);
fprintf('\n%s\n', repmat('=', 1, 37));
fprintf('Data Collection Complete (%.1fs elapsed)\n', elapsed);
fprintf('%s\n\n', repmat('=', 1, 37));

% Display results summary
fprintf('╔════════════════════════════════════╗\n');
fprintf('║    Data Collection Summary        ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

total_messages = 0;
for i = 1:length(sensors)
    count = message_counts(i);
    total_messages = total_messages + count;
    
    % Determine status icon
    if count > 0
        icon = '✓';
    else
        icon = '✗';
    end
    
    % Calculate rate if any messages received
    if count > 0
        rate = count / elapsed;
    else
        rate = 0;
    end
    
    fprintf('  %s %-12s: %3d messages (%.1f Hz)\n', icon, [sensors(i).name], count, rate);
end

fprintf('\n  Total messages: %d\n', total_messages);

% Display sample data from IMU if available
fprintf('\n%s\n', repmat('-', 1, 37));
fprintf('Latest IMU Sample (if available):\n');
fprintf('%s\n', repmat('-', 1, 37));

if ~isempty(subscribers{1})
    try
        imu_msg = subscribers{1}.LatestMessage;
        if ~isempty(imu_msg)
            fprintf('  Acceleration (m/s²):\n');
            fprintf('    X: %.4f\n', imu_msg.linear_acceleration.x);
            fprintf('    Y: %.4f\n', imu_msg.linear_acceleration.y);
            fprintf('    Z: %.4f\n', imu_msg.linear_acceleration.z);
            fprintf('  Angular Velocity (rad/s):\n');
            fprintf('    X: %.4f\n', imu_msg.angular_velocity.x);
            fprintf('    Y: %.4f\n', imu_msg.angular_velocity.y);
            fprintf('    Z: %.4f\n', imu_msg.angular_velocity.z);
        else
            fprintf('  (No IMU message received)\n');
        end
    catch ME
        fprintf('  Error reading IMU: %s\n', ME.message);
    end
else
    fprintf('  (IMU subscriber not available)\n');
end

% Display sample data from Odometry if available
fprintf('\n%s\n', repmat('-', 1, 37));
fprintf('Latest Odometry Sample (if available):\n');
fprintf('%s\n', repmat('-', 1, 37));

if ~isempty(subscribers{2})
    try
        odom_msg = subscribers{2}.LatestMessage;
        if ~isempty(odom_msg)
            fprintf('  Position (m):\n');
            fprintf('    X: %.4f\n', odom_msg.pose.pose.position.x);
            fprintf('    Y: %.4f\n', odom_msg.pose.pose.position.y);
            fprintf('    Z: %.4f\n', odom_msg.pose.pose.position.z);
            fprintf('  Velocity (m/s):\n');
            fprintf('    Linear X: %.4f\n', odom_msg.twist.twist.linear.x);
            fprintf('    Linear Y: %.4f\n', odom_msg.twist.twist.linear.y);
            fprintf('    Linear Z: %.4f\n', odom_msg.twist.twist.linear.z);
        else
            fprintf('  (No Odometry message received)\n');
        end
    catch ME
        fprintf('  Error reading Odometry: %s\n', ME.message);
    end
else
    fprintf('  (Odometry subscriber not available)\n');
end

fprintf('\n');

% Final status
if total_messages > 0
    fprintf('✓ Test complete - Data collection successful\n');
    fprintf('  %d messages collected across %d sensors\n', total_messages, sum(message_counts > 0));
else
    fprintf('✗ Test complete - No messages collected\n');
    fprintf('  Check Gazebo simulation status and ROS2 topics\n');
end

fprintf('\n');

% Cleanup
clear node subscribers;
