%% Simple Data Collection Test - No Process Management
% Direct sensor data collection from running Gazebo simulation
clear; clc;

setenv('ROS_DOMAIN_ID', '0');

fprintf('╔════════════════════════════════════╗\n');
fprintf('║  Direct Sensor Data Collection    ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

% Create ROS2 node
fprintf('Creating ROS2 node...\n');
node = ros2node("/matlab_sensor_collector");

% Subscribe to sensors
fprintf('\nSubscribing to sensors:\n');
subs.imu = [];
subs.odom = [];
subs.tf = [];
subs.battery = [];

try
    subs.imu = ros2subscriber(node, "/alti_transition/imu", "sensor_msgs/Imu");
    fprintf('  ✓ IMU: /alti_transition/imu\n');
catch ME
    fprintf('  ✗ IMU failed: %s\n', ME.message);
end

try
    subs.odom = ros2subscriber(node, "/alti_transition/odometry", "nav_msgs/Odometry");
    fprintf('  ✓ ODOM: /alti_transition/odometry\n');
catch ME
    fprintf('  ✗ ODOM failed: %s\n', ME.message);
end

try
    subs.tf = ros2subscriber(node, "/alti_transition/tf", "tf2_msgs/TFMessage");
    fprintf('  ✓ TF: /alti_transition/tf\n');
catch ME
    fprintf('  ✗ TF failed: %s\n', ME.message);
end

try
    subs.battery = ros2subscriber(node, "/alti_transition/battery/state", "sensor_msgs/BatteryState");
    fprintf('  ✓ BATTERY: /alti_transition/battery/state\n');
catch ME
    fprintf('  ✗ BATTERY failed: %s\n', ME.message);
end

% Collect data for 10 seconds
fprintf('\n═════════════════════════════════════\n');
fprintf('Collecting data (10 seconds)\n');
fprintf('═════════════════════════════════════\n\n');

data = struct();
data.imu_count = 0;
data.odom_count = 0;
data.tf_count = 0;
data.battery_count = 0;
data.imu_samples = [];
data.odom_samples = [];

for t = 1:10
    fprintf('[ %d/10]', t);
    
    % Try to receive IMU
    if ~isempty(subs.imu)
        [msg, status] = receive(subs.imu, 0.5);
        if status && ~isempty(msg)
            data.imu_count = data.imu_count + 1;
            data.imu_samples = [data.imu_samples; ...
                msg.linear_acceleration.x, ...
                msg.linear_acceleration.y, ...
                msg.linear_acceleration.z];
            fprintf(' IMU:%d', data.imu_count);
        end
    end
    
    % Try to receive ODOM
    if ~isempty(subs.odom)
        [msg, status] = receive(subs.odom, 0.5);
        if status && ~isempty(msg)
            data.odom_count = data.odom_count + 1;
            data.odom_samples = [data.odom_samples; ...
                msg.pose.pose.position.x, ...
                msg.pose.pose.position.y, ...
                msg.pose.pose.position.z];
            fprintf(' ODOM:%d', data.odom_count);
        end
    end
    
    % Try to receive TF
    if ~isempty(subs.tf)
        [msg, status] = receive(subs.tf, 0.5);
        if status && ~isempty(msg)
            data.tf_count = data.tf_count + 1;
            fprintf(' TF:%d', data.tf_count);
        end
    end
    
    % Try to receive Battery
    if ~isempty(subs.battery)
        [msg, status] = receive(subs.battery, 0.5);
        if status && ~isempty(msg)
            data.battery_count = data.battery_count + 1;
            fprintf(' BATT:%d', data.battery_count);
        end
    end
    
    fprintf('\n');
end

% Summary
fprintf('\n═════════════════════════════════════\n');
fprintf('Data Collection Summary\n');
fprintf('═════════════════════════════════════\n\n');
fprintf('  IMU messages:      %d\n', data.imu_count);
fprintf('  ODOM messages:     %d\n', data.odom_count);
fprintf('  TF messages:       %d\n', data.tf_count);
fprintf('  BATTERY messages:  %d\n\n', data.battery_count);

if data.imu_count > 0
    fprintf('IMU Sample (last):\n');
    fprintf('  accel: [%.4f, %.4f, %.4f] m/s²\n', ...
        data.imu_samples(end, 1), data.imu_samples(end, 2), data.imu_samples(end, 3));
end

if data.odom_count > 0
    fprintf('\nODOM Sample (last):\n');
    fprintf('  pos: [%.4f, %.4f, %.4f] m\n', ...
        data.odom_samples(end, 1), data.odom_samples(end, 2), data.odom_samples(end, 3));
end

fprintf('\n✓ Test complete\n');
