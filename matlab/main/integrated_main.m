%% Integrated Main - Complete MATLAB Flight Control
% Demonstrates full workflow: SimulationManager + ProcessManager + ROS2Manager
% Auto-launches simulation, collects data, displays statistics
%
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   run('main/integrated_main.m')
%
% Note: Press Ctrl+C to stop (cleanup runs automatically via onCleanup)

clear; clc;
fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║    Integrated Flight Control       ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

% === Setup environment variables (MATLAB standard per MathWorks) ===
fprintf('Setting up environment variables...\\n');

% Set ROS Domain ID
setenv('ROS_DOMAIN_ID', '0');
fprintf('  ROS_DOMAIN_ID: %s\\n', getenv('ROS_DOMAIN_ID'));

% Set Gazebo version
setenv('GZ_VERSION', 'harmonic');
fprintf('  GZ_VERSION: %s\\n', getenv('GZ_VERSION'));

% Set DISPLAY if not already set
if isempty(getenv('DISPLAY'))
    setenv('DISPLAY', ':0');
end
fprintf('  DISPLAY: %s\\n', getenv('DISPLAY'));

% Propagate X11 auth for GUI apps launched from MATLAB
xauth_value = getenv('XAUTHORITY');
if ~isempty(xauth_value)
    setenv('XAUTHORITY', xauth_value);
    fprintf('  XAUTHORITY: %s\n', getenv('XAUTHORITY'));
else
    fprintf('  XAUTHORITY: (not set)\n');
end

fprintf('\\n');

% Setup automatic cleanup on exit
cleanup_obj = onCleanup(@cleanup_on_exit);

try
    % ====== 1. Initialize ======
    fprintf('═════════════════════════════════════\n');
    fprintf('1️⃣ Initializing Managers\n');
    fprintf('═════════════════════════════════════\n\n');
    
    % Add all paths
    matlab_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(genpath(matlab_root));
    
    % Create manager instances
    sim = SimulationManager('verbose', true);
    pm = ProcessManager('verbose', true);
    
    % Verify workspace configuration
    if ~sim.verify_workspace()
        fprintf('\\n⚠️  Warning: Workspace configuration incomplete\\n');
        fprintf('Attempting to continue anyway...\\n\\n');
    end
    
    % Cleanup any existing processes
    fprintf('Cleaning up existing processes...\\n');
    pm = pm.cleanup_all_simulation();
    pause(1);
    
    % Initialize SimulationManager
    fprintf('\\nInitializing SimulationManager...\\n');
    sim = sim.initialize();
    
    % ====== 2. Start Simulation ======
    fprintf('\\n═════════════════════════════════════\n');
    fprintf('2️⃣ Starting Simulation\n');
    fprintf('═════════════════════════════════════\n\n');
    
    sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
    
    % ====== 3. ROS2 Connection Check ======
    fprintf('\\n═════════════════════════════════════\n');
    fprintf('3️⃣ ROS2 Connection Information\n');
    fprintf('═════════════════════════════════════\n\n');
    
    sim.ros2_manager.print_info();
    
    % ====== 4. Data Collection ======
    fprintf('═════════════════════════════════════\n');
    fprintf('4️⃣ Collecting Data (10 seconds)\n');
    fprintf('═════════════════════════════════════\n\n');
    
    topics = sim.get_topics();
    resolved_tf = SimulationConfig.resolve_topic('tf', sim.get_namespace());
    resolved_imu = SimulationConfig.resolve_topic('imu', sim.get_namespace());
    resolved_odom = SimulationConfig.resolve_topic('odometry', sim.get_namespace());
    resolved_gps = SimulationConfig.resolve_topic_if_available('gps', sim.get_namespace());

    try
        % Create ROS2 node for subscribers
        node = ros2node("/matlab_data_collector");

        % Use permissive QoS so MATLAB can match the bridge publishers.
        sub_imu = [];
        sub_odom = [];
        sub_tf = [];
        sub_gps = [];
        last_imu_stamp = NaN;
        last_odom_stamp = NaN;
        last_tf_stamp = NaN;
        last_gps_stamp = NaN;

        try
            sub_imu = ros2subscriber(node, resolved_imu, 'sensor_msgs/msg/Imu');
            fprintf('  ✓ %s\n', resolved_imu);
        catch
            fprintf('  ✗ %s (unavailable)\n', resolved_imu);
        end

        if ~isempty(resolved_gps)
            try
                sub_gps = ros2subscriber(node, resolved_gps, 'sensor_msgs/msg/NavSatFix');
                fprintf('  ✓ %s\n', resolved_gps);
            catch
                fprintf('  ✗ %s (unavailable)\n', resolved_gps);
            end
        else
            fprintf('  - GPS topic not published; skipping GPS subscription\n');
        end

        try
            sub_odom = ros2subscriber(node, resolved_odom, 'nav_msgs/msg/Odometry');
            fprintf('  ✓ %s\n', resolved_odom);
        catch
            fprintf('  ✗ %s (unavailable)\n', resolved_odom);
        end

        try
            sub_tf = ros2subscriber(node, resolved_tf, 'tf2_msgs/msg/TFMessage');
            fprintf('  ✓ %s\n', resolved_tf);
        catch
            fprintf('  ✗ %s (unavailable)\n', resolved_tf);
        end
        fprintf('\n');

        % Collect data for 10 seconds
        data_log = struct('imu', [], 'gps', [], 'odom', [], 'tf', []);

        for t = 1:10
            if ~isempty(sub_imu)
                try
                    [msg, status] = receive(sub_imu, 0.5);
                    if status && ~isempty(msg)
                        stamp = double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nanosec) * 1e-9;
                        if isnan(last_imu_stamp) || stamp ~= last_imu_stamp
                            data_log.imu = [data_log.imu; msg.LinearAcceleration.X, msg.LinearAcceleration.Y, msg.LinearAcceleration.Z];
                            last_imu_stamp = stamp;
                        end
                    end
                catch
                end
            end

            if ~isempty(sub_gps)
                try
                    [msg, status] = receive(sub_gps, 0.5);
                    if status && ~isempty(msg)
                        stamp = double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nanosec) * 1e-9;
                        if isnan(last_gps_stamp) || stamp ~= last_gps_stamp
                            data_log.gps = [data_log.gps; msg.Latitude, msg.Longitude, msg.Altitude];
                            last_gps_stamp = stamp;
                        end
                    end
                catch
                end
            end

            if ~isempty(sub_odom)
                try
                    [msg, status] = receive(sub_odom, 0.5);
                    if status && ~isempty(msg)
                        stamp = double(msg.Header.Stamp.Sec) + double(msg.Header.Stamp.Nanosec) * 1e-9;
                        if isnan(last_odom_stamp) || stamp ~= last_odom_stamp
                            data_log.odom = [data_log.odom; msg.Pose.Pose.Position.X, msg.Pose.Pose.Position.Y, msg.Pose.Pose.Position.Z];
                            last_odom_stamp = stamp;
                        end
                    end
                catch
                end
            end

            if ~isempty(sub_tf)
                try
                    [msg, status] = receive(sub_tf, 0.5);
                    if status && ~isempty(msg) && ~isempty(msg.Transforms)
                        tf = msg.Transforms(1);
                        stamp = double(tf.Header.Stamp.Sec) + double(tf.Header.Stamp.Nanosec) * 1e-9;
                        if isnan(last_tf_stamp) || stamp ~= last_tf_stamp
                            data_log.tf = [data_log.tf; tf.Transform.Translation.X, tf.Transform.Translation.Y, tf.Transform.Translation.Z];
                            last_tf_stamp = stamp;
                        end
                    end
                catch
                end
            end

            fprintf('[%2d/10] IMU: %d messages, GPS: %d messages, ODOM: %d messages, TF: %d messages\n', ...
                t, size(data_log.imu, 1), size(data_log.gps, 1), size(data_log.odom, 1), size(data_log.tf, 1));
            pause(1);
        end

        fprintf('\nData Collection Summary:\n');
        fprintf('  IMU messages: %d\n', size(data_log.imu, 1));
        fprintf('  GPS messages: %d\n', size(data_log.gps, 1));
        fprintf('  ODOM messages: %d\n', size(data_log.odom, 1));
        fprintf('  TF messages: %d\n', size(data_log.tf, 1));

        clear sub_imu sub_gps sub_odom sub_tf node;
    catch ME
        fprintf('Data collection error: %s\n', ME.message);
        try
            clear sub_imu sub_gps sub_odom sub_tf node;
        catch
        end
    end

    sim.print_status();
    
    % ====== 6. Completion ======
    fprintf('═════════════════════════════════════\n');
    fprintf('✓ Integrated test complete\\n');
    fprintf('═════════════════════════════════════\n\\n');
    
catch ME
    fprintf('\\n✗ Error during execution!\\n');
    fprintf('Message: %s\\n', ME.message);
    fprintf('\\nStack trace:\\n');
    disp(ME.stack);
end

% ====== Cleanup Function ======
function cleanup_on_exit()
    fprintf('\\n╔════════════════════════════════════╗\n');
    fprintf('║ Cleanup: Stopping Simulation       ║\n');
    fprintf('╚════════════════════════════════════╝\n\\n');
    
    try
        % Run full cleanup
        pm = ProcessManager('verbose', true);
        pm = pm.cleanup_all_simulation();
        pm.print_cleanup_log();
    catch ME
        fprintf('Cleanup error: %s\\n', ME.message);
    end
    
    fprintf('✓ Cleanup complete\\n\\n');
end
