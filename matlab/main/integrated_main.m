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
fprintf('║ 🎯 Integrated Flight Control       ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

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
    
    try
        % Subscribe to topics
        sub_imu = ros2subscriber(topics.imu);
        sub_gps = ros2subscriber(topics.gps);
        sub_tf = ros2subscriber(topics.tf);
        
        fprintf('Topic subscriptions successful:\\n');
        fprintf('  ✓ %s\\n', topics.imu);
        fprintf('  ✓ %s\\n', topics.gps);
        fprintf('  ✓ %s\\n\\n', topics.tf);
        
        % Collect data for 10 seconds
        data_log = struct('imu', [], 'gps', [], 'tf', []);
        
        for t = 1:10
            % Try to receive IMU data
            try
                msg = receive(sub_imu, 1);
                if ~isempty(msg)
                    data_log.imu = [data_log.imu; msg.LinearAcceleration.X, msg.LinearAcceleration.Y, msg.LinearAcceleration.Z];
                end
            catch
            end
            
            % Try to receive GPS data
            try
                msg = receive(sub_gps, 1);
                if ~isempty(msg)
                    data_log.gps = [data_log.gps; msg.Latitude, msg.Longitude, msg.Altitude];
                end
            catch
            end
            
            % Try to receive TF data
            try
                msg = receive(sub_tf, 1);
                if ~isempty(msg) && ~isempty(msg.Transforms)
                    tf = msg.Transforms(1);
                    data_log.tf = [data_log.tf; tf.Transform.Translation.X, tf.Transform.Translation.Y, tf.Transform.Translation.Z];
                end
            catch
            end
            
            fprintf('[%2d/10] IMU: %d messages, GPS: %d messages, TF: %d messages\\n', ...
                t, size(data_log.imu, 1), size(data_log.gps, 1), size(data_log.tf, 1));
            pause(1);
        end
        
        % Display data statistics
        fprintf('\\nData Collection Summary:\\n');
        fprintf('  IMU messages: %d\\n', size(data_log.imu, 1));
        fprintf('  GPS messages: %d\\n', size(data_log.gps, 1));
        fprintf('  TF messages: %d\\n', size(data_log.tf, 1));
        
        % Cleanup subscribers
        clear sub_imu sub_gps sub_tf;
        
    catch ME
        fprintf('Data collection error: %s\\n', ME.message);
    end
    
    % ====== 5. Final Status ======
    fprintf('\\n═════════════════════════════════════\n');
    fprintf('5️⃣ Final Simulation Status\n');
    fprintf('═════════════════════════════════════\n\n');
    
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

fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║ 🎯 simulation control () ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

% cleanup set ( terminate )
cleanup_obj = onCleanup(@cleanup_on_exit);

try
 % ====== 1. Initialize ======
 fprintf('═════════════════════════════════════\n');
 fprintf('1️⃣ Initialize \n');
 fprintf('═════════════════════════════════════\n\n');
 
 % add (main matlab add)
 matlab_root = fileparts(fileparts(mfilename('fullpath'))); % main 
 addpath(genpath(matlab_root));
 
 % Manager 
 sim = SimulationManager('verbose', true);
 pm = ProcessManager('verbose', true);
 
 % process cleanup
 fprintf(' process cleanup...\n');
 pm = pm.cleanup_all_simulation();
 pause(1);
 
 % simulation Initialize
 fprintf('\nsimulation Manager Initialize...\n');
 sim = sim.Initialize();
 
 % ====== 2. simulation Start ======
 fprintf('\n═════════════════════════════════════\n');
 fprintf('2️⃣ simulation Start\n');
 fprintf('═════════════════════════════════════\n\n');
 
 sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
 
 % ====== 3. ROS2 Connection check ======
 fprintf('\n═════════════════════════════════════\n');
 fprintf('3️⃣ ROS2 Connection check\n');
 fprintf('═════════════════════════════════════\n\n');
 
 sim.ros2_manager.print_info();
 
 % ====== 4. Data Receive ======
 fprintf('═════════════════════════════════════\n');
 fprintf('4️⃣ Data Receive (10)\n');
 fprintf('═════════════════════════════════════\n\n');
 
 topics = sim.get_topics();
 
 try
 % subscribe set
 sub_imu = ros2subscriber(topics.imu);
 sub_gps = ros2subscriber(topics.gps);
 sub_tf = ros2subscriber(topics.tf);
 
 fprintf('Topic subscribe success:\n');
 fprintf(' ✓ %s\n', topics.imu);
 fprintf(' ✓ %s\n', topics.gps);
 fprintf(' ✓ %s\n\n', topics.tf);
 
 % 10 Data Receive
 data_log = struct('imu', [], 'gps', [], 'tf', []);
 
 for t = 1:10
 % IMU Data
 try
 msg = receive(sub_imu, 1);
 if ~isempty(msg)
 data_log.imu = [data_log.imu; msg.LinearAcceleration.X, msg.LinearAcceleration.Y, msg.LinearAcceleration.Z];
 end
 catch
 end
 
 % GPS Data
 try
 msg = receive(sub_gps, 1);
 if ~isempty(msg)
 data_log.gps = [data_log.gps; msg.Latitude, msg.Longitude, msg.Altitude];
 end
 catch
 end
 
 % TF Data
 try
 msg = receive(sub_tf, 1);
 if ~isempty(msg) && ~isempty(msg.Transforms)
 tf = msg.Transforms(1);
 data_log.tf = [data_log.tf; tf.Transform.Translation.X, tf.Transform.Translation.Y, tf.Transform.Translation.Z];
 end
 catch
 end
 
 fprintf('[%2d/10] IMU: %d, GPS: %d, TF: %d\n', ...
 t, size(data_log.imu, 1), size(data_log.gps, 1), size(data_log.tf, 1));
 pause(1);
 end
 
 % 
 fprintf('\nData complete:\n');
 fprintf(' IMU : %d\n', size(data_log.imu, 1));
 fprintf(' GPS : %d\n', size(data_log.gps, 1));
 fprintf(' TF : %d\n', size(data_log.tf, 1));
 
 % cleanup
 clear sub_imu sub_gps sub_tf;
 
 catch ME
 fprintf('Data Receive error: %s\n', ME.message);
 end
 
 % ====== 5. ======
 fprintf('\n═════════════════════════════════════\n');
 fprintf('5️⃣ \n');
 fprintf('═════════════════════════════════════\n\n');
 
 sim.print_status();
 
 % ====== 6. terminate ======
 fprintf('═════════════════════════════════════\n');
 fprintf('✓ complete\n');
 fprintf('═════════════════════════════════════\n\n');
 
catch ME
 fprintf('\n✗ error !\n');
 fprintf(': %s\n', ME.message);
 fprintf('\n :\n');
 disp(ME.stack);
end

% ====== cleanup ======
function cleanup_on_exit()
 fprintf('\n╔════════════════════════════════════╗\n');
 fprintf('║ terminate - cleanup... ║\n');
 fprintf('╚════════════════════════════════════╝\n\n');
 
 try
 % ProcessManager terminate
 pm = ProcessManager('verbose', true);
 pm = pm.cleanup_all_simulation();
 pm.print_cleanup_log();
 catch ME
 fprintf('cleanup error: %s\n', ME.message);
 end
 
 fprintf('✓ terminate\n\n');
end
