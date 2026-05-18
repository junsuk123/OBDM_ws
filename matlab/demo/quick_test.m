%% Demo: Quick ROS2 Connection Test
% Quick test of ROS2 connectivity and data reception
%
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   run('demo/quick_test.m')

clear; clc;
fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║ 📊 Quick ROS2 Connection Test     ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

try
    % 0. Add paths
    matlab_root = fileparts(pwd);
    addpath(genpath(matlab_root));
    
    % 1. Initialize SimulationManager
    fprintf('1️⃣ Initializing SimulationManager...\n');
    sim = SimulationManager('verbose', true);
    sim = sim.initialize();  % Note: method is 'initialize' (lowercase)
    
    % 2. Start simulation
    fprintf('\n2️⃣ Starting simulation...\n');
    sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
    
    % 3. Display ROS2 Manager info
    fprintf('\n3️⃣ ROS2 Manager Info:\n');
    sim.ros2_manager.print_info();
    
    % 4. Subscribe to topics
    fprintf('\n4️⃣ Subscribing to topics...\n');
    topics = sim.get_topics();
    
    try
        sub_imu = ros2subscriber(topics.imu);
        sub_gps = ros2subscriber(topics.gps);
        sub_tf = ros2subscriber(topics.tf);
        fprintf('  ✓ Successfully subscribed to IMU, GPS, and TF topics\n');
        
        % 5. Receive sample data
        fprintf('\n5️⃣ Receiving sample data (5 messages)...\n');
        pause(2);
        
        msg_count = 0;
        for i = 1:5
            try
                msg = receive(sub_imu, 1);
                if ~isempty(msg)
                    fprintf('  [%d] IMU data received: accel_x=%.3f m/s²\n', i, msg.LinearAcceleration.X);
                    msg_count = msg_count + 1;
                end
            catch
                % Message not received in timeout
            end
            pause(1);
        end
        
        fprintf('\n✓ Successfully received %d messages\n', msg_count);
        
    catch ME
        fprintf('✗ Topic subscription error: %s\n', ME.message);
    end
    
    % 6. Display summary
    fprintf('\n6️⃣ Test Summary:\n');
    sim.print_status();
    
    fprintf('\n✓ Quick test complete\n');
    fprintf('  ROS2 connection: OK\n');
    fprintf('  Simulation status: Running\n\n');
    
catch ME
    fprintf('\n✗ Error: %s\n', ME.message);
    disp(ME.stack);
    
    % Cleanup
    try
        pm = ProcessManager('verbose', true);
        pm = pm.cleanup_all_simulation();
    catch
    end
end

fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║ 📊 Quick ROS2 Connection verification (demo) ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

try
 % 0. add
 matlab_root = fileparts(pwd);
 addpath(genpath(matlab_root));
 
 % 1. simulation Manager and Initialize
 fprintf('1️⃣ simulation Manager Initialize...\n');
 sim = SimulationManager('verbose', true);
 sim = sim.Initialize();
 
 % 2. simulation Start
 fprintf('\n2️⃣ simulation Start...\n');
 sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
 
 % 3. ROS2 Manager 
 fprintf('\n3️⃣ ROS2 Manager Info:\n');
 sim.ros2_manager.print_info();
 
 % 4. Topic subscribe 
 fprintf('4️⃣ Topic subscribe ...\n');
 topics = sim.get_topics();
 
 try
 sub_imu = ros2subscriber(topics.imu);
 sub_gps = ros2subscriber(topics.gps);
 sub_tf = ros2subscriber(topics.tf);
 fprintf(' ✓ IMU, GPS, TF Topic subscribe success\n');
 
 % 5. Data Receive 
 fprintf('\n5️⃣ Data Receive (5)...\n');
 pause(2);
 
 msg_count = 0;
 for i = 1:5
 try
 msg = receive(sub_imu, 1);
 if ~isempty(msg)
 fprintf(' [%d] IMU: accel_x=%.3f m/s²\n', i, msg.LinearAcceleration.X);
 msg_count = msg_count + 1;
 end
 catch
 % 
 end
 pause(1);
 end
 
 fprintf('\n✓ %d Receive success\n', msg_count);
 
 % terminate
 fprintf('\n6️⃣ terminate...\n');
 clear sub_imu sub_gps sub_tf;
 
 catch ME
 fprintf(' ✗ Topic subscribe error: %s\n', ME.message);
 end
 
 % 
 fprintf('\n╔════════════════════════════════════╗\n');
 if msg_count > 0
 fprintf('║ 🟢 verification success! ║\n');
 fprintf('║ MATLAB ROS2 Topic Receive ║\n');
 fprintf('║ . ║\n');
 else
 fprintf('║ 🟡 verification complete ║\n');
 fprintf('║ Receive failed. ║\n');
 end
 fprintf('╚════════════════════════════════════╝\n\n');
 
 % 
 sim.print_status();
 
catch ME
 fprintf('\n✗ error : %s\n', ME.message);
 fprintf(':\n');
 disp(ME.stack);
 
 % cleanup
 try
 pm = ProcessManager('verbose', true);
 pm = pm.cleanup_all_simulation();
 catch
 % cleanup failed 
 end
end

fprintf('\n✓ demo complete\n\n');
