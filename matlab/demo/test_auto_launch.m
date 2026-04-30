%% Test: Gazebo Auto-Launch
% Simple test to verify SimulationManager can auto-launch Gazebo

clear; clc;
fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║ 🚀 Gazebo Auto-Launch Test        ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

try
    % Add paths
    matlab_root = fileparts(pwd);
    addpath(genpath(matlab_root));
    
    % Create manager with verbose output
    fprintf('1️⃣ Creating SimulationManager...\n');
    sim = SimulationManager('verbose', true);
    
    % Initialize
    fprintf('\n2️⃣ Initializing...\n');
    sim = sim.initialize();
    
    % Auto-launch without waiting for topics
    fprintf('\n3️⃣ Starting auto-launch...\n');
    sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', false);
    
    % Check if process started
    fprintf('\n4️⃣ Checking launch status...\n');
    pause(5);
    
    % Verify Gazebo process
    [status, output] = system('ps aux | grep -E "(launch.*alti|gz sim)" | grep -v grep');
    if status == 0
        fprintf('\n✓ SUCCESS: Gazebo process detected!\n');
        fprintf('Process info:\n');
        disp(output);
    else
        fprintf('\n✗ WARNING: Gazebo process not detected\n');
        fprintf('Check launch log: /tmp/obdm_plane_launch.log\n');
        system('tail -30 /tmp/obdm_plane_launch.log');
    end
    
    % Check ROS2 topics
    fprintf('\n5️⃣ Checking ROS2 topics...\n');
    [status, topics] = system('ros2 topic list 2>/dev/null | grep -E "(imu|gps|tf)" | head -5');
    if status == 0
        fprintf('✓ ROS2 topics available:\n');
        disp(topics);
    else
        fprintf('⚠️  No topics yet (may still be initializing)\n');
    end
    
    fprintf('\n✓ Test complete\n\n');
    
catch ME
    fprintf('\n✗ Error: %s\n', ME.message);
    disp(ME.stack);
end
