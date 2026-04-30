%% Demo: Circle Flight Control
% Auto-starts simulation using SimulationManager
%
% Usage:
%   cd ~/OBDM_ws/matlab
%   addpath(genpath(pwd))
%   run('demo/circle_flight_demo.m')

clear; clc;
fprintf('\n╔════════════════════════════════════╗\n');
fprintf('║ ✈️ Circle Flight Control Demo    ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

try
    % Add path
    matlab_root = fileparts(pwd);
    addpath(genpath(matlab_root));
    
    % 1. Initialize SimulationManager
    fprintf('1️⃣ Initializing...\n');
    sim = SimulationManager('verbose', true);
    sim = sim.initialize();
    
    % 2. Start simulation
    fprintf('\n2️⃣ Starting simulation...\n');
    sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
    
    % 3. Flight parameters
    fprintf('\n3️⃣ Setting flight parameters...\n');
    RADIUS = 100;          % meters
    NUM_POINTS = 36;       % waypoints
    ALTITUDE = 50;         % meters
    
    fprintf('  Circle radius: %.1f m\n', RADIUS);
    fprintf('  Target altitude: %.1f m\n', ALTITUDE);
    fprintf('  Waypoints: %d\n', NUM_POINTS);
    
    % 4. ROS2 Service Discovery
    fprintf('\n4️⃣ ROS2 Service Discovery...\n');
    services = sim.get_services();
    
    fprintf('  Detected services:\n');
    fprintf('    Arm: %s\n', services.arm_motors);
    fprintf('    Mode: %s\n', services.mode_switch);
    fprintf('  Namespace: %s\n', sim.get_namespace());
    
    % Try to connect to services (optional, may not be available)
    arm_client = [];
    mode_client = [];
    services_available = false;
    
    try
        if exist('ros2serviceclient', 'builtin') || ~isempty(which('ros2serviceclient'))
            arm_client = ros2serviceclient(services.arm_motors, 'ardupilot_msgs/srv/ArmMotors');
            mode_client = ros2serviceclient(services.mode_switch, 'ardupilot_msgs/srv/ModeSwitch');
            services_available = true;
            fprintf('  ✓ Services connected\n');
        else
            fprintf('  ⚠️ Robotics System Toolbox not available\n');
        end
    catch ME
        fprintf('  ⚠️ Could not connect to services: %s\n', ME.message);
        fprintf('     Continuing with demo...\n');
    end
    
    % 5. Flight operations
    if services_available
        fprintf('\n5️⃣ Flight Operations...\n');
        fprintf('  Note: Demo simulation mode (no actual flight)\n\n');
        
        % Attempt arm
        fprintf('  → Attempting arm...\n');
        try
            arm_req = ros2message(arm_client);
            arm_req.Value = uint8(1);
            arm_resp = call(arm_client, arm_req);
            fprintf('  ✓ Arm command sent\n');
        catch ME
            fprintf('  ⚠️ Arm error: %s\n', ME.message);
        end
        
        pause(1);
        
        % Change mode
        fprintf('  → Changing mode to GUIDED...\n');
        try
            mode_req = ros2message(mode_client);
            mode_req.Mode = 'GUIDED';
            mode_resp = call(mode_client, mode_req);
            fprintf('  ✓ Mode change sent\n');
        catch ME
            fprintf('  ⚠️ Mode change error: %s\n', ME.message);
        end
    else
        fprintf('\n5️⃣ Skipping service operations (services not available)\n');
    end
    
    % 6. Generate waypoints
    fprintf('\n6️⃣ Generating waypoints...\n');
    
    center_lat = 37.5665;   % Seoul City Hall
    center_lon = 126.9780;
    
    % Coordinate conversion (approximate)
    lat_offset = 0.00001;   % ~1.1 m per degree
    lon_offset = 0.00001;   % ~0.9 m per degree (latitude dependent)
    
    waypoints = [];
    for i = 0:NUM_POINTS-1
        angle = 2*pi*i/NUM_POINTS;
        dx = RADIUS * cos(angle);
        dy = RADIUS * sin(angle);
        
        % Convert to lat/lon
        lat = center_lat + (dy / 111000);  % 1 degree ~111 km
        lon = center_lon + (dx / (111000 * cos(center_lat*pi/180)));
        
        waypoints = [waypoints; lat, lon, ALTITUDE];
    end
    
    fprintf('  Generated waypoints:\n');
    fprintf('    Center: (%.4f, %.4f)\n', center_lat, center_lon);
    fprintf('    Radius: %.1f m\n', RADIUS);
    fprintf('    Altitude: %.1f m\n', ALTITUDE);
    fprintf('    Total waypoints: %d\n', size(waypoints, 1));
    
    % 7. Simulation status
    fprintf('\n7️⃣ Simulation Status:\n');
    sim.print_status();
    
    % Complete
    fprintf('\n✓ Demo complete\n');
    fprintf('\nTip: Use SimulationManager and ProcessManager\n');
    fprintf('     to build complete flight control systems.\n\n');
    
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
fprintf('║ ✈️ Circle flight control demo ║\n');
fprintf('╚════════════════════════════════════╝\n\n');

try
 % add
 matlab_root = fileparts(pwd);
 addpath(genpath(matlab_root));
 
 % 1. simulation Manager Initialize
 fprintf('1️⃣ Initialize...\n');
 sim = SimulationManager('verbose', true);
 sim = sim.Initialize();
 
 % 2. simulation Start
 fprintf('\n2️⃣ simulation Start...\n');
 sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
 
 % 3. set
 fprintf('\n3️⃣ set...\n');
 RADIUS = 100; % (m)
 NUM_POINTS = 36; % 
 ALTITUDE = 50; % (m)
 
 fprintf(' : %.1f m\n', RADIUS);
 fprintf(' : %.1f m\n', ALTITUDE);
 fprintf(' : %d\n', NUM_POINTS);
 
 % 4. ROS2 set
 fprintf('\n4️⃣ ROS2 Connection...\n');
 services = sim.get_services();
 
 try
 arm_client = ros2serviceclient(services.arm_motors, 'ardupilot_msgs/srv/ArmMotors');
 mode_client = ros2serviceclient(services.mode_switch, 'ardupilot_msgs/srv/ModeSwitch');
 
 % Connection check
 if ~arm_client.isConnected || ~mode_client.isConnected
 pause(2); % 
 end
 
 fprintf(' ✓ Arm : %s\n', services.arm_motors);
 fprintf(' ✓ Mode : %s\n', services.mode_switch);
 
 catch ME
 fprintf(' ✗ Connection failed: %s\n', ME.message);
 fprintf(' : /ap namespace .\n');
 fprintf(' namespace: %s\n', sim.get_namespace());
 error(' Connection failed');
 end
 
 % 5. Start
 fprintf('\n5️⃣ process Start...\n');
 fprintf(' ⚠️ simulation .\n\n');
 
 % Arm 
 fprintf(' → ...\n');
 arm_req = ros2message(arm_client);
 arm_req.Value = uint8(1); % 
 try
 arm_resp = call(arm_client, arm_req);
 fprintf(' ✓ \n');
 catch ME
 fprintf(' ⚠️ error: %s\n', ME.message);
 end
 
 pause(1);
 
 % change/modify 
 fprintf(' → change/modify (GUIDED)...\n');
 mode_req = ros2message(mode_client);
 mode_req.Mode = 'GUIDED';
 try
 mode_resp = call(mode_client, mode_req);
 fprintf(' ✓ change/modify \n');
 catch ME
 fprintf(' ⚠️ change/modify error: %s\n', ME.message);
 end
 
 % 6. and publish 
 fprintf('\n6️⃣ ...\n');
 
 center_lat = 37.5665; % 
 center_lon = 126.9780;
 
 % ( )
 lat_offset = 0.00001; % ~1.1 m
 lon_offset = 0.00001; % ~0.9 m ( )
 
 waypoints = [];
 for i = 0:NUM_POINTS-1
 angle = 2*pi*i/NUM_POINTS;
 dx = RADIUS * cos(angle);
 dy = RADIUS * sin(angle);
 
 lat = center_lat + (dy / 111000); % 1 ~111km
 lon = center_lon + (dx / (111000 * cos(center_lat*pi/180)));
 
 waypoints = [waypoints; lat, lon, ALTITUDE];
 end
 
 fprintf(' :\n');
 fprintf(': (%.4f, %.4f)\n', center_lat, center_lon);
 fprintf(' : %.1f m\n', RADIUS);
 fprintf(' : %.1f m\n', ALTITUDE);
 
 % 7. 
 fprintf('\n7️⃣ simulation :\n');
 sim.print_status();
 
 % terminate
 fprintf('\n✓ demo complete\n');
 fprintf('\n💡 : Manager \n');
 fprintf(' control .\n\n');
 
catch ME
 fprintf('\n✗ error : %s\n', ME.message);
 disp(ME.stack);
 
 % cleanup
 try
 pm = ProcessManager('verbose', true);
 pm = pm.cleanup_all_simulation();
 catch
 end
end
