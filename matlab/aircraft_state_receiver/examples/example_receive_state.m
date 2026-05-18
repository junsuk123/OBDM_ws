%% receive 
% MATLAB ROS2 simulation receive 

clear; clc;

%% set
fprintf('========== OBDM receive ==========\n\n');

% simulation set 
SimulationConfig.print_config();

% ROS2 verify
SimulationConfig.validate_ros2_environment();

%% ROS2 set
fprintf('ROS2 set ...\n');
fprintf('- ROS_DOMAIN_ID: %d\n', SimulationConfig.ROS_DOMAIN_ID);
fprintf('- ROS_LOCALHOST_ONLY: %d\n', SimulationConfig.ROS_LOCALHOST_ONLY)\n');

% set ( )
% setenv('ROS_DOMAIN_ID', '0');

%% receive 
fprintf('\n receive Initialize/init...\n');
receiver = AircraftStateReceiver('verbose', true, 'enable_logg', true);

%% ROS2 subscribe Start
fprintf('ROS2 topic subscribe ...\n');
receiver.subscribe_to_topics();

if ~receiver.is_subscribed
 error('ROS2 subscribe failed. simulation check.');
end

%% data receive 
fprintf('\n receive ... (Ctrl+C terminate)\n');
fprintf('( 2 )\n\n');

% 
max_iterations = 60; % 2 (60 × 2 )
update_interval = 2; % 2 
iteration = 0;

try
 while iteration < max_iterations
 iteration = iteration + 1;
 
 % 
 state = receiver.get_current_state();
 
 if state.is_valid
 % Info ( )
 fprintf('[%03d] ', iteration);
 fprintf('Lat: %+.5f°, Lon: %+.5f°, Alt: %+7.2f m | ', ...
 state.position.latitude, state.position.longitude, state.position.altitude);
 fprintf('Speed: %+6.2f m/s | ', state.velocity.speed);
 fprintf('Battery: %+6.1f%% | ', state.battery.percent);
 fprintf('Yaw: %+7.2f°\n', rad2deg(state.attitude.yaw));
 else
 fprintf('[%03d] data ...\n', iteration);
 end
 
 % 
 pause(update_interval);
 end
 
catch exception
 fprintf('\n\n terminate: %s\n', exception.message);
end

%% 
fprintf('\n\n :\n');
receiver.print_state();

%% cleanup
fprintf(' cleanup ...\n');
receiver.unsubscribe_from_topics();
clear receiver;

fprintf(' complete/done!\n');
fprintf('===============================================\n\n');

%% 
fprintf(' :\n');
fprintf('1. max_iterations \n');
fprintf('2. data log logs/ \n');
fprintf('3. Start\n');
fprintf(' - decision_logic/RouteOptimizer.m \n');
fprintf(' - decision_logic/WeatherDataProcessor.m \n');
