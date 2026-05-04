%% ROS2 Connection verify 
% MATLAB simulation ROS2 topic receive check

%% cleanup 
function cleanup_matlab_session(receiver)
 % MATLAB cleanup: subscribe release, cleanup
 try
 if nargin > 0 && ~isempty(receiver) && isvalid(receiver)
 if receiver.is_subscribed
 receiver.unsubscribe_from_topics();
 fprintf('[Cleanup] ROS2 topic subscribe release complete/done\n');
 end
 end
 catch ME
 fprintf('[Cleanup] subscribe release error: %s\n', ME.message);
 end
 fprintf('[Cleanup] terminate\n');
end

function cleanup_orphaned_processes()
 % check
 try
 [status, out] = system('ps aux | grep -E "matlab" | grep -v grep | grep -v verify_ros2 || true');
 if status == 0 && ~isempty(strtrim(out))
 fprintf('[] exist MATLAB process detect ()\n');
 else
 fprintf('[] MATLAB Start\n');
 end
 catch
 % 
 end
end

clear; clc;

%% Initialize/init and cleanup 
fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║ MATLAB ROS2 Connection verify ( simulation) ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

cleanup_orphaned_processes();
receiver = []; % 

% set
TEST_DURATION = 30; % ( )
SAMPLE_INTERVAL = 2; % sample ( )
VERBOSITY = true; % 

%% Phase 1: set
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 1: set\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

try
 % add
 addpath(genpath(fileparts(pwd)));
 fprintf('✓ add\n');
 
 % set 
 SimulationConfig.print_config();
 
 % ROS2 verify
 SimulationConfig.validate_ros2_environment();
 
catch ME
 fprintf('✗ set failed: %s\n', ME.message);
 return;
end

%% Phase 2: topic check
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 2: available ROS2 topic check\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% namespace detect: namespace(/ap ) .
detected_ns = '/ap';
try
 [st, out] = system('ros2 topic list 2>/dev/null');
 if st == 0 && ~isempty(out)
 lines = strsplit(out, '\n');
 for i=1:length(lines)
 ln = strtrim(lines{i});
 if contains(ln, '/mavros/')
 detected_ns = '/mavros';
 break;
 elseif contains(ln, '/gps')
 parts = strsplit(ln, '/');
 if length(parts) >= 2 && ~isempty(parts{2})
 detected_ns = ['/' parts{2}];
 break;
 end
 end
 end
 end
catch
 detected_ns = '/ap';
end

topics = struct(...
 'tf', SimulationConfig.resolve_topic('tf', detected_ns), ...
 'imu', SimulationConfig.resolve_topic('imu', detected_ns), ...
 'gps', SimulationConfig.resolve_topic('gps', detected_ns), ...
 'baro', SimulationConfig.resolve_topic('baro', detected_ns), ...
 'battery', SimulationConfig.resolve_topic('battery', detected_ns) ...
);

expected_topics = {
 topics.tf, 'geometry_msgs/TransformStamped', 'TF (/)';
 topics.imu, 'sensor_msgs/Imu', 'IMU ()';
 topics.gps, 'sensor_msgs/NavSatFix', 'GPS';
 topics.baro, 'sensor_msgs/FluidPressure', 'Barometer';
 topics.battery, 'sensor_msgs/BatteryState', 'Battery'
};

fprintf(' topic:\n');
for i = 1:size(expected_topics, 1)
 fprintf(' [%d] %s\n', i, expected_topics{i, 1});
 fprintf(' : %s\n', expected_topics{i, 2});
 fprintf(' : %s\n\n', expected_topics{i, 3});
end

%% Phase 3: receive and subscribe
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 3: receive and subscribe Start\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

try
 receiver = AircraftStateReceiver('verbose', VERBOSITY, 'enable_logg', true);
 fprintf('✓ AircraftStateReceiver \n\n');
 
 fprintf('ROS2 topic subscribe Start...\n');
 receiver.subscribe_to_topics();
 
 if receiver.is_subscribed
 fprintf('✓ topic subscribe complete/done\n\n');
 else
 fprintf('✗ subscribe failed\n');
 fprintf(' → simulation check\n');
 return;
 end
 
catch ME
 fprintf('✗ receive error: %s\n', ME.message);
 return;
end

%% Phase 4: data receive verify
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 4: data receive verify\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('( %d data , %d sample)\n\n', TEST_DURATION, SAMPLE_INTERVAL);

% data 
collected_states = [];
timestamps = [];
received_count = 0;
valid_count = 0;

tic;
elapsed = 0;
sample_num = 1;

while elapsed < TEST_DURATION
 % 
 state = receiver.get_current_state();
 
 if state.is_valid
 valid_count = valid_count + 1;
 collected_states = [collected_states; state];
 timestamps = [timestamps; datetime('now')];
 
 % Info 
 fprintf('[%02d] : %s\n', sample_num, datestr(state.timestamp, 'HH:MM:SS'));
 fprintf(' : LAT=%.4f°, LON=%.4f°, ALT=%.1f m\n', ...
 state.position.latitude, state.position.longitude, state.position.altitude);
 fprintf(' : Vx=%.2f, Vy=%.2f, Vz=%.2f m/s\n', ...
 state.velocity.vx, state.velocity.vy, state.velocity.vz);
 fprintf(' : Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n', ...
 rad2deg(state.attitude.roll), rad2deg(state.attitude.pitch), rad2deg(state.attitude.yaw));
 fprintf(' : %.1f%% (%.2f V, %.2f A)\n', ...
 state.battery.percent, state.battery.voltage, state.battery.current);
 fprintf(' : ✓ \n\n');
 
 sample_num = sample_num + 1;
 else
 fprintf('[%02d] data ...\n', sample_num);
 end
 
 received_count = received_count + 1;
 
 % 
 elapsed = toc;
 
 % sample 
 if elapsed < TEST_DURATION
 pause(SAMPLE_INTERVAL);
 end
end

%% Phase 5: data quality 
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 5: data quality \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

fprintf('receive :\n');
fprintf(' sample : %d\n', received_count);
fprintf(' data: %d (%.1f%%)\n', valid_count, (valid_count/received_count)*100);

if valid_count > 0
 fprintf('\ndata :\n');
 
 % 
 lats = [collected_states.position.latitude]';
 lons = [collected_states.position.longitude]';
 alts = [collected_states.position.altitude]';
 
 fprintf(' : %.4f ~ %.4f (: %.4f)\n', min(lats), max(lats), mean(lats));
 fprintf(' : %.4f ~ %.4f (: %.4f)\n', min(lons), max(lons), mean(lons));
 fprintf(' : %.1f ~ %.1f m (: %.1f m)\n', min(alts), max(alts), mean(alts));
 
 % 
 speeds = [];
 for i = 1:length(collected_states)
 vx = collected_states(i).velocity.vx;
 vy = collected_states(i).velocity.vy;
 vz = collected_states(i).velocity.vz;
 speed = sqrt(vx^2 + vy^2 + vz^2);
 speeds = [speeds; speed];
 end
 
 fprintf('\n : %.2f ~ %.2f m/s (: %.2f m/s)\n', ...
 min(speeds), max(speeds), mean(speeds));
 
 % 
 batteries = [collected_states.battery.percent]';
 fprintf('\n : %.1f ~ %.1f%% (: %.1f%%)\n', ...
 min(batteries), max(batteries), mean(batteries));
 
 % 
 rolls = rad2deg([collected_states.attitude.roll]');
 pitches = rad2deg([collected_states.attitude.pitch]');
 yaws = rad2deg([collected_states.attitude.yaw]');
 
 fprintf('\n Roll: %.1f ~ %.1f° (: %.1f°)\n', ...
 min(rolls), max(rolls), mean(rolls));
 fprintf(' Pitch: %.1f ~ %.1f° (: %.1f°)\n', ...
 min(pitches), max(pitches), mean(pitches));
 fprintf(' Yaw: %.1f ~ %.1f° (: %.1f°)\n', ...
 min(yaws), max(yaws), mean(yaws));
end

%% Phase 6: topic receive 
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 6: topic receive \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% topic data check
topics_status = {
 '/ (TF)', ~isnan(collected_states(1).position.latitude);
 'IMU ', ~isnan(collected_states(1).imu.accel_x);
 'GPS', ~isnan(collected_states(1).gps.status);
 '', ~isnan(collected_states(1).barometer.pressure);
 '', ~isnan(collected_states(1).battery.percent)
};

fprintf('topic receive :\n');
for i = 1:size(topics_status, 1)
 topic_name = topics_status{i, 1};
 is_received = topics_status{i, 2};
 
 if is_received
 fprintf(' ✓ %s\n', topic_name);
 else
 fprintf(' ✗ %s (data )\n', topic_name);
 end
end

%% Phase 7: 
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Phase 7: \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

receiver.print_state();

%% 
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('verify \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

if valid_count > received_count * 0.8
 fprintf('🟢 success! simulation MATLAB normal .\n\n');
 fprintf(' :\n');
 fprintf(' 1. \n');
 fprintf(' 2. data API \n');
 fprintf(' 3. verify\n\n');
else
 fprintf('🟡 ! data receive . (%.0f%%)\n', (valid_count/received_count)*100);
 fprintf('\ncheck :\n');
 fprintf(' • simulation ?\n');
 fprintf(' • ROS_DOMAIN_ID ?\n');
 fprintf(' • set normal?\n\n');
end

fprintf('log : %s\n', receiver.data_logger.get_log_file());
fprintf('\nverify complete/done!\n\n');

%% cleanup and terminate
fprintf('═════════════════════════════════════════════════\n');
fprintf(' cleanup and terminate\n');
fprintf('═════════════════════════════════════════════════\n\n');

% cleanup 
cleanup_matlab_session(receiver);
clear receiver;

fprintf('\n terminate complete/done\n');
fprintf(' verify .\n\n');
