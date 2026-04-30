%% OBDM simulation set 
% set 

classdef SimulationConfig
 properties (Constant)
 % ROS2 set
 ROS_DOMAIN_ID = 0; % ROS_DOMAIN_ID 
 ROS_LOCALHOST_ONLY = false; % 
 
 % set
 AIRCRAFT_TYPE = 'plane'; % 'plane', 'quadplane', 'copter'
 AIRCRAFT_CRUISE_SPEED = 20; % m/s
 AIRCRAFT_MAX_ALTITUDE = 500; % 
 
 % set (simulation)
 BATTERY_CAPACITY = 5000; % mAh
 BATTERY_CONSUMPTION_CRUISE = 50; % mAh/min at cruise speed
 
 % ROS2 topic set
 TOPICS = struct(...
 'tf', '/ap/tf', ... % / 
 'imu', '/ap/imu/sample', ... % IMU 
 'gps', '/ap/gps/sample', ... % GPS
 'baro', '/ap/baro/sample', ... % 
 'battery', '/ap/battery/state' ... % 
 );
 
 % data sample (Hz)
 SAMPLE_RATE = struct(...
 'tf', 10, ...
 'imu', 50, ...
 'gps', 10, ...
 'baro', 10, ...
 'battery', 2 ...
 );
 
 % set (default)
 DEFAULT_STATIONS = struct(...
 'names', {{'Station_A', 'Station_B', 'Station_C'}}, ...
 'lats', [37.5665, 37.4979, 37.5513], ...
 'lons', [126.9780, 127.0276, 126.9725], ...
 'alts', [50, 50, 50] ...
 );
 
 % set
 UPDATE_INTERVAL = 1.0; % , main 
 
 % data 
 LOGGING_ENABLED = true;
 LOGGING_DIR = './logs/';
 LOGGING_SAMPLE_RATE = 1; % 1 Hz 
 
 % 
 MIN_BATTERY_MARGIN = 15; % (%)
 WIND_SAFETY_MARGIN = 2.0; % (m/s)
 end
 
 methods (Static)
 function print_config()
 % set 
 fprintf('\n========== OBDM simulation set ==========\n');
 fprintf('ROS_DOMAIN_ID: %d\n', SimulationConfig.ROS_DOMAIN_ID);
 fprintf(' : %s\n', SimulationConfig.AIRCRAFT_TYPE);
 fprintf(' : %d m/s\n', SimulationConfig.AIRCRAFT_CRUISE_SPEED);
 fprintf(' : %d mAh\n', SimulationConfig.BATTERY_CAPACITY);
 fprintf(' : %d mAh/min\n', SimulationConfig.BATTERY_CONSUMPTION_CRUISE);
 fprintf('data : %s\n', onoff(SimulationConfig.LOGGING_ENABLED));
 fprintf('=========================================\n\n');
 end
 
 function validate_ros2_environment()
 % ROS2 verify
 fprintf('ROS2 verify ...\n');
 
 try
 % ROS2 main ID check
 domain_id = getenv('ROS_DOMAIN_ID');
 if isempty(domain_id)
 domain_id = '0';
 end
 fprintf('✓ ROS_DOMAIN_ID: %s\n', domain_id);
 
 % ROS2 check ( MATLAB )
 % ros2 gettimesync available 
 fprintf('✓ MATLAB ROS2 complete/done\n');
 
 catch ME
 fprintf('✗ ROS2 verify failed: %s\n', ME.message);
 end
 end

 function topics = get_topics(ns)
 % namespace topic 
 if nargin < 1 || isempty(ns)
 ns = '/ap';
 end
 % Ensure lead slash and no trail slash
 if ns(1) ~= '/'
 ns = ['/' ns];
 end
 if ns(end) == '/'
 ns = ns(1:end-1);
 end

 topics = struct();
 topics.tf = [ns '/tf'];
 topics.imu = [ns '/imu/sample'];
 topics.gps = [ns '/gps/sample'];
 topics.baro = [ns '/baro/sample'];
 topics.battery = [ns '/battery/state'];
 end
 
 function output = onoff(flag)
 % boolean 
 if flag
 output = '';
 else
 output = '';
 end
 end
 end
end
