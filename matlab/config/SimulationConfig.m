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
 'imu', '/ap/imu/experimental/data', ... % IMU 
 'odometry', '/ap/pose/filtered', ... % odometry 
	 'gps', '/ap/navsat/navsat0', ... % GPS
	 'navsat', '/ap/navsat/navsat0', ... % GPS sensor
 'airspeed', '/ap/airspeed', ... % airspeed
 'status', '/ap/status', ... % vehicle health/status
 'rc', '/ap/rc', ... % RC state
 'time', '/ap/time', ... % AP time
 'clock', '/ap/clock', ... % AP clock
 'battery', '/ap/battery', ... % 
 'attitude', '/ap/attitude', ... % optional telemetry
 'attitude_target', '/ap/attitude_target', ... % optional control feedback
 'nav_controller_output', '/ap/nav_controller_output', ... % optional control feedback
 'wind', '/ap/wind', ... % optional telemetry
 'pid_tuning', '/ap/pid_tuning' ... % optional control feedback
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

 function result = apply_ap_dds_sensor_config(overrides)
 % Apply AP_DDS sensor/control overrides to the workspace header.
 % Intended to be called before launching or rebuilding SITL.

 if nargin < 1 || isempty(overrides)
 result = struct('changed', false, 'config_path', '', 'backup_path', '', 'applied_fields', {{}}, 'messages', {{}});
 return;
 end

 if isstruct(overrides) && isfield(overrides, 'sensor') && isstruct(overrides.sensor)
 overrides = overrides.sensor;
 end

 config_path = SimulationConfig.get_ap_dds_config_path();
 if isempty(config_path) || ~exist(config_path, 'file')
 error('AP_DDS config file not found: %s', config_path);
 end

 field_map = SimulationConfig.get_ap_dds_override_field_map();
 fields = fieldnames(overrides);
 original_text = fileread(config_path);
 updated_text = original_text;
 applied_fields = {};
 messages = {};

 for i = 1:numel(fields)
	 field_name = fields{i};
	 if ~isfield(field_map, field_name)
		 messages{end+1} = sprintf('Ignored unknown AP DDS override field: %s', field_name); %#ok<AGROW>
		 continue;
	 end

	 macro_name = field_map.(field_name);
	 value = overrides.(field_name);
	 if islogical(value)
		 value = double(value);
	 elseif ischar(value) || isstring(value)
		 value = str2double(string(value));
	 end

	 if isnan(value)
		 error('Invalid AP DDS override value for %s', field_name);
	 end

	 pattern = sprintf('(?m)^(\\s*#define\\s+%s\\s+).*$' , macro_name);
	 replacement = sprintf('$1%d', value);
	 new_text = regexprep(updated_text, pattern, replacement);

	 if strcmp(new_text, updated_text)
		 messages{end+1} = sprintf('No matching macro line found for %s', macro_name); %#ok<AGROW>
	 else
		 updated_text = new_text;
		 applied_fields{end+1} = field_name; %#ok<AGROW>
	 end
 end

 changed = ~strcmp(original_text, updated_text);
 backup_path = '';
 if changed
	 backup_path = [config_path, '.obdm.bak'];
	 if ~exist(backup_path, 'file')
		 fid_backup = fopen(backup_path, 'w');
		 if fid_backup >= 0
			 fwrite(fid_backup, original_text);
			 fclose(fid_backup);
		 end
	 end

	 fid = fopen(config_path, 'w');
	 if fid < 0
		 error('Failed to open AP_DDS config for writing: %s', config_path);
	 end
	 cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
	 fwrite(fid, updated_text);
 end

 result = struct(...
	 'changed', changed, ...
	 'config_path', config_path, ...
	 'backup_path', backup_path, ...
	 'applied_fields', {applied_fields}, ...
	 'messages', {messages});
 end

 function config_path = get_ap_dds_config_path()
 % Return the workspace-local AP_DDS config header path.
 config_path = fullfile(SimulationConfig.get_ardu_ws_path(), 'src', 'ardupilot', 'libraries', 'AP_DDS', 'AP_DDS_config.h');
 end

 function repo_root = get_repo_root()
 % Return OBDM_ws root based on this file location.
 repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
 end

 function ardu_ws = get_ardu_ws_path()
 % Prefer the workspace-local ArduPilot ROS/Gazebo workspace.
 override = getenv('OBDM_ARDU_WS');
 if ~isempty(override)
	 ardu_ws = override;
	 return;
 end

 repo_candidate = fullfile(SimulationConfig.get_repo_root(), 'ardu_ws');
 if isfolder(repo_candidate)
	 ardu_ws = repo_candidate;
	 return;
 end

 ardu_ws = fullfile(getenv('HOME'), 'ardu_ws');
 end

 function field_map = get_ap_dds_override_field_map()
 % Map user-facing override names to AP_DDS config macros.
 field_map = struct( ...
	 'imu', 'AP_DDS_IMU_PUB_ENABLED', ...
	 'navsat', 'AP_DDS_NAVSATFIX_PUB_ENABLED', ...
	 'static_tf', 'AP_DDS_STATIC_TF_PUB_ENABLED', ...
	 'gps_global_origin', 'AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED', ...
	 'geopose', 'AP_DDS_GEOPOSE_PUB_ENABLED', ...
	 'pose', 'AP_DDS_LOCAL_POSE_PUB_ENABLED', ...
	 'twist', 'AP_DDS_LOCAL_VEL_PUB_ENABLED', ...
	 'airspeed', 'AP_DDS_AIRSPEED_PUB_ENABLED', ...
	 'rc', 'AP_DDS_RC_PUB_ENABLED', ...
	 'battery', 'AP_DDS_BATTERY_STATE_PUB_ENABLED', ...
	 'status', 'AP_DDS_STATUS_PUB_ENABLED', ...
	 'time', 'AP_DDS_TIME_PUB_ENABLED', ...
	 'clock', 'AP_DDS_CLOCK_PUB_ENABLED', ...
	 'joy', 'AP_DDS_JOY_SUB_ENABLED', ...
	 'cmd_vel', 'AP_DDS_VEL_CTRL_ENABLED', ...
	 'cmd_gps_pose', 'AP_DDS_GLOBAL_POS_CTRL_ENABLED', ...
	 'dynamic_tf', 'AP_DDS_DYNAMIC_TF_SUB_ENABLED', ...
	 'arm_motors', 'AP_DDS_ARM_SERVER_ENABLED', ...
	 'mode_switch', 'AP_DDS_MODE_SWITCH_SERVER_ENABLED', ...
	 'takeoff', 'AP_DDS_VTOL_TAKEOFF_SERVER_ENABLED', ...
	 'prearm_check', 'AP_DDS_ARM_CHECK_SERVER_ENABLED', ...
	 'parameter_server', 'AP_DDS_PARAMETER_SERVER_ENABLED', ...
	 'experimental', 'AP_DDS_EXPERIMENTAL_ENABLED');
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

 lower_ns = lower(ns);
 is_mavros = contains(lower_ns, 'mavros');

 topics = struct();

 if is_mavros
 topics.tf = '/tf';
 topics.imu = [ns '/imu/data'];
 topics.odometry = [ns '/local_position/odom'];
 topics.gps = [ns '/global_position/raw/fix'];
 topics.baro = [ns '/altitude'];
 topics.battery = [ns '/battery'];
 topics.navsat = [ns '/global_position/raw/fix'];
 topics.airspeed = [ns '/airspeed'];
 topics.status = [ns '/state'];
 topics.rc = [ns '/rc/in'];
 topics.time = [ns '/time'];
 topics.clock = [ns '/clock'];
 topics.attitude = [ns '/local_position/pose'];
 topics.attitude_target = [ns '/setpoint_raw/local'];
 topics.nav_controller_output = [ns '/nav_controller_output'];
 topics.wind = [ns '/wind'];
 topics.pid_tuning = [ns '/pid_tuning'];
 else
 topics.tf = [ns '/tf'];
 topics.imu = [ns '/imu/experimental/data'];
 topics.odometry = [ns '/pose/filtered'];
 topics.gps = [ns '/navsat'];
 topics.baro = [ns '/baro/sample'];
 topics.battery = [ns '/battery'];
 topics.navsat = [ns '/navsat'];
topics.airspeed = [ns '/air_speed'];
 topics.status = [ns '/status'];
 topics.rc = [ns '/rc'];
 topics.time = [ns '/time'];
 topics.clock = [ns '/clock'];
 topics.attitude = [ns '/attitude'];
 topics.attitude_target = [ns '/attitude_target'];
 topics.nav_controller_output = [ns '/nav_controller_output'];
 topics.wind = [ns '/wind'];
 topics.pid_tuning = [ns '/pid_tuning'];
 end
 end

 function candidates = get_topic_candidates(topic_key, ns)
 % Return ordered topic candidates for ArduPilot and MAVROS-style bridges.
 % Prioritizes official Gazebo bridge topics (/iris/*) per ArduPilot ROS2 docs
 if nargin < 2 || isempty(ns)
 ns = '/ap';
 end
 
 if ns(1) ~= '/'
 ns = ['/' ns];
 end
 if ns(end) == '/'
 ns = ns(1:end-1);
 end
 
 is_mavros = contains(lower(ns), 'mavros');
 is_iris = contains(lower(ns), 'iris');
 
 switch lower(topic_key)
 case 'tf'
 candidates = {[ns '/tf'], [ns '/gz/tf'], '/iris/tf', '/iris/gz/tf', '/ap/tf', '/tf', '/mavros/tf', '/alti_transition/tf'};
 case 'imu'
 candidates = {[ns '/imu'], [ns '/imu/experimental/data'], [ns '/imu/sample'], '/iris/imu', '/imu/experimental/data', '/imu/sample', '/imu', '/mavros/imu/data', '/mavros/imu/data_raw', '/alti_transition/imu'};
 case 'odometry'
 candidates = {[ns '/odometry'], [ns '/pose/filtered'], '/iris/odometry', '/pose/filtered', '/odometry', '/ap/pose/filtered', '/ap/twist/filtered', '/ap/geopose/filtered', '/mavros/local_position/odom', '/alti_transition/odometry'};
 case 'geopose'
 candidates = {[ns '/geopose/filtered'], [ns '/geopose'], '/iris/geopose/filtered', '/iris/geopose', '/ap/geopose/filtered', '/ap/geopose', '/mavros/geopose', '/alti_transition/geopose'};
 case 'gps'
 candidates = {[ns '/navsat'], [ns '/navsat/navsat0'], [ns '/gpsfix'], [ns '/gps/sample'], [ns '/gps'], '/iris/navsat', '/navsat', '/navsat/navsat0', '/iris/gpsfix', '/gps/sample', '/gps', '/ap/navsat/navsat0', '/ap/navsat', '/mavros/global_position/raw/fix', '/mavros/global_position/global'};
 case 'navsat'
 candidates = {[ns '/navsat/navsat0'], [ns '/navsat'], '/iris/navsat', '/navsat/navsat0', '/navsat', '/ap/navsat/navsat0', '/ap/navsat', '/mavros/global_position/raw/fix'};
 case 'baro'
 candidates = {[ns '/air_pressure'], [ns '/baro/sample'], [ns '/baro'], '/iris/air_pressure', '/baro/sample', '/baro', '/mavros/altitude'};
 case 'airspeed'
 candidates = {[ns '/air_speed'], [ns '/airspeed'], '/iris/air_speed', '/iris/airspeed', '/air_speed', '/airspeed', '/ap/air_speed', '/ap/airspeed'};
 case 'battery'
 candidates = {[ns '/battery'], [ns '/battery/state'], '/iris/battery', '/iris/battery/state', '/battery', '/battery/state', '/ap/battery', '/mavros/battery'};
 case 'status'
 candidates = {[ns '/status'], '/status', '/ap/status', '/mavros/state'};
 case 'rc'
 candidates = {[ns '/rc'], '/rc', '/ap/rc', '/mavros/rc/in'};
 case 'time'
 candidates = {[ns '/time'], '/time', '/ap/time'};
 case 'clock'
 candidates = {[ns '/clock'], [ns '/gz/clock'], '/clock', '/ap/clock', '/iris/clock', '/iris/gz/clock'};
 case 'attitude'
 candidates = {[ns '/attitude'], '/iris/attitude', '/ap/pose/filtered', '/mavros/local_position/pose', '/mavros/imu/data'};
 case 'attitude_target'
 if is_mavros
 candidates = {'/mavros/setpoint_position/local', '/mavros/setpoint_raw/local', [ns '/attitude_target']};
 else
 candidates = {[ns '/attitude_target'], [ns '/cmd_gps_pose'], '/iris/attitude_target', '/mavros/setpoint_position/local', '/mavros/setpoint_raw/local'};
 end
 case 'nav_controller_output'
 candidates = {[ns '/nav_controller_output'], '/mavros/nav_controller_output'};
 case 'wind'
 candidates = {[ns '/wind'], '/mavros/wind'};
 case 'pid_tuning'
 candidates = {[ns '/pid_tuning'], '/mavros/pid_tuning'};
 case 'cmd_gps_pose'
 if is_mavros
 candidates = {'/mavros/setpoint_position/local', '/mavros/setpoint_raw/local', [ns '/cmd_gps_pose']};
 else
 candidates = {[ns '/cmd_gps_pose'], '/iris/cmd_gps_pose', '/ap/cmd_gps_pose', '/ap/cmd_vel', '/mavros/setpoint_position/local', '/mavros/setpoint_raw/local'};
 end
 case 'state'
 candidates = {[ns '/state'], '/mavros/state'};
 otherwise
 candidates = {[ns '/' topic_key]};
 end

 candidates = unique(candidates, 'stable');
 end

 function candidates = get_service_candidates(service_key, ns)
 % Return ordered service candidates for ArduPilot and MAVROS-style bridges.
 if nargin < 2 || isempty(ns)
 ns = '/ap';
 end

 if ns(1) ~= '/'
 ns = ['/' ns];
 end
 if ns(end) == '/'
 ns = ns(1:end-1);
 end

 switch lower(service_key)
 case 'arm_motors'
	 candidates = {[ns '/arm_motors'], '/arm_motors', '/ap/arm_motors', '/mavros/cmd/arming'};
 case 'mode_switch'
	 candidates = {[ns '/mode_switch'], '/mode_switch', '/ap/mode_switch', '/mavros/set_mode'};
 case 'prearm_check'
	 candidates = {[ns '/prearm_check'], '/prearm_check', '/ap/prearm_check'};
 case 'takeoff'
	 candidates = {[ns '/experimental/takeoff'], '/experimental/takeoff', '/ap/experimental/takeoff'};
 case 'cmd_gps_pose'
	 candidates = {[ns '/cmd_gps_pose'], '/cmd_gps_pose', '/ap/cmd_gps_pose', '/mavros/setpoint_position/local', '/mavros/setpoint_raw/local'};
 otherwise
 candidates = {[ns '/' service_key]};
 end

 candidates = unique(candidates, 'stable');
 end

 function service = resolve_service(service_key, ns)
 % Resolve the first service candidate that actually exists in ros2 service list.
 candidates = SimulationConfig.get_service_candidates(service_key, ns);
 service = candidates{1};

 [status, output] = system('ros2 service list 2>/dev/null');
 if status ~= 0 || isempty(output)
 return;
 end

 available = strsplit(strtrim(output), newline);
 available(cellfun(@isempty, available)) = [];

 for i = 1:length(candidates)
 if any(strcmp(available, candidates{i}))
 service = candidates{i};
 return;
 end
 end
 end

 function exists = topic_exists(topic_name)
 % Return true when the topic is present in the current ros2 topic list.
 exists = false;
 if nargin < 1 || isempty(topic_name)
 return;
 end

 [status, output] = system('ros2 topic list 2>/dev/null');
 if status ~= 0 || isempty(output)
 return;
 end

 available = strsplit(strtrim(output), newline);
 available(cellfun(@isempty, available)) = [];
 exists = any(strcmp(available, topic_name));
 end

 function topic = resolve_topic_if_available(topic_key, ns)
 % Resolve a topic only if one of the candidates is actually available.
 candidates = SimulationConfig.get_topic_candidates(topic_key, ns);
 topic = '';
 for i = 1:length(candidates)
 if SimulationConfig.topic_exists(candidates{i})
 topic = candidates{i};
 return;
 end
 end
 end

 function topic = resolve_topic(topic_key, ns)
 % Resolve the first topic candidate that actually exists in ros2 topic list.
 candidates = SimulationConfig.get_topic_candidates(topic_key, ns);
 topic = candidates{1};

 [status, output] = system('ros2 topic list 2>/dev/null');
 if status ~= 0 || isempty(output)
 return;
 end

 available = strsplit(strtrim(output), newline);
 available(cellfun(@isempty, available)) = [];

 for i = 1:length(candidates)
 if any(strcmp(available, candidates{i}))
 topic = candidates{i};
 return;
 end
 end
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
