%% ROS2Manager - ROS2 namespace and Topic 
% ROS2 detect, namespace detect, Topic 

classdef ROS2Manager
 properties (Constant)
 DEFAULT_NAMESPACE = '/alti_transition'; % namespace
 AUTO_DETECT_TIMEOUT = 5; % detect ( )
 end
 
 properties
 detected_namespace = ''; % detect namespace
 is_Initialized = false;
 verbose = false;
 end
 
 methods
 function obj = ROS2Manager(varargin)
 % 
 % Usage: mgr = ROS2Manager('verbose', true);
 
 p = inputParser;
 addParameter(p, 'verbose', false, @islogical);
 parse(p, varargin{:});
 
 obj.verbose = p.Results.verbose;
 obj.detected_namespace = ROS2Manager.DEFAULT_NAMESPACE;
 end
 
 function obj = Initialize(obj)
 % Initialize ROS2 environment and auto-detect namespace
 
 if obj.verbose
 fprintf('[ROS2Manager] Initializing ROS2 environment...\n');
 end
 
 % Check ROS_DOMAIN_ID
 domain_id = getenv('ROS_DOMAIN_ID');
 if isempty(domain_id)
 setenv('ROS_DOMAIN_ID', '0');
 if obj.verbose
 fprintf('[ROS2Manager] ROS_DOMAIN_ID set to 0\n');
 end
 end
 
 % Auto-detect namespace
 obj = obj.auto_detect_namespace();
 
 obj.is_Initialized = true;
 
 if obj.verbose
 fprintf('[ROS2Manager] Initialization complete. Namespace: %s\n', obj.detected_namespace);
 end
 end
 
 function obj = auto_detect_namespace(obj)
 % Auto-detect namespace from ROS2 topic list
 % Priority: /ap → /plane1 → other namespaces
 
 if obj.verbose
 fprintf('[ROS2Manager] Detecting namespace...\n');
 end
 
 [status, output] = system('ros2 topic list 2>/dev/null');
 
 if status ~= 0 || isempty(output)
 if obj.verbose
 fprintf('[ROS2Manager] Failed to get topic list. Using default: %s\n', ROS2Manager.DEFAULT_NAMESPACE);
 end
 obj.detected_namespace = ROS2Manager.DEFAULT_NAMESPACE;
 return;
 end
 
 % Check MAVROS namespace first when present.
 if contains(output, '/mavros/state') || contains(output, '/mavros/imu/data') || contains(output, '/mavros/local_position/odom')
 obj.detected_namespace = '/mavros';
 if obj.verbose
 fprintf('[ROS2Manager] /mavros namespace detected\n');
 end
 return;
 end

 % Check /ap namespace first
 if contains(output, '/ap/navsat') || contains(output, '/ap/imu/experimental/data') || contains(output, '/ap/battery') || contains(output, '/ap/airspeed')
 obj.detected_namespace = '/ap';
 if obj.verbose
 fprintf('[ROS2Manager] /ap namespace detected\n');
 end
 return;
 end
 
 % Check /plane1 namespace
 if contains(output, '/plane1/gps/sample') || contains(output, '/plane1/imu/sample')
 obj.detected_namespace = '/plane1';
 if obj.verbose
 fprintf('[ROS2Manager] /plane1 namespace detected\n');
 end
 return;
 end
 
 % Check /alti_transition namespace (default namespace)
 if contains(output, '/alti_transition/imu') || contains(output, '/alti_transition/odometry') || contains(output, '/alti_transition/tf')
 obj.detected_namespace = '/alti_transition';
 if obj.verbose
 fprintf('[ROS2Manager] /alti_transition namespace detected\n');
 end
 return;
 end
 
 % Search for other namespaces
 lines = strsplit(output, newline);
 for i = 1:length(lines)
 line = strtrim(lines{i});
 if contains(line, '/imu') && ~contains(line, '/gz')
 parts = strsplit(line, '/');
 if length(parts) >= 2
 ns = ['/' parts{2}];
 obj.detected_namespace = ns;
 if obj.verbose
 fprintf('[ROS2Manager] Custom namespace detected: %s\n', ns);
 end
 return;
 end
 end
 end
 
 % Use default
 obj.detected_namespace = ROS2Manager.DEFAULT_NAMESPACE;
 if obj.verbose
 fprintf('[ROS2Manager] No namespace detected. Using default: %s\n', ROS2Manager.DEFAULT_NAMESPACE);
 end
 end
 
 function ns = get_namespace(obj)
 % detect namespace 
 ns = obj.detected_namespace;
 end
 
 function topic = get_topic(obj, topic_name)
 % Topic 
 % Usage: imu_topic = mgr.get_topic('imu/sample');
 topic = sprintf('%s/%s', obj.detected_namespace, topic_name);
 end
 
 function service = get_service(obj, service_name)
 % 
 % Usage: arm_svc = mgr.get_service('arm_motors');
 service = sprintf('%s/%s', obj.detected_namespace, service_name);
 end
 
function topics = get_standard_topics(obj)
	% Return likely topic names for the detected namespace.
	% Match the topics actually published by the current ArduPilot + Gazebo bridge.
	ns = obj.detected_namespace;
	topics = struct(...
		'tf', SimulationConfig.resolve_topic('tf', ns), ...
		'imu', SimulationConfig.resolve_topic('imu', ns), ...
		'gps', SimulationConfig.resolve_topic('gps', ns), ...
		'navsat', SimulationConfig.resolve_topic('navsat', ns), ...
		'odometry', SimulationConfig.resolve_topic('odometry', ns), ...
		'baro', SimulationConfig.resolve_topic('baro', ns), ...
		'airspeed', SimulationConfig.resolve_topic('airspeed', ns), ...
		'battery', SimulationConfig.resolve_topic('battery', ns), ...
		'rc', SimulationConfig.resolve_topic('rc', ns), ...
		'status', SimulationConfig.resolve_topic('status', ns), ...
		'time', SimulationConfig.resolve_topic('time', ns), ...
		'clock', SimulationConfig.resolve_topic('clock', ns), ...
		'attitude', SimulationConfig.resolve_topic('attitude', ns), ...
		'attitude_target', SimulationConfig.resolve_topic('attitude_target', ns), ...
		'nav_controller_output', SimulationConfig.resolve_topic('nav_controller_output', ns), ...
		'wind', SimulationConfig.resolve_topic('wind', ns), ...
		'pid_tuning', SimulationConfig.resolve_topic('pid_tuning', ns) ...
	);
    
	% Also keep legacy variants as fallback (not returned but used by check logic)
	% e.g. '/<ns>/imu/sample' or '/<ns>/gps/sample' may exist in some setups
end
 
 function topics = get_standard_services(obj)
 % 
 ns = obj.detected_namespace;
 topics = struct(...
	'arm_motors', SimulationConfig.resolve_service('arm_motors', ns), ...
	'mode_switch', SimulationConfig.resolve_service('mode_switch', ns), ...
	'cmd_gps_pose', SimulationConfig.resolve_service('cmd_gps_pose', ns) ...
 );
 end
 
 function is_available = check_topic_available(obj, topic_name)
 % Topic check
 [status, output] = system('ros2 topic list 2>/dev/null');
 is_available = (status == 0) && contains(output, topic_name);
 end
 
 function list = list_available_topics(obj)
 % enabled/enable Topic 
 [status, output] = system('ros2 topic list 2>/dev/null');
 if status == 0
 list = strsplit(output, newline);
 list(cellfun(@isempty, list)) = []; % 
 system_topics = {'/parameter_events', '/rosout'};
 list = list(~ismember(list, system_topics));
 else
 list = {};
 end
 end
 
 function print_info(obj)
 % Print ROS2 information
 fprintf('\n╔════════════════════════════════╗\n');
 fprintf('║ ROS2Manager Info ║\n');
 fprintf('╚════════════════════════════════╝\n');
 fprintf('Initialized: %d\n', obj.is_Initialized);
 fprintf('Detected Namespace: %s\n', obj.detected_namespace);
 fprintf('ROS_DOMAIN_ID: %s\n', getenv('ROS_DOMAIN_ID'));
 
 topics = obj.get_standard_topics();
 services = obj.get_standard_services();
 
 fprintf('\nStandard Topics:\n');
 fprintf(' TF: %s\n', topics.tf);
 fprintf(' IMU: %s\n', topics.imu);
 fprintf(' GPS: %s\n', topics.gps);
 fprintf(' Odometry: %s\n', topics.odometry);
 fprintf(' Baro: %s\n', topics.baro);
 fprintf(' Battery: %s\n', topics.battery);
 fprintf(' Attitude: %s\n', topics.attitude);
 fprintf(' Attitude Target: %s\n', topics.attitude_target);
 fprintf(' Nav Controller Output: %s\n', topics.nav_controller_output);
 fprintf(' Wind: %s\n', topics.wind);
 fprintf(' PID Tuning: %s\n', topics.pid_tuning);
 
 fprintf('\nStandard Services:\n');
 fprintf(' ARM: %s\n', services.arm_motors);
 fprintf(' MODE: %s\n', services.mode_switch);
 fprintf(' POSE: %s\n', services.cmd_gps_pose);
 fprintf('\n');
 end
 end
end
