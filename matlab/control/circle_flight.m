%% 
% → → MATLAB 
% :
% cd ~/OBDM_ws/matlab
% addpath(genpath(pwd))
% run('control/circle_flight.m')

clear; clc;
fprintf('\n=== MATLAB Circle Flight Controller (Demo) ===\n');

% add (SimulationConfig required)
addpath(genpath(pwd));

% set
RADIUS_M = 100; % (m)
NUM_POINTS = 36; % 
ALTITUDE_AGL = 30; % ( )
SPEED = SimulationConfig.AIRCRAFT_CRUISE_SPEED; % m/s
PAUSE_BETWEEN_CMD = 1.0; % (s)

try
 fprintf(' add\n');

 % namespace detect
 fprintf('namespace detect ...\n');
 [status, output] = system('ros2 topic list 2>/dev/null');
 detected_ns = '/ap'; % default
 
 if status == 0 && ~isempty(output)
 % /ap namespace 
 if contains(output, '/ap/gps')
 detected_ns = '/ap';
 else
 % namespace 
 lines = strsplit(output, newline);
 for i = 1:length(lines)
 line = strtrim(lines{i});
 if contains(line, '/gps') && ~contains(line, '/gz')
 parts = strsplit(line, '/');
 if length(parts) >= 2
 detected_ns = ['/' parts{2}];
 break;
 end
 end
 end
 end
 end
 
 fprintf('detect namespace: %s\n', detected_ns);
 
 % ROS2 topic/ ( detect namespace)
 ARM_SVC = sprintf('%s/arm_motors', detected_ns);
 MODE_SVC = sprintf('%s/mode_switch', detected_ns);
 GOAL_TOPIC = sprintf('%s/cmd_gps_pose', detected_ns);

 % Gazebo(ArduPlane) check , 
 fprintf('ROS2 topic check ... ( 30 )\n');
 max_wait = 30; t0 = tic;
 topic_ready = false;
 gps_topic = sprintf('%s/gps/sample', detected_ns);
 while toc(t0) < max_wait
 [s, out] = system(sprintf('ros2 topic list 2>/dev/null | grep "%s" || true', gps_topic));
 if s == 0 && ~isempty(strtrim(out))
 topic_ready = true; break;
 end
 if toc(t0) > 2 && ~exist(fullfile(pwd,'../simulation/launch/launch_plane.sh'),'file')
 % no local launch script, break and try creat receiver (user may manage launch externally)
 break;
 end
 % try to start the launch script once if topic not ready after a short delay
 if toc(t0) < 5
 % continue wait
 elseif toc(t0) < 6
 % attempt to start the launch script in background
 launch_script = fullfile('..','simulation','launch','launch_plane.sh');
 if exist(launch_script,'file')
 fprintf('simulation Start ()...\n');
 cmd = sprintf('bash %s > /tmp/obdm_plane_launch.log 2>&1 &', launch_script);
 system(cmd);
 end
 end
 pause(1.0);
 end

 % receive and subscribe
 receiver = AircraftStateReceiver('verbose', true, 'enable_logg', false);
 receiver.subscribe_to_topics();
 pause(1.0);

 % 
 st = receiver.get_current_state();
 if ~st.is_valid
 warn(' Info receive . ...');
 pause(3.0);
 st = receiver.get_current_state();
 end

 lat0 = st.position.latitude;
 lon0 = st.position.longitude;
 alt0 = st.position.altitude;

 fprintf(' : LAT=%.6f LON=%.6f ALT=%.1f m\n', lat0, lon0, alt0);

 % 
 arm_client = ros2serviceclient(ARM_SVC, 'ardupilot_msgs/srv/ArmMotors');
 mode_client = ros2serviceclient(MODE_SVC, 'ardupilot_msgs/srv/ModeSwitch');
 goal_pub = ros2publisher(GOAL_TOPIC, 'ardupilot_msgs/msg/GlobalPosition');

 % 1) (arm)
 fprintf(' ...\n');
 arm_req = ros2message(arm_client);
 arm_req.arm = true;
 try
 arm_resp = call(arm_client, arm_req, 'Timeout', 5);
 fprintf('Arm : %s\n', mat2str(arm_resp));
 catch
 warn('Arm failed (). .');
 end

 % 2) 
 PLANE_MODE_TAKEOFF = 13;
 PLANE_MODE_GUIDED = 15;
 fprintf(' : TAKEOFF\n');
 mode_req = ros2message(mode_client);
 mode_req.mode = PLANE_MODE_TAKEOFF;
 try
 call(mode_client, mode_req, 'Timeout', 5);
 catch
 warn(' () failed');
 end

 % : (simulation )
 target_alt = alt0 + ALTITUDE_AGL;
 fprintf(' : %.1f m (AGL +%d m)\n', target_alt, ALTITUDE_AGL);
 tstart = tic;
 while receiver.get_current_state().position.altitude < target_alt - 2 && toc(tstart) < 60
 fprintf(' : %.1f m\n', receiver.get_current_state().position.altitude);
 pause(1.0);
 end

 % 3) GUIDED 
 fprintf(' : GUIDED\n');
 mode_req = ros2message(mode_client);
 mode_req.mode = PLANE_MODE_GUIDED;
 try
 call(mode_client, mode_req, 'Timeout', 5);
 catch
 warn(' (GUIDED) failed');
 end

 % 4) publish
 fprintf(' Start ( %d m, %d )\n', RADIUS_M, NUM_POINTS);

 % ( 1 deg = ~111319.9 m)
 deg_per_m_lat = 1 / 111319.9;
 deg_per_m_lon = 1 / (111319.9 * cosd(lat0));

 for k = 1:NUM_POINTS
 theta = 2*pi*(k-1)/NUM_POINTS;
 dx = RADIUS_M * cos(theta);
 dy = RADIUS_M * sin(theta);

 lat_cmd = lat0 + dy * deg_per_m_lat;
 lon_cmd = lon0 + dx * deg_per_m_lon;
 alt_cmd = target_alt;

 msg = ros2message(goal_pub);
 msg.latitude = lat_cmd;
 msg.longitude = lon_cmd;
 msg.altitude = alt_cmd;
 msg.coordinate_frame = uint8(5); % FRAME_GLOBAL_INT
 msg.header.frame_id = 'map';

 send(goal_pub, msg);
 fprintf('[%02d/%02d] publish: LAT=%.6f LON=%.6f ALT=%.1f\n', k, NUM_POINTS, lat_cmd, lon_cmd, alt_cmd);
 pause(PAUSE_BETWEEN_CMD);
 end

 % 5) ( ): 
 fprintf(' : \n');
 land_msg = ros2message(goal_pub);
 land_msg.latitude = lat0;
 land_msg.longitude = lon0;
 land_msg.altitude = alt0 + 2; % 
 land_msg.coordinate_frame = uint8(5);
 land_msg.header.frame_id = 'map';
 send(goal_pub, land_msg);
 pause(5.0);

 fprintf(' \n');
 arm_req = ros2message(arm_client);
 arm_req.arm = false;
 try
 call(arm_client, arm_req, 'Timeout', 5);
 catch
 warn(' failed');
 end

 fprintf(' complete/done\n');

catch ME
 fprintf('error : %s\n', ME.message);
 rethrow(ME);
end

% cleanup
try
 receiver.unsubscribe_from_topics();
 clear receiver;
catch
end

fprintf(' terminate\n');
