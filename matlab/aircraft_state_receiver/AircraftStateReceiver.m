%% receive 
% ROS2 data receive 

classdef AircraftStateReceiver < handle
 properties (Access = private)
 % ROS2 subscribe (subscribers)
 subscriber_node
 subscriber_tf % / 
 subscriber_imu % IMU 
 subscriber_gps % GPS
 subscriber_baro % 
 subscriber_battery % 
 
 % 
 current_state % struct: 
 
 % 
 state_updated = false;
 
 % data 
 data_logger
 
 % 
 last_update_time
 end
 
 properties (Access = public)
 is_subscribed = false;
 verbose = false;
 end
 
 methods
 %% 
 function obj = AircraftStateReceiver(varargin)
 % AircraftStateReceiver 
 % : receiver = AircraftStateReceiver('verbose', true);
 
 % 
 p = inputParser;
 addParameter(p, 'verbose', false, @islogical);
 addParameter(p, 'enable_logg', true, @islogical);
 parse(p, varargin{:});
 
 obj.verbose = p.Results.verbose;
 obj.last_update_time = tic;
 
 % 
 obj.current_state = obj.create_empty_state();
 
 % data 
 if p.Results.enable_logg
 obj.data_logger = DataLogger();
 end
 
 if obj.verbose
 fprintf('[AircraftStateReceiver] complete/done\n');
 end
 end
 
 %% ROS2 subscribe set
 function subscribe_to_topics(obj)
 % ROS2 topic subscribe Start
 
 if obj.verbose
 fprintf('[AircraftStateReceiver] ROS2 topic subscribe Start...\n');
 end

 try
 ros2_manager = ROS2Manager('verbose', obj.verbose);
 ros2_manager = ros2_manager.Initialize();
 current_ns = ros2_manager.get_namespace();

 obj.subscriber_node = ros2node('/aircraft_state_receiver');

 resolved_tf = SimulationConfig.resolve_topic('tf', current_ns);
 resolved_imu = SimulationConfig.resolve_topic('imu', current_ns);
 resolved_odom = SimulationConfig.resolve_topic('odometry', current_ns);
 resolved_gps = SimulationConfig.resolve_topic_if_available('gps', current_ns);
 resolved_baro = SimulationConfig.resolve_topic('baro', current_ns);
 resolved_battery = SimulationConfig.resolve_topic('battery', current_ns);

 obj.subscriber_tf = obj.try_subscribe(resolved_tf, 'tf2_msgs/msg/TFMessage', @obj.callback_tf);
 obj.subscriber_imu = obj.try_subscribe(resolved_imu, 'sensor_msgs/msg/Imu', @obj.callback_imu);
 obj.subscriber_odometry = obj.try_subscribe(resolved_odom, 'nav_msgs/msg/Odometry', @obj.callback_odom);
 if ~isempty(resolved_gps)
	 obj.subscriber_gps = obj.try_subscribe(resolved_gps, 'sensor_msgs/msg/NavSatFix', @obj.callback_gps);
 end
 obj.subscriber_baro = obj.try_subscribe(resolved_baro, 'sensor_msgs/msg/FluidPressure', @obj.callback_baro);
 obj.subscriber_battery = obj.try_subscribe(resolved_battery, 'sensor_msgs/msg/BatteryState', @obj.callback_battery);

 obj.is_subscribed = ~isempty(obj.subscriber_tf) || ~isempty(obj.subscriber_imu) || ...
 ~isempty(obj.subscriber_gps) || ~isempty(obj.subscriber_baro) || ~isempty(obj.subscriber_battery);

 if obj.verbose
 fprintf('[AircraftStateReceiver] topic subscribe result:\n');
 fprintf('  TF: %s\n', resolved_tf);
 fprintf('  IMU: %s\n', resolved_imu);
 fprintf('  ODOM: %s\n', resolved_odom);
 if isempty(resolved_gps)
 fprintf('  GPS: (not published)\n');
 else
 fprintf('  GPS: %s\n', resolved_gps);
 end
 fprintf('  BARO: %s\n', resolved_baro);
 fprintf('  BATTERY: %s\n', resolved_battery);
 end

 if ~obj.is_subscribed
 fprintf('[AircraftStateReceiver] subscribe error: no topic matched\n');
 end
 end
 end
 
 %% 
 function state = get_current_state(obj)
 % 
 state = obj.current_state;
 state.is_valid = obj.state_updated;
 end
 
 %% Info 
 function print_state(obj)
 % 
 
 state = obj.current_state;
 
 fprintf('\n========== ==========\n');
 fprintf(': %s\n', datestr(state.timestamp));
 
 fprintf('\n--- ---\n');
 fprintf(': %.6f°\n', state.position.latitude);
 fprintf(': %.6f°\n', state.position.longitude);
 fprintf(': %.2f m\n', state.position.altitude);
 
 fprintf('\n--- ---\n');
 fprintf('(Vx): %.2f m/s\n', state.velocity.vx);
 fprintf('(Vy): %.2f m/s\n', state.velocity.vy);
 fprintf('(Vz): %.2f m/s\n', state.velocity.vz);
 fprintf(': %.2f m/s\n', state.velocity.speed);
 
 fprintf('\n--- ---\n');
 fprintf('(Roll): %.2f°\n', rad2deg(state.attitude.roll));
 fprintf('(Pitch): %.2f°\n', rad2deg(state.attitude.pitch));
 fprintf('(Yaw): %.2f°\n', rad2deg(state.attitude.yaw));
 
 fprintf('\n--- ---\n');
 fprintf('(X): %.3f m/s²\n', state.imu.accel_x);
 fprintf('(Y): %.3f m/s²\n', state.imu.accel_y);
 fprintf('(Z): %.3f m/s²\n', state.imu.accel_z);
 
 fprintf('\n--- ---\n');
 fprintf(' : %.1f%%\n', state.battery.percent);
 fprintf(' : %.2f V\n', state.battery.voltage);
 fprintf(' : %.2f A\n', state.battery.current);
 
 fprintf('\ndata : %s\n', onoff(state.is_valid));
 fprintf('=================================\n\n');
 end
 
 %% subscribe release
 function unsubscribe_from_topics(obj)
 % topic subscribe release
 
 clear obj.subscriber_tf;
 clear obj.subscriber_imu;
 clear obj.subscriber_odometry;
 clear obj.subscriber_gps;
 clear obj.subscriber_baro;
 clear obj.subscriber_battery;
 clear obj.subscriber_node;
 
 obj.is_subscribed = false;
 
 if obj.verbose
 fprintf('[AircraftStateReceiver] topic subscribe release\n');
 end
 end
 
 %% ( )
 function delete(obj)
 % subscribe release
 if obj.is_subscribed
 obj.unsubscribe_from_topics();
 end
 end
 end
 
 methods (Access = private)
 %% 

function sub = try_subscribe(obj, topic_name, message_type, callback_handle)
 % Try to subscribe to a topic and return [] if it is unavailable.
 sub = [];
 try
	sub = ros2subscriber(obj.subscriber_node, topic_name, message_type, callback_handle);
 catch
 end
 end
 
 function callback_tf(obj, msg)
 % Transform ( / ) 
 
 try
 % 
 obj.current_state.position.x = msg.Transform.Translation.X;
 obj.current_state.position.y = msg.Transform.Translation.Y;
 obj.current_state.position.z = msg.Transform.Translation.Z;
 
 % (Quaternion → Euler)
 quat = [msg.Transform.Rotation.W, ...
 msg.Transform.Rotation.X, ...
 msg.Transform.Rotation.Y, ...
 msg.Transform.Rotation.Z];
 
 % Quaternion Euler 
 [roll, pitch, yaw] = quat2euler(quat);
 obj.current_state.attitude.roll = roll;
 obj.current_state.attitude.pitch = pitch;
 obj.current_state.attitude.yaw = yaw;
 
 % GPS (simulation )
 % : GPS topic 
 obj.current_state.timestamp = now;
 obj.state_updated = true;
 
 catch ME
 if obj.verbose
 fprintf('[AircraftStateReceiver] TF error: %s\n', ME.message);
 end
 end
 end
 
 function callback_imu(obj, msg)
 % IMU 
 
 try
 % 
 obj.current_state.imu.accel_x = msg.LinearAcceleration.X;
 obj.current_state.imu.accel_y = msg.LinearAcceleration.Y;
 obj.current_state.imu.accel_z = msg.LinearAcceleration.Z;
 
 % 
 obj.current_state.imu.gyro_x = msg.AngularVelocity.X;
 obj.current_state.imu.gyro_y = msg.AngularVelocity.Y;
 obj.current_state.imu.gyro_z = msg.AngularVelocity.Z;
 
 obj.current_state.timestamp = now;
 
 catch ME
 if obj.verbose
 fprintf('[AircraftStateReceiver] IMU error: %s\n', ME.message);
 end
 end

 function callback_odom(obj, msg)
 % ODOM

 try
 obj.current_state.position.x = msg.Pose.Pose.Position.X;
 obj.current_state.position.y = msg.Pose.Pose.Position.Y;
 obj.current_state.position.z = msg.Pose.Pose.Position.Z;
 obj.current_state.position.altitude = msg.Pose.Pose.Position.Z;

 obj.current_state.velocity.vx = msg.Twist.Twist.Linear.X;
 obj.current_state.velocity.vy = msg.Twist.Twist.Linear.Y;
 obj.current_state.velocity.vz = msg.Twist.Twist.Linear.Z;
 obj.current_state.velocity.speed = sqrt(obj.current_state.velocity.vx^2 + obj.current_state.velocity.vy^2 + obj.current_state.velocity.vz^2);

 obj.current_state.timestamp = now;
 obj.state_updated = true;

 catch ME
 if obj.verbose
 fprintf('[AircraftStateReceiver] ODOM error: %s\n', ME.message);
 end
 end
 end
 end
 
 function callback_gps(obj, msg)
 % GPS 
 
 try
 % GPS ( , , )
 obj.current_state.position.latitude = msg.Latitude;
 obj.current_state.position.longitude = msg.Longitude;
 obj.current_state.position.altitude = msg.Altitude;
 
 % GPS quality Info
 obj.current_state.gps.status = msg.Status.Status;
 obj.current_state.gps.num_satellites = msg.Status.Service;
 
 obj.current_state.timestamp = now;
 obj.state_updated = true;
 
 catch ME
 if obj.verbose
 fprintf('[AircraftStateReceiver] GPS error: %s\n', ME.message);
 end
 end
 end
 
 function callback_baro(obj, msg)
 % 
 
 try
 % data
 obj.current_state.barometer.pressure = msg.Fluid_pressure;
 
 % ( )
 obj.current_state.barometer.altitude = ...
 44330 * (1 - (msg.Fluid_pressure / 101325) ^ (1/5.255));
 
 obj.current_state.timestamp = now;
 
 catch ME
 if obj.verbose
 fprintf('[AircraftStateReceiver] error: %s\n', ME.message);
 end
 end
 end
 
 function callback_battery(obj, msg)
 % 
 
 try
 % Info
 if isfield(msg, 'Percentage')
 obj.current_state.battery.percent = msg.Percentage * 100;
 else
 obj.current_state.battery.percent = 50; % default
 end
 
 obj.current_state.battery.voltage = msg.Voltage;
 obj.current_state.battery.current = msg.Current;
 obj.current_state.battery.charge = msg.Charge;
 obj.current_state.battery.capacity = msg.Capacity;
 obj.current_state.battery.design_capacity = msg.Design_capacity;
 
 obj.current_state.timestamp = now;
 
 catch ME
 if obj.verbose
 fprintf('[AircraftStateReceiver] error: %s\n', ME.message);
 end
 end
 end
 
 %% 
 
 function state = create_empty_state(obj)
 % 
 
 state = struct();
 
 % 
 state.position = struct(...
 'latitude', 0, ...
 'longitude', 0, ...
 'altitude', 0, ...
 'x', 0, ...
 'y', 0, ...
 'z', 0 ...
 );
 
 % 
 state.velocity = struct(...
 'vx', 0, ...
 'vy', 0, ...
 'vz', 0, ...
 'speed', 0 ...
 );
 
 % 
 state.attitude = struct(...
 'roll', 0, ...
 'pitch', 0, ...
 'yaw', 0 ...
 );
 
 % IMU 
 state.imu = struct(...
 'accel_x', 0, ...
 'accel_y', 0, ...
 'accel_z', 0, ...
 'gyro_x', 0, ...
 'gyro_y', 0, ...
 'gyro_z', 0 ...
 );
 
 % GPS
 state.gps = struct(...
 'status', 0, ...
 'num_satellites', 0 ...
 );
 
 % 
 state.barometer = struct(...
 'pressure', 101325, ...
 'altitude', 0 ...
 );
 
 % 
 state.battery = struct(...
 'percent', 100, ...
 'voltage', 12.0, ...
 'current', 0, ...
 'charge', 5000, ...
 'capacity', 5000, ...
 'design_capacity', 5000 ...
 );
 
 % data
 state.timestamp = now;
 state.is_valid = false;
 end
 end
end

%% : Quaternion Euler 
function [roll, pitch, yaw] = quat2euler(quat)
 % quat = [w, x, y, z]
 
 w = quat(1);
 x = quat(2);
 y = quat(3);
 z = quat(4);
 
 % Roll (Phi)
 sinr_cosp = 2 * (w * x + y * z);
 cosr_cosp = 1 - 2 * (x * x + y * y);
 roll = atan2(sinr_cosp, cosr_cosp);
 
 % Pitch (Theta)
 sinp = 2 * (w * y - z * x);
 if abs(sinp) >= 1
 pitch = copysign(pi / 2, sinp);
 else
 pitch = asin(sinp);
 end
 
 % Yaw (Psi)
 siny_cosp = 2 * (w * z + x * y);
 cosy_cosp = 1 - 2 * (y * y + z * z);
 yaw = atan2(siny_cosp, cosy_cosp);
end

%% : copysign
function result = copysign(magnitude, sign_value)
 if sign_value < 0
 result = -abs(magnitude);
 else
 result = abs(magnitude);
 end
end

%% : 
function output = onoff(flag)
 if flag
 output = '';
 else
 output = '';
 end
end
