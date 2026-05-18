%% data 
% data 

classdef DataLogger < handle
 properties (Access = private)
 log_data = []; % log data 
 log_dir % log 
 log_file % log 
 is_logg = false;
 verbose = false;
 end
 
 methods
 %% 
 function obj = DataLogger(varargin)
 % DataLogger 
 
 p = inputParser;
 addParameter(p, 'log_dir', SimulationConfig.LOGGING_DIR, @ischar);
 addParameter(p, 'verbose', false, @islogical);
 parse(p, varargin{:});
 
 obj.log_dir = p.Results.log_dir;
 obj.verbose = p.Results.verbose;
 
 % log 
 if ~exist(obj.log_dir, 'dir')
 mkdir(obj.log_dir);
 end
 
 % log 
 timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
 obj.log_file = fullfile(obj.log_dir, ...
 sprintf('obdm_flight_log_%s.csv', timestamp));
 
 obj.is_logg = SimulationConfig.LOGGING_ENABLED;
 
 if obj.verbose
 fprintf('[DataLogger] complete/done: %s\n', obj.log_file);
 end
 end
 
 %% data 
 function log_state(obj, timestamp, state)
 % 
 
 if ~obj.is_logg
 return;
 end
 
 try
 % log data 
 log_entry = struct(...
 'timestamp', timestamp, ...
 'lat', state.position.latitude, ...
 'lon', state.position.longitude, ...
 'alt', state.position.altitude, ...
 'vx', state.velocity.vx, ...
 'vy', state.velocity.vy, ...
 'vz', state.velocity.vz, ...
 'roll', rad2deg(state.attitude.roll), ...
 'pitch', rad2deg(state.attitude.pitch), ...
 'yaw', rad2deg(state.attitude.yaw), ...
 'accel_x', state.imu.accel_x, ...
 'accel_y', state.imu.accel_y, ...
 'accel_z', state.imu.accel_z, ...
 'battery_percent', state.battery.percent, ...
 'battery_voltage', state.battery.voltage ...
 );
 
 % log 
 if ~isfile(obj.log_file)
 obj.write_header();
 end
 
 % data add
 obj.append_to_file(log_entry);
 
 catch ME
 if obj.verbose
 fprintf('[DataLogger] error: %s\n', ME.message);
 end
 end
 end
 
 %% log 
 function filepath = get_log_file(obj)
 filepath = obj.log_file;
 end
 end
 
 methods (Access = private)
 %% 
 function write_header(obj)
 header = 'timestamp,lat,lon,alt,vx,vy,vz,roll,pitch,yaw,accel_x,accel_y,accel_z,battery_percent,battery_voltage\n';
 fid = fopen(obj.log_file, 'w');
 fprintf(fid, header);
 fclose(fid);
 end
 
 %% data add
 function append_to_file(obj, log_entry)
 fid = fopen(obj.log_file, 'a');
 
 % data : CSV
 line = sprintf('%s,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n', ...
 datestr(log_entry.timestamp, 'HH:MM:SS'), ...
 log_entry.lat, log_entry.lon, log_entry.alt, ...
 log_entry.vx, log_entry.vy, log_entry.vz, ...
 log_entry.roll, log_entry.pitch, log_entry.yaw, ...
 log_entry.accel_x, log_entry.accel_y, log_entry.accel_z, ...
 log_entry.battery_percent, log_entry.battery_voltage);
 
 fprintf(fid, line);
 fclose(fid);
 end
 end
end
