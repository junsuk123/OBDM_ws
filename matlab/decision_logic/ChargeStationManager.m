%% 
% and 

classdef ChargeStationManager < handle
 properties (Access = private)
 stations = struct(); % Info
 num_stations = 0; % 
 verbose = false;
 end
 
 methods
 %% 
 function obj = ChargeStationManager(varargin)
 % ChargeStationManager 
 
 p = inputParser;
 addParameter(p, 'verbose', false, @islogical);
 parse(p, varargin{:});
 
 obj.verbose = p.Results.verbose;
 
 % add
 obj.add_default_stations();
 
 if obj.verbose
 fprintf('[ChargeStationManager] %d Initialize/init\n', obj.num_stations);
 end
 end
 
 %% add
 function add_station(obj, name, latitude, longitude, altitude, varargin)
 % add
 % add_station(obj, name, lat, lon, alt, 'charg_rate', 500)
 
 p = inputParser;
 addParameter(p, 'charg_rate', 100, @isnumeric); % mAh/min
 addParameter(p, 'land_difficulty', 1, @isnumeric); % 1( )-5( )
 parse(p, varargin{:});
 
 obj.num_stations = obj.num_stations + 1;
 idx = obj.num_stations;
 
 obj.stations(idx).name = name;
 obj.stations(idx).latitude = latitude;
 obj.stations(idx).longitude = longitude;
 obj.stations(idx).altitude = altitude;
 obj.stations(idx).charg_rate = p.Results.charg_rate;
 obj.stations(idx).land_difficulty = p.Results.land_difficulty;
 obj.stations(idx).is_available = true;
 obj.stations(idx).last_serviced = now;
 
 if obj.verbose
 fprintf('[ChargeStationManager] add: %s (Lat: %.4f, Lon: %.4f)\n', ...
 name, latitude, longitude);
 end
 end
 
 %% 
 function [station_id, distance] = find_nearest_station(obj, lat, lon, varargin)
 % 
 % [id, distance] = find_nearest_station(obj, lat, lon, 'available_only', true)
 
 p = inputParser;
 addParameter(p, 'available_only', true, @islogical);
 parse(p, varargin{:});
 
 if obj.num_stations == 0
 error(' .');
 end
 
 min_distance = inf;
 station_id = -1;
 
 for i = 1:obj.num_stations
 % available check
 if p.Results.available_only && ~obj.stations(i).is_available
 continue;
 end
 
 % Haversine 
 distance = obj.calculate_distance(...
 lat, lon, ...
 obj.stations(i).latitude, obj.stations(i).longitude);
 
 if distance < min_distance
 min_distance = distance;
 station_id = i;
 end
 end
 
 distance = min_distance;
 
 if station_id == -1
 error(' available .');
 end
 
 if obj.verbose
 fprintf('[ChargeStationManager] : %s (: %.2f km)\n', ...
 obj.stations(station_id).name, distance);
 end
 end
 
 %% 
 function print_stations(obj)
 % Info 
 
 fprintf('\n========== ==========\n');
 fprintf(' %d \n\n', obj.num_stations);
 
 for i = 1:obj.num_stations
 station = obj.stations(i);
 status = '';
 if station.is_available
 status = '✓ available';
 else
 status = '✗ ';
 end
 
 fprintf('%d. %s\n', i, station.name);
 fprintf(' : %.4f°, %.4f° (: %.1f m)\n', ...
 station.latitude, station.longitude, station.altitude);
 fprintf(' : %d mAh/min\n', station.charg_rate);
 fprintf(' : %d/5\n', station.land_difficulty);
 fprintf(' : %s\n', status);
 fprintf(' : %s\n\n', datestr(station.last_serviced));
 end
 fprintf('================================\n\n');
 end
 
 %% Info 
 function station_info = get_station_info(obj, station_id)
 % Info 
 
 if station_id < 1 || station_id > obj.num_stations
 error('Invalid station ID: %d', station_id);
 end
 
 station_info = obj.stations(station_id);
 end
 
 %% 
 function nearby_ids = find_stations_in_radius(obj, lat, lon, radius_km)
 % 
 % radius_km: ( )
 
 nearby_ids = [];
 
 for i = 1:obj.num_stations
 distance = obj.calculate_distance(...
 lat, lon, ...
 obj.stations(i).latitude, obj.stations(i).longitude);
 
 if distance <= radius_km
 nearby_ids = [nearby_ids; i];
 end
 end
 
 if obj.verbose
 fprintf('[ChargeStationManager] %d %.1f km \n', ...
 length(nearby_ids), radius_km);
 end
 end
 
 %% 
 function set_station_availability(obj, station_id, is_available)
 % available set
 
 if station_id < 1 || station_id > obj.num_stations
 error('Invalid station ID: %d', station_id);
 end
 
 obj.stations(station_id).is_available = is_available;
 obj.stations(station_id).last_updated = now;
 
 if obj.verbose
 status = '';
 if is_available
 status = ' available';
 else
 status = ' ';
 end
 fprintf('[ChargeStationManager] %s change/modify: %s\n', ...
 obj.stations(station_id).name, status);
 end
 end
 end
 
 methods (Access = private)
 %% Haversine 
 function distance_km = calculate_distance(obj, lat1, lon1, lat2, lon2)
 % Haversine 
 % : 
 
 R = 6371; % (km)
 
 lat1_rad = deg2rad(lat1);
 lon1_rad = deg2rad(lon1);
 lat2_rad = deg2rad(lat2);
 lon2_rad = deg2rad(lon2);
 
 dlat = lat2_rad - lat1_rad;
 dlon = lon2_rad - lon1_rad;
 
 a = sin(dlat/2)^2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon/2)^2;
 c = 2 * asin(sqrt(a));
 
 distance_km = R * c;
 end
 
 %% add
 function add_default_stations(obj)
 % add ( )
 
 % 
 obj.add_station('Seoul_Station_A', 37.5665, 126.9780, 50, ...
 'charg_rate', 100, 'land_difficulty', 2);
 obj.add_station('Seoul_Station_B', 37.4979, 127.0276, 50, ...
 'charg_rate', 120, 'land_difficulty', 1);
 obj.add_station('Seoul_Station_C', 37.5513, 126.9725, 50, ...
 'charg_rate', 80, 'land_difficulty', 3);
 end
 end
end
