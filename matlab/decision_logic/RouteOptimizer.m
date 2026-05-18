%% optimization 
% , , 

classdef RouteOptimizer < handle
 properties (Access = private)
 charge_manager % ChargeStationManager 
 weather_processor % WeatherDataProcessor 
 verbose = false;
 end
 
 methods
 %% 
 function obj = RouteOptimizer(varargin)
 % RouteOptimizer 
 
 p = inputParser;
 addParameter(p, 'charge_manager', [], @(x) isa(x, 'ChargeStationManager'));
 addParameter(p, 'weather_processor', [], @(x) isa(x, 'WeatherDataProcessor'));
 addParameter(p, 'verbose', false, @islogical);
 parse(p, varargin{:});
 
 obj.charge_manager = p.Results.charge_manager;
 obj.weather_processor = p.Results.weather_processor;
 obj.verbose = p.Results.verbose;
 
 % ( )
 if isempty(obj.charge_manager)
 obj.charge_manager = ChargeStationManager('verbose', obj.verbose);
 end
 if isempty(obj.weather_processor)
 obj.weather_processor = WeatherDataProcessor('verbose', obj.verbose);
 end
 
 if obj.verbose
 fprintf('[RouteOptimizer] optimization Initialize/init complete/done\n');
 end
 end
 
 %% 
 function [best_station_id, decision_info] = find_optimal_charge_station(...
 obj, current_position, aircraft_state, weather, varargin)
 % 
 %
 % :
 % - current_position: [latitude, longitude, altitude]
 % - aircraft_state: struct ( , )
 % - weather: struct ( , )
 %
 % :
 % - best_station_id: optional ID
 % - decision_info: Info
 
 p = inputParser;
 addParameter(p, 'search_radius', 50, @isnumeric); % km
 addParameter(p, 'min_battery', SimulationConfig.MIN_BATTERY_MARGIN, @isnumeric);
 parse(p, varargin{:});
 
 % 
 candidate_ids = obj.charge_manager.find_stations_in_radius(...
 current_position(1), current_position(2), ...
 p.Results.search_radius);
 
 if isempty(candidate_ids)
 error(' ');
 end
 
 % 
 scores = [];
 info = struct();
 
 for i = 1:length(candidate_ids)
 station_id = candidate_ids(i);
 station = obj.charge_manager.get_station_info(station_id);
 
 % 1. 
 distance = obj.calculate_distance(...
 current_position(1), current_position(2), ...
 station.latitude, station.longitude);
 
 distance_score = 1 / (1 + distance); % 
 
 % 2. ( )
 altitude_diff = abs(current_position(3) - station.altitude);
 altitude_score = 1 / (1 + altitude_diff / 100); % 100m = 0.5 
 
 % 3. 
 flight_time = distance / SimulationConfig.AIRCRAFT_CRUISE_SPEED; % 
 consumption_rate = SimulationConfig.BATTERY_CONSUMPTION_CRUISE / 60; % %/ → %/ 
 expected_battery = aircraft_state.battery.percent - (flight_time * 3600 * consumption_rate);
 
 % ( )
 if expected_battery < p.Results.min_battery
 battery_score = 0; % available
 else
 battery_score = min(expected_battery / 100, 1.0);
 end
 
 % 4. 
 weather_impact = obj.weather_processor.calculate_impact(...
 aircraft_state, weather);
 
 % ( - )
 wind_land_score = (1 + weather_impact.wind_effect) / 2;
 
 % / 
 condition_score = weather_impact.temperature_effect * ...
 weather_impact.pressure_effect;
 
 weather_score = (wind_land_score + condition_score) / 2;
 
 % 5. ( )
 difficulty_score = 1 / (1 + station.land_difficulty);
 
 % 6. ( )
 charg_score = station.charg_rate / 200; % 200 mAh/min 
 
 % ( )
 weights = struct();
 weights.distance = 0.25;
 weights.altitude = 0.10;
 weights.battery = 0.25;
 weights.weather = 0.15;
 weights.difficulty = 0.15;
 weights.charg = 0.10;
 
 total_score = ...
 weights.distance * distance_score + ...
 weights.altitude * altitude_score + ...
 weights.battery * battery_score + ...
 weights.weather * weather_score + ...
 weights.difficulty * difficulty_score + ...
 weights.charg * charg_score;
 
 scores = [scores; total_score];
 
 % Info 
 info(i).station_id = station_id;
 info(i).station_name = station.name;
 info(i).distance = distance;
 info(i).altitude_diff = altitude_diff;
 info(i).expected_battery = expected_battery;
 info(i).distance_score = distance_score;
 info(i).altitude_score = altitude_score;
 info(i).battery_score = battery_score;
 info(i).weather_score = weather_score;
 info(i).difficulty_score = difficulty_score;
 info(i).charg_score = charg_score;
 info(i).total_score = total_score;
 
 if obj.verbose
 fprintf('[RouteOptimizer] %s: %.3f\n', ...
 station.name, total_score);
 end
 end
 
 % optional
 [~, best_idx] = max(scores);
 best_station_id = info(best_idx).station_id;
 
 % Info 
 decision_info = struct();
 decision_info.best_station = info(best_idx);
 decision_info.all_candidates = info;
 decision_info.num_candidates = length(candidate_ids);
 decision_info.decision_time = datetime('now');
 
 if obj.verbose
 fprintf('[RouteOptimizer] optional: %s (: %.3f)\n', ...
 info(best_idx).station_name, info(best_idx).total_score);
 end
 end
 
 %% 
 function route_plan = plan_route(obj, start_position, destination, ...
 aircraft_state, weather, varargin)
 % Start ( )
 %
 % :
 % - start_position: [latitude, longitude, altitude]
 % - destination: [latitude, longitude, altitude]
 % - aircraft_state: 
 % - weather: Info
 %
 % :
 % - route_plan: Info
 
 route_plan = struct();
 route_plan.start_position = start_position;
 route_plan.destination = destination;
 route_plan.waypoints = [];
 route_plan.is_feasible = false;
 
 % Start 
 total_distance = obj.calculate_distance(...
 start_position(1), start_position(2), ...
 destination(1), destination(2));
 
 % available 
 max_flight_time = (aircraft_state.battery.percent / 100) * ...
 (SimulationConfig.BATTERY_CAPACITY / SimulationConfig.BATTERY_CONSUMPTION_CRUISE);
 max_distance = max_flight_time * SimulationConfig.AIRCRAFT_CRUISE_SPEED / 3600;
 
 % available 
 if total_distance <= max_distance * 0.8 % 20% 
 route_plan.waypoints = destination;
 route_plan.is_feasible = true;
 route_plan.num_stops = 0;
 
 if obj.verbose
 fprintf('[RouteOptimizer] available\n');
 end
 return;
 end
 
 % 
 current_pos = start_position;
 waypoints = [current_pos];
 num_stops = 0;
 
 while true
 % 
 try
 [station_id, decision_info] = ...
 obj.find_optimal_charge_station(...
 current_pos, aircraft_state, weather);
 
 station = obj.charge_manager.get_station_info(station_id);
 
 % add
 current_pos = [station.latitude, station.longitude, station.altitude];
 waypoints = [waypoints; current_pos];
 num_stops = num_stops + 1;
 
 % available check
 dist_to_dest = obj.calculate_distance(...
 current_pos(1), current_pos(2), ...
 destination(1), destination(2));
 
 if dist_to_dest <= max_distance * 0.8
 waypoints = [waypoints; destination];
 route_plan.is_feasible = true;
 break;
 end
 
 % 
 if num_stops > 10
 error(' failed: ');
 end
 
 catch ME
 fprintf('[RouteOptimizer] error: %s\n', ME.message);
 break;
 end
 end
 
 route_plan.waypoints = waypoints;
 route_plan.num_stops = num_stops;
 
 if obj.verbose
 fprintf('[RouteOptimizer] complete/done: %d \n', ...
 num_stops);
 end
 end
 
 %% Info 
 function print_route_plan(obj, route_plan)
 % Info 
 
 fprintf('\n========== ==========\n');
 
 if route_plan.is_feasible
 fprintf(': ✓ available\n');
 else
 fprintf(': ✗ available\n');
 end
 
 fprintf(' : %d\n', route_plan.num_stops);
 fprintf('\n:\n');
 
 for i = 1:size(route_plan.waypoints, 1)
 waypoint = route_plan.waypoints(i, :);
 fprintf(' %d. (%.4f°, %.4f°, %.1f m)\n', ...
 i, waypoint(1), waypoint(2), waypoint(3));
 end
 
 fprintf('===============================\n\n');
 end
 end
 
 methods (Access = private)
 %% Haversine 
 function distance_km = calculate_distance(obj, lat1, lon1, lat2, lon2)
 % Haversine 
 
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
 end
end
