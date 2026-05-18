%% data 
% API data and 

classdef WeatherDataProcessor < handle
 properties (Access = private)
 last_update = datetime('now');
 cached_weather = struct();
 verbose = false;
 api_key = ''; % API 
 end
 
 methods
 %% 
 function obj = WeatherDataProcessor(varargin)
 % WeatherDataProcessor 
 % WeatherDataProcessor('api_key', 'YOUR_KEY', 'verbose', true)
 
 p = inputParser;
 addParameter(p, 'api_key', '', @ischar);
 addParameter(p, 'verbose', false, @islogical);
 parse(p, varargin{:});
 
 obj.api_key = p.Results.api_key;
 obj.verbose = p.Results.verbose;
 
 % Initialize/init
 obj.cached_weather = obj.create_default_weather();
 
 if obj.verbose
 fprintf('[WeatherDataProcessor] Initialize/init complete/done\n');
 end
 end
 
 %% data ( data)
 function weather = get_weather(obj, latitude, longitude)
 % data 
 % API 
 
 % default (API )
 weather = obj.cached_weather;
 weather.latitude = latitude;
 weather.longitude = longitude;
 weather.timestamp = datetime('now');
 
 if obj.verbose
 fprintf('[WeatherDataProcessor] : (%.4f, %.4f)\n', ...
 latitude, longitude);
 end
 end
 
 %% 
 function impact = calculate_impact(obj, state, weather)
 % 
 %
 % : struct
 % - wind_effect: (-1.0 ~ 1.0)
 % - temperature_effect: ( )
 % - pressure_effect: 
 % - overall_risk: (0 ~ 1.0)
 
 impact = struct();
 
 % 1. 
 % 
 wind_speed = weather.wind_speed;
 aircraft_head = state.attitude.yaw; % 
 wind_direction = weather.wind_direction * pi / 180; % 
 
 % 
 relative_angle = aircraft_head - wind_direction;
 
 % (-1: , 0: , 1: )
 wind_factor = cos(relative_angle); % -1 ~ 1
 
 % (0 ~ 1)
 max_wind_speed = 15; % m/s
 normalized_wind = min(wind_speed / max_wind_speed, 1.0);
 impact.wind_effect = wind_factor * normalized_wind;
 impact.wind_speed = wind_speed;
 impact.wind_direction = weather.wind_direction;
 
 % 2. 
 % (15°C, ) 
 temperature = weather.temperature;
 standard_temp = 15;
 temp_difference = abs(temperature - standard_temp);
 
 % 10 3% 
 temp_effect = 1.0 - (temp_difference / 10) * 0.03;
 impact.temperature_effect = max(temp_effect, 0.7); % 70%
 impact.temperature = temperature;
 
 % 3. 
 % = = 
 pressure = weather.pressure;
 standard_pressure = 101325; % Pa
 
 pressure_ratio = pressure / standard_pressure;
 impact.pressure_effect = pressure_ratio;
 impact.pressure = pressure;
 
 % 4. 
 % and 
 precipitation = weather.precipitation;
 precip_effect = 1.0 - (precipitation / 100) * 0.2; % 100mm = 20% 
 impact.precipitation_effect = max(precip_effect, 0.8);
 impact.precipitation = precipitation;
 
 % 5. ( )
 humidity = weather.humidity;
 % 
 if humidity > 80
 humidity_effect = 1.0 - (humidity - 80) * 0.002;
 else
 humidity_effect = 1.0;
 end
 impact.humidity_effect = humidity_effect;
 impact.humidity = humidity;
 
 % 6. 
 % 
 weights = struct();
 weights.wind = 0.3;
 weights.temperature = 0.2;
 weights.pressure = 0.2;
 weights.precipitation = 0.2;
 weights.humidity = 0.1;
 
 % (0: ~ 1: )
 wind_risk = (1 - abs(wind_factor)) * normalized_wind;
 temp_risk = 1 - impact.temperature_effect;
 pressure_risk = 1 - impact.pressure_effect;
 precip_risk = 1 - impact.precipitation_effect;
 humidity_risk = 1 - impact.humidity_effect;
 
 overall_risk = ...
 weights.wind * wind_risk + ...
 weights.temperature * temp_risk + ...
 weights.pressure * pressure_risk + ...
 weights.precipitation * precip_risk + ...
 weights.humidity * humidity_risk;
 
 impact.overall_risk = overall_risk;
 
 % 7. 
 if overall_risk < 0.2
 impact.safety_level = ' ';
 elseif overall_risk < 0.4
 impact.safety_level = '';
 elseif overall_risk < 0.6
 impact.safety_level = '';
 elseif overall_risk < 0.8
 impact.safety_level = '';
 else
 impact.safety_level = '';
 end
 
 if obj.verbose
 fprintf('[WeatherDataProcessor] complete/done (: %.2f)\n', ...
 overall_risk);
 end
 end
 
 %% API data ( )
 function weather = fetch_from_api(obj, latitude, longitude)
 % ASOS API data 
 % (API )
 
 if isempty(obj.api_key)
 if obj.verbose
 fprintf('[WeatherDataProcessor] API . default \n');
 end
 weather = obj.create_default_weather();
 return;
 end
 
 try
 % TODO: API 
 % URL: https://data.kma.go.kr/api/...
 
 % temporary: default 
 weather = obj.create_default_weather();
 
 if obj.verbose
 fprintf('[WeatherDataProcessor] API data receive: (%.4f, %.4f)\n', ...
 latitude, longitude);
 end
 
 catch ME
 fprintf('[WeatherDataProcessor] API error: %s\n', ME.message);
 weather = obj.create_default_weather();
 end
 end
 
 %% 
 function print_weather(obj, weather)
 % Info 
 
 fprintf('\n========== Info ==========\n');
 fprintf(': (%.4f°, %.4f°)\n', weather.latitude, weather.longitude);
 fprintf(': %s\n\n', datestr(weather.timestamp));
 
 fprintf(': %.1f°C\n', weather.temperature);
 fprintf(': %.0f%%\n', weather.humidity);
 fprintf(': %.0f Pa (%.2f hPa)\n', weather.pressure, weather.pressure/100);
 
 fprintf('\n:\n');
 fprintf(' : %.1f m/s\n', weather.wind_speed);
 fprintf(' : %.0f°\n', weather.wind_direction);
 
 fprintf('\n:\n');
 fprintf(' : %.1f mm\n', weather.precipitation);
 fprintf(' : %s\n', weather.precipitation_type);
 
 fprintf('\n:\n');
 fprintf(' : %.0f m\n', weather.visibility);
 fprintf('===============================\n\n');
 end
 end
 
 methods (Access = private)
 %% data 
 function weather = create_default_weather(obj)
 % ( )
 
 weather = struct();
 weather.latitude = 37.5665; % 
 weather.longitude = 126.9780;
 weather.timestamp = datetime('now');
 weather.temperature = 20; % °C
 weather.humidity = 60; % %
 weather.pressure = 101325; % Pa ( )
 weather.wind_speed = 5; % m/s
 weather.wind_direction = 45; % ° ( )
 weather.wind_gust = 8; % m/s
 weather.precipitation = 0; % mm
 weather.precipitation_type = ''; % '', '', ''
 weather.visibility = 10000; % m ( )
 weather.dew_point = 12; % °C
 end
 end
end
