%% DataLogger - Save trajectory and virtual wind sensor data to CSV files
% Saves odometry and wind vector data to matlab/data/test/<timestamp>/ folder

classdef DataLogger < handle
    properties
        data_dir
        test_dir
        timestamp_str
        traj_file
        wind_file
        traj_fid
        wind_fid
        verbose = true
        start_time = 0
        ros_node
        sub_odom
        sub_airspeed
        sub_wind
        timer_obj
        namespace = '/iris'
        latest_airspeed_row = []
        latest_airspeed_stamp = NaN
        last_logged_airspeed_stamp = NaN
        latest_wind_row = []
        latest_wind_stamp = NaN
        last_logged_wind_stamp = NaN
        latest_odom_row = []
        latest_odom_stamp = NaN
        last_logged_odom_stamp = NaN
    end

    methods
        function obj = DataLogger(varargin)
            p = inputParser;
            addParameter(p, 'base_dir', 'matlab/data', @ischar);
            addParameter(p, 'namespace', '/iris', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});

            obj.verbose = p.Results.verbose;
            obj.namespace = p.Results.namespace;
            obj.start_time = tic;
            
            % Create data directory structure
            if startsWith(p.Results.base_dir, filesep)
                obj.data_dir = p.Results.base_dir;
            else
                obj.data_dir = fullfile(pwd, p.Results.base_dir);
            end
            obj.test_dir = fullfile(obj.data_dir, 'test');
            
            % Create timestamp string (YYYYMMDD_HHMMSS)
            now_dt = datetime('now');
            obj.timestamp_str = sprintf('%04d%02d%02d_%02d%02d%02d', ...
                year(now_dt), month(now_dt), day(now_dt), ...
                hour(now_dt), minute(now_dt), floor(second(now_dt)));
            
            % Create run-specific directory
            run_dir = fullfile(obj.test_dir, obj.timestamp_str);
            if ~isfolder(run_dir)
                try
                    mkdir(run_dir);
                    if obj.verbose
                        fprintf('[DataLogger] Created directory: %s\n', run_dir);
                    end
                catch ME
                    warning('[DataLogger] Failed to create directory: %s\n', ME.message);
                end
            end
            
            % Set file paths
            obj.traj_file = fullfile(run_dir, 'trajectory.csv');
            obj.wind_file = fullfile(run_dir, 'wind.csv');
            
            % Initialize trajectory CSV with header
            try
                obj.traj_fid = fopen(obj.traj_file, 'w');
                if obj.traj_fid > 0
                    fprintf(obj.traj_fid, 'time_s,x_m,y_m,z_m\n');
                    fclose(obj.traj_fid);
                    obj.traj_fid = -1;
                    if obj.verbose
                        fprintf('[DataLogger] Trajectory file ready: %s\n', obj.traj_file);
                    end
                end
            catch ME
                warning('[DataLogger] Failed to init trajectory file: %s\n', ME.message);
            end
            
            % Initialize wind CSV with header
            try
                obj.wind_fid = fopen(obj.wind_file, 'w');
                if obj.wind_fid > 0
                    fprintf(obj.wind_fid, 'time_s,wind_x,wind_y,wind_z,magnitude,source\n');
                    fclose(obj.wind_fid);
                    obj.wind_fid = -1;
                    if obj.verbose
                        fprintf('[DataLogger] Wind vector file ready: %s\n', obj.wind_file);
                    end
                end
            catch ME
                warning('[DataLogger] Failed to init wind file: %s\n', ME.message);
            end
        end

        function log_trajectory(obj, time_s, x, y, z)
            % Append trajectory point to CSV
            try
                obj.traj_fid = fopen(obj.traj_file, 'a');
                if obj.traj_fid > 0
                    fprintf(obj.traj_fid, '%.3f,%.6f,%.6f,%.6f\n', time_s, x, y, z);
                    fclose(obj.traj_fid);
                    obj.traj_fid = -1;
                end
            catch ME
                if obj.verbose
                    fprintf('[DataLogger] Failed to log trajectory: %s\n', ME.message);
                end
            end
        end

        function log_wind(obj, time_s, wind_x, wind_y, wind_z, source)
            % Append wind vector to CSV
            if nargin < 6 || isempty(source)
                source = 'wind_sensor';
            end
            try
                magnitude = norm([wind_x, wind_y, wind_z]);
                obj.wind_fid = fopen(obj.wind_file, 'a');
                if obj.wind_fid > 0
                    fprintf(obj.wind_fid, '%.3f,%.6f,%.6f,%.6f,%.6f,%s\n', time_s, wind_x, wind_y, wind_z, magnitude, source);
                    fclose(obj.wind_fid);
                    obj.wind_fid = -1;
                end
            catch ME
                if obj.verbose
                    fprintf('[DataLogger] Failed to log wind vector: %s\n', ME.message);
                end
            end
        end

        function subscribe_to_topics(obj)
            % Subscribe to odometry, virtual wind sensor, and airspeed topics
            try
                obj.ros_node = ros2node('/matlab_data_logger');
                
                odom_topic = sprintf('%s/odometry', obj.namespace);
                try
                    obj.sub_odom = ros2subscriber(obj.ros_node, odom_topic, 'nav_msgs/Odometry', @obj.callback_odom);
                    if obj.verbose
                        fprintf('[DataLogger] ✓ Subscribed to ODOM: %s\n', odom_topic);
                    end
                catch ME
                    if obj.verbose
                        fprintf('[DataLogger] ✗ ODOM subscribe failed: %s\n', ME.message);
                    end
                    obj.sub_odom = [];
                end
                
                if obj.verbose
                    fprintf('[DataLogger] Using virtual wind sensor as primary wind source\n');
                end

                wind_topic = sprintf('%s/wind_sensor', obj.namespace);
                try
                    obj.sub_wind = ros2subscriber(obj.ros_node, wind_topic, 'geometry_msgs/Vector3Stamped', @obj.callback_wind, ...
                        'Reliability', 'besteffort', 'Durability', 'volatile');
                    if obj.verbose
                        fprintf('[DataLogger] ✓ Subscribed to WIND SENSOR: %s\n', wind_topic);
                    end
                catch ME
                    if obj.verbose
                        fprintf('[DataLogger] ⚠ WIND SENSOR subscribe failed: %s\n', ME.message);
                    end
                    obj.sub_wind = [];
                end

                airspeed_topic = sprintf('%s/air_speed', obj.namespace);
                try
                    obj.sub_airspeed = ros2subscriber(obj.ros_node, airspeed_topic, 'geometry_msgs/Vector3Stamped', @obj.callback_airspeed, ...
                        'Reliability', 'besteffort', 'Durability', 'volatile');
                    if obj.verbose
                        fprintf('[DataLogger] ✓ Subscribed to AIRSPEED: %s\n', airspeed_topic);
                    end
                catch
                    try
                        obj.sub_airspeed = ros2subscriber(obj.ros_node, airspeed_topic, 'ardupilot_msgs/Airspeed', @obj.callback_airspeed, ...
                            'Reliability', 'besteffort', 'Durability', 'volatile');
                        if obj.verbose
                            fprintf('[DataLogger] ✓ Subscribed to AIRSPEED: %s\n', airspeed_topic);
                        end
                    catch ME
                        if obj.verbose
                            fprintf('[DataLogger] ✗ AIRSPEED subscribe failed: %s\n', ME.message);
                        end
                        obj.sub_airspeed = [];
                    end
                end
            catch ME
                if obj.verbose
                    fprintf('[DataLogger] Subscription failed: %s\n', ME.message);
                end
            end
        end

        function start_background(obj, interval_s)
            % Start background timer for continuous data collection
            if nargin < 2, interval_s = 0.5; end
            try
                obj.subscribe_to_topics();
            catch
            end
            obj.timer_obj = timer('ExecutionMode','fixedSpacing','BusyMode','drop','Period',interval_s,...
                'TimerFcn',@(~,~)obj.update_once());
            start(obj.timer_obj);
            if obj.verbose
                fprintf('[DataLogger] Background timer started (%.2fs interval)\n', interval_s);
            end
        end

        function update_once(obj)
            % Periodically collect and log data
            try
                elapsed_s = toc(obj.start_time);
                
                % Log the latest samples collected by subscriber callbacks.
                if ~isempty(obj.latest_odom_row) && isfinite(obj.latest_odom_stamp) && obj.latest_odom_stamp ~= obj.last_logged_odom_stamp
                    try
                        obj.log_trajectory(elapsed_s, obj.latest_odom_row(1), obj.latest_odom_row(2), obj.latest_odom_row(3));
                        obj.last_logged_odom_stamp = obj.latest_odom_stamp;
                    catch
                    end
                end
                
                if ~isempty(obj.latest_wind_row) && isfinite(obj.latest_wind_stamp) && obj.latest_wind_stamp ~= obj.last_logged_wind_stamp
                    try
                        obj.log_wind(elapsed_s, obj.latest_wind_row(1), obj.latest_wind_row(2), obj.latest_wind_row(3), 'virtual_wind_sensor');
                        obj.last_logged_wind_stamp = obj.latest_wind_stamp;
                    catch
                    end
                elseif ~isempty(obj.latest_airspeed_row) && isfinite(obj.latest_airspeed_stamp) && obj.latest_airspeed_stamp ~= obj.last_logged_airspeed_stamp
                    try
                        obj.log_wind(elapsed_s, obj.latest_airspeed_row(1), obj.latest_airspeed_row(2), obj.latest_airspeed_row(3), 'airspeed_fallback');
                        obj.last_logged_airspeed_stamp = obj.latest_airspeed_stamp;
                    catch
                    end
                end
            catch
            end
        end

        function stop_background(obj)
            % Stop the background timer
            try
                if ~isempty(obj.timer_obj) && isvalid(obj.timer_obj)
                    stop(obj.timer_obj);
                    delete(obj.timer_obj);
                    obj.timer_obj = [];
                end
            catch
            end
            try
                if ~isempty(obj.sub_odom), clear obj.sub_odom; end
                if ~isempty(obj.sub_wind), clear obj.sub_wind; end
                if ~isempty(obj.sub_airspeed), clear obj.sub_airspeed; end
                if ~isempty(obj.ros_node), clear obj.ros_node; end
            catch
            end
            if obj.verbose
                fprintf('[DataLogger] Background timer stopped\n');
            end
        end

        function cleanup(obj)
            % Stop background collection and close files
            try
                obj.stop_background();
            catch
            end
            
            try
                if ~isempty(obj.traj_fid) && obj.traj_fid > 0
                    fclose(obj.traj_fid);
                    obj.traj_fid = -1;
                end
                if ~isempty(obj.wind_fid) && obj.wind_fid > 0
                    fclose(obj.wind_fid);
                    obj.wind_fid = -1;
                end
                
                % Print summary
                if obj.verbose
                    fprintf('[DataLogger] Data saved to: %s\n', fileparts(obj.traj_file));
                    if isfile(obj.traj_file)
                        try
                            traj_data = readtable(obj.traj_file);
                            fprintf('[DataLogger] Trajectory points: %d\n', height(traj_data));
                        catch
                            fprintf('[DataLogger] Trajectory file created\n');
                        end
                    end
                    if isfile(obj.wind_file)
                        try
                            wind_data = readtable(obj.wind_file);
                            fprintf('[DataLogger] Wind vector samples: %d\n', height(wind_data));
                        catch
                            fprintf('[DataLogger] Wind vector file created\n');
                        end
                    end
                end
            catch ME
                warning('[DataLogger] Cleanup error: %s\n', ME.message);
            end
        end

        function callback_airspeed(obj, msg)
            try
                [wind_x, wind_y, wind_z] = obj.extract_airspeed_vector(msg);
                obj.latest_airspeed_row = [wind_x, wind_y, wind_z];
                obj.latest_airspeed_stamp = now;
            catch
            end
        end

        function callback_wind(obj, msg)
            try
                [wind_x, wind_y, wind_z] = obj.extract_vector3(msg);
                obj.latest_wind_row = [wind_x, wind_y, wind_z];
                obj.latest_wind_stamp = now;
            catch
            end
        end

        function callback_odom(obj, msg)
            try
                obj.latest_odom_row = [ ...
                    double(msg.pose.pose.position.x), ...
                    double(msg.pose.pose.position.y), ...
                    double(msg.pose.pose.position.z)];
                obj.latest_odom_stamp = now;
            catch
            end
        end

        function [wind_x, wind_y, wind_z] = extract_airspeed_vector(obj, msg)
            wind_x = 0;
            wind_y = 0;
            wind_z = 0;
            try
                if isfield(msg, 'vector')
                    [wind_x, wind_y, wind_z] = obj.extract_vector3(msg);
                elseif isfield(msg, 'true_airspeed')
                    wind_x = double(msg.true_airspeed.x);
                    wind_y = double(msg.true_airspeed.y);
                    wind_z = double(msg.true_airspeed.z);
                end
            catch
            end
        end

        function [x, y, z] = extract_vector3(~, msg)
            x = 0;
            y = 0;
            z = 0;
            try
                if isfield(msg, 'vector') || isprop(msg, 'vector')
                    x = double(msg.vector.x);
                    y = double(msg.vector.y);
                    z = double(msg.vector.z);
                end
            catch
            end
        end
    end
end
