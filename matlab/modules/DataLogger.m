%% DataLogger - Save trajectory and wind data to timestamped CSV files
% Saves odometry and wind data to matlab/data/test/<timestamp>/ folder

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
        sub_wind
        timer_obj
        namespace = '/iris'
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
            obj.data_dir = fullfile(pwd, p.Results.base_dir);
            obj.test_dir = fullfile(obj.data_dir, 'test');
            
            % Create timestamp string (YYYYMMDD_HHMMSS)
            now_dt = datetime('now');
            obj.timestamp_str = sprintf('%04d%02d%02d_%02d%02d%02d', ...
                year(now_dt), month(now_dt), day(now_dt), ...
                hour(now_dt), minute(now_dt), second(now_dt));
            
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
                    fprintf(obj.wind_fid, 'time_s,wind_x,wind_y,wind_z,magnitude\n');
                    fclose(obj.wind_fid);
                    if obj.verbose
                        fprintf('[DataLogger] Wind file ready: %s\n', obj.wind_file);
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
                end
            catch ME
                if obj.verbose
                    fprintf('[DataLogger] Failed to log trajectory: %s\n', ME.message);
                end
            end
        end

        function log_wind(obj, time_s, wind_x, wind_y, wind_z)
            % Append wind measurement to CSV
            try
                magnitude = norm([wind_x, wind_y, wind_z]);
                obj.wind_fid = fopen(obj.wind_file, 'a');
                if obj.wind_fid > 0
                    fprintf(obj.wind_fid, '%.3f,%.6f,%.6f,%.6f,%.6f\n', time_s, wind_x, wind_y, wind_z, magnitude);
                    fclose(obj.wind_fid);
                end
            catch ME
                if obj.verbose
                    fprintf('[DataLogger] Failed to log wind: %s\n', ME.message);
                end
            end
        end

        function subscribe_to_topics(obj)
            % Subscribe to odometry and wind topics
            try
                obj.ros_node = ros2node('/matlab_data_logger');
                
                odom_topic = sprintf('%s/odometry', obj.namespace);
                try
                    obj.sub_odom = ros2subscriber(obj.ros_node, odom_topic, 'nav_msgs/Odometry');
                    if obj.verbose
                        fprintf('[DataLogger] ✓ Subscribed to ODOM: %s\n', odom_topic);
                    end
                catch ME
                    if obj.verbose
                        fprintf('[DataLogger] ✗ ODOM subscribe failed: %s\n', ME.message);
                    end
                    obj.sub_odom = [];
                end
                
                wind_topic = sprintf('%s/wind_command_az', obj.namespace);
                try
                    obj.sub_wind = ros2subscriber(obj.ros_node, wind_topic, 'ros_gz_interfaces/Float32Array');
                    if obj.verbose
                        fprintf('[DataLogger] ✓ Subscribed to WIND: %s\n', wind_topic);
                    end
                catch ME
                    if obj.verbose
                        fprintf('[DataLogger] ✗ WIND subscribe failed: %s\n', ME.message);
                    end
                    obj.sub_wind = [];
                    % Try alternative wind topics
                    alt_topics = {
                        sprintf('%s/wind', obj.namespace)
                        sprintf('%s/wind_command', obj.namespace)
                        '/wind_command_az'
                        '/iris/wind_command'
                    };
                    for i = 1:length(alt_topics)
                        try
                            obj.sub_wind = ros2subscriber(obj.ros_node, alt_topics{i}, 'ros_gz_interfaces/Float32Array');
                            if obj.verbose
                                fprintf('[DataLogger] ✓ Alternative wind topic found: %s\n', alt_topics{i});
                            end
                            break;
                        catch
                        end
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
            obj.timer_obj = timer('ExecutionMode','fixedSpacing','Period',interval_s,...
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
                
                % Collect odometry
                if ~isempty(obj.sub_odom)
                    try
                        [odom, ok] = receive(obj.sub_odom, 0.05);
                        if ok && ~isempty(odom)
                            x = odom.pose.pose.position.x;
                            y = odom.pose.pose.position.y;
                            z = odom.pose.pose.position.z;
                            obj.log_trajectory(elapsed_s, x, y, z);
                        end
                    catch
                    end
                end
                
                % Collect wind
                if ~isempty(obj.sub_wind)
                    try
                        [wmsg, wok] = receive(obj.sub_wind, 0.05);
                        if wok && ~isempty(wmsg) && isfield(wmsg, 'data')
                            try
                                arr = double(wmsg.data);
                                if length(arr) >= 3
                                    wind_x = arr(1); wind_y = arr(2); wind_z = arr(3);
                                else
                                    wind_x = arr(1); wind_y = 0; wind_z = 0;
                                end
                                obj.log_wind(elapsed_s, wind_x, wind_y, wind_z);
                            catch
                            end
                        end
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
                end
                if ~isempty(obj.wind_fid) && obj.wind_fid > 0
                    fclose(obj.wind_fid);
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
                            fprintf('[DataLogger] Wind samples: %d\n', height(wind_data));
                        catch
                            fprintf('[DataLogger] Wind file created\n');
                        end
                    end
                end
            catch ME
                warning('[DataLogger] Cleanup error: %s\n', ME.message);
            end
        end
    end
end
