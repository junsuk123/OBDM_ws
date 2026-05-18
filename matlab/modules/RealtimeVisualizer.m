%% RealtimeVisualizer - Live plotting with data logging and wind diagnostics
% Subscribes to odometry and wind topics, plots in real-time, and logs data to CSV

classdef RealtimeVisualizer < handle
    properties
        namespace = '/iris'
        ros_node
        sub_odom
        sub_wind
        fig
        h_traj
        h_wind
        h_status
        traj_x = []
        traj_y = []
        traj_z = []
        traj_time = []
        wind_history = []
        wind_time = []
        wind_msg_count = 0
        odom_msg_count = 0
        timer_obj
        verbose = true
        start_tic
    end

    methods
        function obj = RealtimeVisualizer(varargin)
            p = inputParser;
            addParameter(p, 'namespace', '/iris', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});

            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
            obj.start_tic = tic;
        end

        function init(obj)
            try
                obj.ros_node = ros2node('/matlab_realtime_vis');
                if obj.verbose
                    fprintf('[RealtimeVisualizer] ROS2 node created: /matlab_realtime_vis\n');
                end
                
                % Subscribe to odometry
                odom_topic = sprintf('%s/odometry', obj.namespace);
                try
                    obj.sub_odom = ros2subscriber(obj.ros_node, odom_topic, 'nav_msgs/Odometry');
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Subscribed to ODOM: %s\n', odom_topic); end
                catch ME
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✗ ODOM subscribe failed: %s\n', ME.message); end
                    obj.sub_odom = [];
                end

                % Diagnose wind topic availability
                if obj.verbose
                    fprintf('[RealtimeVisualizer] Diagnosing wind topics...\n');
                    obj.diagnose_wind_topics();
                end
                
                % Subscribe to wind
                wind_topic = sprintf('%s/wind_command_az', obj.namespace);
                try
                    obj.sub_wind = ros2subscriber(obj.ros_node, wind_topic, 'ros_gz_interfaces/Float32Array');
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Subscribed to WIND: %s\n', wind_topic); end
                catch ME
                    if obj.verbose
                        fprintf('[RealtimeVisualizer] ✗ WIND subscribe failed: %s\n', ME.message);
                        fprintf('[RealtimeVisualizer] Trying alternative wind topics...\n');
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
                            if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Alternative wind topic found: %s\n', alt_topics{i}); end
                            break;
                        catch
                        end
                    end
                end

                % Setup figure with 3 subplots
                obj.fig = figure('Name', sprintf('Realtime Visualizer (%s)', obj.namespace), ...
                    'NumberTitle', 'off', 'Color', 'w', 'Position', [100 100 1200 600]);
                tiledlayout(1,3);
                
                % Trajectory plot
                ax1 = nexttile;
                obj.h_traj = plot3(ax1, NaN, NaN, NaN, '-b', 'LineWidth', 1.5);
                hold(ax1,'on'); grid(ax1,'on'); view(ax1,3);
                xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)');
                title(ax1, 'Vehicle trajectory');
                
                % Wind magnitude plot
                ax2 = nexttile;
                obj.h_wind = plot(ax2, NaN, NaN, '-r', 'LineWidth', 1.5);
                grid(ax2,'on'); xlabel(ax2,'Time (s)'); ylabel(ax2,'Wind magnitude (m/s)');
                title(ax2,'Wind magnitude over time');
                
                % Status plot (text display)
                ax3 = nexttile;
                axis(ax3, 'off');
                obj.h_status = text(ax3, 0.1, 0.9, '', 'VerticalAlignment', 'top', ...
                    'FontFamily', 'monospace', 'FontSize', 10);
                title(ax3, 'Status & Diagnostics');

            catch ME
                warning('[RealtimeVisualizer] init failed: %s', ME.message);
            end
        end

        function diagnose_wind_topics(obj)
            % Check available topics via system command
            try
                [status, output] = system('ros2 topic list 2>/dev/null | grep -i wind || echo "no wind topics"');
                if status == 0
                    lines = strsplit(strtrim(output), newline);
                    lines(cellfun(@isempty, lines)) = [];
                    if ~isempty(lines)
                        fprintf('[RealtimeVisualizer] Available wind topics:\n');
                        for i = 1:length(lines)
                            fprintf('  - %s\n', lines{i});
                        end
                    else
                        fprintf('[RealtimeVisualizer] ⚠ No wind topics found in ROS2\n');
                    end
                end
            catch
            end
        end

        function start_background(obj, interval_s)
            if nargin < 2, interval_s = 0.5; end
            try
                obj.init();
            catch
            end
            obj.timer_obj = timer('ExecutionMode','fixedSpacing','Period',interval_s,...
                'TimerFcn',@(~,~)obj.update_once(),'ErrorFcn',@(~,~)fprintf('[RealtimeVisualizer] Timer error\n'));
            start(obj.timer_obj);
            if obj.verbose
                fprintf('[RealtimeVisualizer] Background timer started (%.2fs interval)\n',interval_s);
            end
        end

        function update_once(obj)
            try
                elapsed_s = toc(obj.start_tic);
                
                % ODOM
                if ~isempty(obj.sub_odom)
                    try
                        [odom, ok] = receive(obj.sub_odom, 0.05);
                        if ok && ~isempty(odom)
                            x = odom.pose.pose.position.x;
                            y = odom.pose.pose.position.y;
                            z = odom.pose.pose.position.z;
                            obj.traj_x(end+1) = x; %#ok<AGROW>
                            obj.traj_y(end+1) = y; %#ok<AGROW>
                            obj.traj_z(end+1) = z; %#ok<AGROW>
                            obj.traj_time(end+1) = elapsed_s; %#ok<AGROW>
                            obj.odom_msg_count = obj.odom_msg_count + 1;
                        end
                    catch
                    end
                end

                % WIND
                if ~isempty(obj.sub_wind)
                    try
                        [wmsg, wok] = receive(obj.sub_wind, 0.05);
                        if wok && ~isempty(wmsg) && isfield(wmsg,'data')
                            try
                                arr = double(wmsg.data);
                                if length(arr) >= 3
                                    wind_x = arr(1); wind_y = arr(2); wind_z = arr(3);
                                else
                                    wind_x = arr(1); wind_y = 0; wind_z = 0;
                                end
                                mag = norm([wind_x, wind_y, wind_z]);
                                obj.wind_history(end+1) = mag; %#ok<AGROW>
                                obj.wind_time(end+1) = elapsed_s; %#ok<AGROW>
                                obj.wind_msg_count = obj.wind_msg_count + 1;
                            catch
                            end
                        end
                    catch
                    end
                end

                % Update plots if figure still valid
                if ~isempty(obj.fig) && isvalid(obj.fig)
                    try
                        if ~isempty(obj.traj_x)
                            set(obj.h_traj, 'XData', obj.traj_x, 'YData', obj.traj_y, 'ZData', obj.traj_z);
                        end
                        if ~isempty(obj.wind_history)
                            set(obj.h_wind, 'XData', obj.wind_time, 'YData', obj.wind_history);
                        end
                        
                        % Update status text
                        status_str = sprintf(...
                            'Elapsed: %.1f s\nOdom msgs: %d\nWind msgs: %d\nTraj points: %d\nWind samples: %d\n\nStatus:\n', ...
                            elapsed_s, obj.odom_msg_count, obj.wind_msg_count, ...
                            length(obj.traj_x), length(obj.wind_history));
                        
                        if isempty(obj.sub_odom)
                            status_str = sprintf('%s✗ ODOM not subscribed\n', status_str);
                        else
                            status_str = sprintf('%s✓ ODOM subscribed\n', status_str);
                        end
                        
                        if isempty(obj.sub_wind)
                            status_str = sprintf('%s✗ WIND not subscribed\n', status_str);
                        else
                            if obj.wind_msg_count > 0
                                status_str = sprintf('%s✓ WIND subscribed & receiving\n', status_str);
                            else
                                status_str = sprintf('%s⚠ WIND subscribed but no data yet\n', status_str);
                            end
                        end
                        
                        set(obj.h_status, 'String', status_str);
                        
                        drawnow limitrate nocallbacks;
                    catch
                    end
                else
                    % figure closed - stop timer
                    obj.stop();
                end
            catch
            end
        end

        function stop(obj)
            % Stop timer
            try
                if ~isempty(obj.timer_obj) && isvalid(obj.timer_obj)
                    stop(obj.timer_obj); delete(obj.timer_obj); obj.timer_obj = [];
                end
            catch
            end
            
            % Clean up subscriptions and ROS node
            try
                if ~isempty(obj.sub_odom), clear obj.sub_odom; end
                if ~isempty(obj.sub_wind), clear obj.sub_wind; end
                if ~isempty(obj.ros_node), clear obj.ros_node; end
            catch
            end
            
            % Close figure
            try
                if ~isempty(obj.fig) && isvalid(obj.fig), close(obj.fig); end
            catch
            end
            
            if obj.verbose
                fprintf('[RealtimeVisualizer] Stopped and cleaned up\n');
                fprintf('[RealtimeVisualizer] Final stats: ODOM=%d msgs, WIND=%d msgs\n', ...
                    obj.odom_msg_count, obj.wind_msg_count);
            end
        end
    end
end
