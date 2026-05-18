%% RealtimeVisualizer - Live plotting of odometry and wind commands
% Subscribes to odometry and wind topics and plots position + wind magnitude in real-time

classdef RealtimeVisualizer < handle
    properties
        namespace = '/iris'
        ros_node
        sub_odom
        sub_wind
        fig
        h_traj
        h_wind
        traj_x = []
        traj_y = []
        traj_z = []
        wind_history = []
        wind_time = []
        timer_obj
        verbose = true
    end

    methods
        function obj = RealtimeVisualizer(varargin)
            p = inputParser;
            addParameter(p, 'namespace', '/iris', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});

            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
        end

        function init(obj)
            try
                obj.ros_node = ros2node('/matlab_realtime_vis');
                odom_topic = sprintf('%s/odometry', obj.namespace);
                try
                    obj.sub_odom = ros2subscriber(obj.ros_node, odom_topic, 'nav_msgs/Odometry');
                    if obj.verbose, fprintf('[RealtimeVisualizer] Subscribed: %s\n', odom_topic); end
                catch ME
                    if obj.verbose, fprintf('[RealtimeVisualizer] ODOM subscribe failed: %s\n', ME.message); end
                    obj.sub_odom = [];
                end

                wind_topic = sprintf('%s/wind_command_az', obj.namespace);
                try
                    obj.sub_wind = ros2subscriber(obj.ros_node, wind_topic, 'ros_gz_interfaces/Float32Array');
                    if obj.verbose, fprintf('[RealtimeVisualizer] Subscribed: %s\n', wind_topic); end
                catch ME
                    if obj.verbose, fprintf('[RealtimeVisualizer] WIND subscribe failed: %s\n', ME.message); end
                    obj.sub_wind = [];
                end

                % Setup figure
                obj.fig = figure('Name', sprintf('Realtime Visualizer (%s)', obj.namespace), 'NumberTitle', 'off', 'Color', 'w');
                tiledlayout(1,2);
                ax1 = nexttile;
                obj.h_traj = plot3(ax1, NaN, NaN, NaN, '-b'); hold(ax1,'on'); grid(ax1,'on'); view(ax1,3);
                xlabel(ax1, 'X (m)'); ylabel(ax1, 'Y (m)'); zlabel(ax1, 'Z (m)'); title(ax1, 'Vehicle trajectory');
                ax2 = nexttile;
                obj.h_wind = plot(ax2, NaN, NaN, '-r'); grid(ax2,'on'); xlabel(ax2,'Time (s)'); ylabel(ax2,'Wind mag'); title(ax2,'Wind magnitude');

            catch ME
                warning('[RealtimeVisualizer] init failed: %s', ME.message);
            end
        end

        function start_background(obj, interval_s)
            if nargin < 2, interval_s = 0.5; end
            if isempty(obj.ros_node) || (~isempty(obj.sub_odom) && isempty(obj.sub_wind) && isempty(obj.sub_odom))
                try
                    obj.init();
                catch
                end
            end
            obj.timer_obj = timer('ExecutionMode','fixedSpacing','Period',interval_s,'TimerFcn',@(~,~)obj.update_once());
            start(obj.timer_obj);
            if obj.verbose, fprintf('[RealtimeVisualizer] Background timer started (%.2fs)\n',interval_s); end
        end

        function update_once(obj)
            try
                t_now = tic;
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
                        end
                    catch
                    end
                end

                % WIND
                if ~isempty(obj.sub_wind)
                    try
                        [wmsg, wok] = receive(obj.sub_wind, 0.05);
                        if wok && ~isempty(wmsg) && isfield(wmsg,'data')
                            mag = 0;
                            try
                                arr = double(wmsg.data);
                                mag = norm(arr(:)');
                            catch
                                mag = double(wmsg.data(1));
                            end
                            obj.wind_history(end+1) = mag; %#ok<AGROW>
                            if isempty(obj.wind_time)
                                obj.wind_time(end+1) = 0; %#ok<AGROW>
                            else
                                obj.wind_time(end+1) = obj.wind_time(end) + toc(t_now); %#ok<AGROW>
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
            try
                if ~isempty(obj.timer_obj) && isvalid(obj.timer_obj)
                    stop(obj.timer_obj); delete(obj.timer_obj); obj.timer_obj = [];
                end
            catch
            end
            try
                if ~isempty(obj.sub_odom), clear obj.sub_odom; end
                if ~isempty(obj.sub_wind), clear obj.sub_wind; end
                if ~isempty(obj.ros_node), clear obj.ros_node; end
            catch
            end
            try
                if ~isempty(obj.fig) && isvalid(obj.fig), close(obj.fig); end
            catch
            end
            if obj.verbose, fprintf('[RealtimeVisualizer] Stopped and cleaned up\n'); end
        end
    end
end
