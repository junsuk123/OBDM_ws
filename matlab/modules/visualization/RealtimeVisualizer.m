%% RealtimeVisualizer - Live plotting with virtual wind sensor overlay
% Subscribes to odometry and virtual wind sensor topics and plots in real-time

classdef RealtimeVisualizer < handle
    properties
        namespace = '/iris'
        ros_node
        sub_odom
        sub_wind
        sub_airspeed
        fig
        ax_traj
        ax_status
        h_traj
        h_air_scatter
        h_air_quiver
        h_status
        traj_x = []
        traj_y = []
        traj_z = []
        traj_time = []
        traj_wind_mag = []
        traj_wind_vec = []
        wind_history = []
        wind_time = []
        latest_wind_vec = [NaN, NaN, NaN]
        latest_wind_mag = NaN
        wind_msg_count = 0
        odom_msg_count = 0
        timer_obj
        verbose = true
        start_tic
        is_drawing = false
        max_history_points = 3000
        max_render_points = 700
        show_traj_line = false
    end

    methods
        function obj = RealtimeVisualizer(varargin)
            p = inputParser;
            addParameter(p, 'namespace', '/iris', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            addParameter(p, 'max_history_points', 3000, @(x) isnumeric(x) && isscalar(x) && x > 100);
            addParameter(p, 'max_render_points', 700, @(x) isnumeric(x) && isscalar(x) && x > 50);
            addParameter(p, 'show_traj_line', false, @islogical);
            parse(p, varargin{:});

            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
            obj.max_history_points = double(p.Results.max_history_points);
            obj.max_render_points = double(p.Results.max_render_points);
            obj.show_traj_line = p.Results.show_traj_line;
            obj.start_tic = tic;
        end

        function init(obj)
            try
                obj.ros_node = ros2node('/matlab_realtime_vis');
                if obj.verbose
                    fprintf('[RealtimeVisualizer] ROS2 node created: /matlab_realtime_vis\n');
                end
                
                % Subscribe to odometry
                odom_topic = SimulationConfig.resolve_topic('odometry', obj.namespace);
                try
                    obj.sub_odom = ros2subscriber(obj.ros_node, odom_topic, 'nav_msgs/Odometry', ...
                        'Reliability', 'besteffort', 'Durability', 'volatile');
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Subscribed to ODOM: %s\n', odom_topic); end
                catch ME
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✗ ODOM subscribe failed: %s\n', ME.message); end
                    obj.sub_odom = [];
                end

                if obj.verbose
                    fprintf('[RealtimeVisualizer] Using virtual wind sensor for wind display\n');
                end
                wind_topic = sprintf('%s/wind_sensor', obj.namespace);
                try
                    obj.sub_wind = ros2subscriber(obj.ros_node, wind_topic, 'geometry_msgs/Vector3Stamped', @obj.callback_wind, ...
                        'Reliability', 'besteffort', 'Durability', 'volatile');
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Subscribed to WIND SENSOR: %s\n', wind_topic); end
                catch ME
                    if obj.verbose, fprintf('[RealtimeVisualizer] ⚠ WIND SENSOR subscribe failed: %s\n', ME.message); end
                    obj.sub_wind = [];
                end

                airspeed_topic = sprintf('%s/air_speed', obj.namespace);
                try
                    obj.sub_airspeed = ros2subscriber(obj.ros_node, airspeed_topic, 'geometry_msgs/Vector3Stamped', @obj.callback_airspeed, ...
                        'Reliability', 'besteffort', 'Durability', 'volatile');
                    if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Subscribed to AIRSPEED: %s\n', airspeed_topic); end
                catch
                    try
                        obj.sub_airspeed = ros2subscriber(obj.ros_node, airspeed_topic, 'ardupilot_msgs/Airspeed', @obj.callback_airspeed, ...
                            'Reliability', 'besteffort', 'Durability', 'volatile');
                        if obj.verbose, fprintf('[RealtimeVisualizer] ✓ Subscribed to AIRSPEED: %s\n', airspeed_topic); end
                    catch ME
                        if obj.verbose, fprintf('[RealtimeVisualizer] ✗ AIRSPEED subscribe failed: %s\n', ME.message); end
                        obj.sub_airspeed = [];
                    end
                end

                % Setup figure with trajectory plus status. Virtual wind is
                % overlaid on the trajectory as color and direction arrows.
                obj.fig = figure('Name', sprintf('Realtime Visualizer (%s)', obj.namespace), ...
                    'NumberTitle', 'off', 'Color', 'w', 'Position', [100 100 1200 600]);
                tiledlayout(1,2);
                
                % Trajectory plot
                obj.ax_traj = nexttile;
                obj.h_traj = plot3(obj.ax_traj, NaN, NaN, NaN, '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2);
                hold(obj.ax_traj,'on'); grid(obj.ax_traj,'on'); view(obj.ax_traj,3);
                if ~obj.show_traj_line
                    set(obj.h_traj, 'Visible', 'off');
                end
                obj.h_air_scatter = scatter3(obj.ax_traj, NaN, NaN, NaN, 28, NaN, 'filled');
                obj.h_air_quiver = quiver3(obj.ax_traj, NaN, NaN, NaN, NaN, NaN, NaN, 0, ...
                    'Color', [0.05 0.2 0.45], 'LineWidth', 1.0, 'MaxHeadSize', 0.8);
                xlabel(obj.ax_traj, 'X (m)'); ylabel(obj.ax_traj, 'Y (m)'); zlabel(obj.ax_traj, 'Z (m)');
                title(obj.ax_traj, 'Trajectory colored by relative wind');
                colormap(obj.ax_traj, turbo);
                cb = colorbar(obj.ax_traj);
                cb.Label.String = 'Relative wind magnitude (m/s)';
                
                % Status plot (text display)
                obj.ax_status = nexttile;
                axis(obj.ax_status, 'off');
                obj.h_status = text(obj.ax_status, 0.1, 0.9, '', 'VerticalAlignment', 'top', ...
                    'FontName', 'monospace', 'FontSize', 10);
                title(obj.ax_status, 'Status & Diagnostics');

            catch ME
                warning('[RealtimeVisualizer] init failed: %s', ME.message);
            end
        end

        function diagnose_wind_topics(obj)
            % Backward-compatible diagnostic: report airspeed topics.
            try
                [status, output] = system('ros2 topic list 2>/dev/null | grep -Ei "air_speed|airspeed" || echo "no airspeed topics"');
                if status == 0
                    lines = strsplit(strtrim(output), newline);
                    lines(cellfun(@isempty, lines)) = [];
                    if ~isempty(lines)
                        fprintf('[RealtimeVisualizer] Available airspeed topics:\n');
                        for i = 1:length(lines)
                            fprintf('  - %s\n', lines{i});
                        end
                    else
                        fprintf('[RealtimeVisualizer] ⚠ No airspeed topics found in ROS2\n');
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
            obj.timer_obj = timer('ExecutionMode','fixedSpacing','BusyMode','drop','Period',interval_s,...
                'TimerFcn',@(~,~)obj.update_once(),'ErrorFcn',@(~,~)fprintf('[RealtimeVisualizer] Timer error\n'));
            start(obj.timer_obj);
            if obj.verbose
                fprintf('[RealtimeVisualizer] Background timer started (%.2fs interval)\n',interval_s);
            end
        end

        function update_once(obj)
            try
                if obj.is_drawing
                    return;
                end
                obj.is_drawing = true;
                cleanup_draw = onCleanup(@() obj.set_drawing(false)); %#ok<NASGU>
                elapsed_s = toc(obj.start_tic);
                
                obj.poll_odom();

                % Update plots if figure still valid
                if ~isempty(obj.fig) && isvalid(obj.fig)
                    try
                        [x, y, z, speeds, vecs] = obj.snapshot_plot_data();
                        if ~isempty(x)
                            set(obj.h_traj, 'XData', x, 'YData', y, 'ZData', z);
                            obj.update_wind_overlay(x, y, z, speeds, vecs);
                        end
                        
                        % Update status text
                        status_str = sprintf(...
                            'Elapsed: %.1f s\nOdom msgs: %d\nWind msgs: %d\nTraj points: %d\nWind samples: %d\nLatest wind: [%.2f %.2f %.2f] m/s\nWind mag: %.3f m/s\n\nStatus:\n', ...
                            elapsed_s, obj.odom_msg_count, obj.wind_msg_count, ...
                            numel(x), length(obj.wind_history), ...
                            obj.latest_wind_vec(1), obj.latest_wind_vec(2), obj.latest_wind_vec(3), obj.latest_wind_mag);
                        
                        if isempty(obj.sub_odom)
                            status_str = sprintf('%s✗ ODOM not subscribed\n', status_str);
                        else
                            status_str = sprintf('%s✓ ODOM subscribed\n', status_str);
                        end
                        
                        if ~isempty(obj.sub_wind) && obj.wind_msg_count > 0
                            status_str = sprintf('%s✓ Virtual wind overlay active\n', status_str);
                        elseif isempty(obj.sub_wind) && isempty(obj.sub_airspeed)
                            status_str = sprintf('%s✗ WIND/AIRSPEED not subscribed\n', status_str);
                        elseif obj.wind_msg_count > 0
                            status_str = sprintf('%s✓ Airspeed fallback overlay active\n', status_str);
                        else
                            status_str = sprintf('%s⚠ Wind source subscribed but no data yet\n', status_str);
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

        function set_drawing(obj, value)
            obj.is_drawing = value;
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
                if ~isempty(obj.sub_airspeed), clear obj.sub_airspeed; end
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

        function callback_odom(obj, msg)
            try
                obj.traj_x(end+1) = double(msg.pose.pose.position.x); %#ok<AGROW>
                obj.traj_y(end+1) = double(msg.pose.pose.position.y); %#ok<AGROW>
                obj.traj_z(end+1) = double(msg.pose.pose.position.z); %#ok<AGROW>
                obj.traj_time(end+1) = toc(obj.start_tic); %#ok<AGROW>
                obj.traj_wind_mag(end+1) = obj.latest_wind_mag; %#ok<AGROW>
                obj.traj_wind_vec(end+1, :) = obj.latest_wind_vec; %#ok<AGROW>

                excess = numel(obj.traj_x) - obj.max_history_points;
                if excess > 0
                    keep_start = excess + 1;
                    obj.traj_x = obj.traj_x(keep_start:end);
                    obj.traj_y = obj.traj_y(keep_start:end);
                    obj.traj_z = obj.traj_z(keep_start:end);
                    obj.traj_time = obj.traj_time(keep_start:end);
                    obj.traj_wind_mag = obj.traj_wind_mag(keep_start:end);
                    if size(obj.traj_wind_vec, 1) >= keep_start
                        obj.traj_wind_vec = obj.traj_wind_vec(keep_start:end, :);
                    end
                end

                obj.odom_msg_count = obj.odom_msg_count + 1;
            catch
            end
        end

        function poll_odom(obj)
            try
                if isempty(obj.sub_odom)
                    return;
                end
                max_msgs_per_cycle = 80;
                for msg_idx = 1:max_msgs_per_cycle
                    timeout_s = 0.0;
                    if msg_idx == 1
                        timeout_s = 0.01;
                    end
                    [msg, ~] = receive(obj.sub_odom, timeout_s);
                    if isempty(msg)
                        break;
                    end
                    obj.callback_odom(msg);
                end
            catch
            end
        end

        function callback_wind(obj, msg)
            try
                [wind_x, wind_y, wind_z] = obj.extract_vector3(msg);
                obj.latest_wind_vec = [wind_x, wind_y, wind_z];
                obj.latest_wind_mag = norm(obj.latest_wind_vec);
                obj.wind_history(end+1) = obj.latest_wind_mag; %#ok<AGROW>
                obj.wind_time(end+1) = toc(obj.start_tic); %#ok<AGROW>
                excess = numel(obj.wind_history) - obj.max_history_points;
                if excess > 0
                    keep_start = excess + 1;
                    obj.wind_history = obj.wind_history(keep_start:end);
                    obj.wind_time = obj.wind_time(keep_start:end);
                end
                obj.wind_msg_count = obj.wind_msg_count + 1;
            catch
            end
        end

        function callback_airspeed(obj, msg)
            try
                [wind_x, wind_y, wind_z] = obj.extract_airspeed_vector(msg);
                if isempty(obj.sub_wind) || obj.wind_msg_count == 0
                    obj.latest_wind_vec = [wind_x, wind_y, wind_z];
                    obj.latest_wind_mag = norm(obj.latest_wind_vec);
                    obj.wind_history(end+1) = obj.latest_wind_mag; %#ok<AGROW>
                    obj.wind_time(end+1) = toc(obj.start_tic); %#ok<AGROW>
                    excess = numel(obj.wind_history) - obj.max_history_points;
                    if excess > 0
                        keep_start = excess + 1;
                        obj.wind_history = obj.wind_history(keep_start:end);
                        obj.wind_time = obj.wind_time(keep_start:end);
                    end
                    obj.wind_msg_count = obj.wind_msg_count + 1;
                end
            catch
            end
        end

        function [wind_x, wind_y, wind_z] = extract_airspeed_vector(~, msg)
            wind_x = 0;
            wind_y = 0;
            wind_z = 0;
            try
                if isfield(msg, 'vector')
                    wind_x = double(msg.vector.x);
                    wind_y = double(msg.vector.y);
                    wind_z = double(msg.vector.z);
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

        function [x, y, z, speeds, vecs] = snapshot_plot_data(obj)
            n = min([numel(obj.traj_x), numel(obj.traj_y), numel(obj.traj_z)]);
            if n < 1
                x = [];
                y = [];
                z = [];
                speeds = [];
                vecs = zeros(0, 3);
                return;
            end

            x = obj.traj_x(1:n).';
            y = obj.traj_y(1:n).';
            z = obj.traj_z(1:n).';

            speeds = obj.traj_wind_mag(:);
            if numel(speeds) < n
                speeds(end+1:n) = NaN;
            elseif numel(speeds) > n
                speeds = speeds(1:n);
            end

            vecs = obj.traj_wind_vec;
            if isempty(vecs)
                vecs = NaN(n, 3);
            else
                if size(vecs, 2) < 3
                    vecs(:, end+1:3) = NaN;
                elseif size(vecs, 2) > 3
                    vecs = vecs(:, 1:3);
                end
                if size(vecs, 1) < n
                    vecs(end+1:n, 1:3) = NaN;
                elseif size(vecs, 1) > n
                    vecs = vecs(1:n, :);
                end
            end

            if n > obj.max_render_points
                idx = unique(round(linspace(1, n, obj.max_render_points)));
                x = x(idx);
                y = y(idx);
                z = z(idx);
                speeds = speeds(idx);
                vecs = vecs(idx, :);
            end
        end

        function update_wind_overlay(obj, x, y, z, speeds, vecs)
            try
                n = min([numel(x), numel(y), numel(z), numel(speeds), size(vecs, 1)]);
                if n < 1
                    return;
                end

                x = x(1:n);
                y = y(1:n);
                z = z(1:n);
                speeds = speeds(1:n);
                vecs = vecs(1:n, 1:3);
                if all(~isfinite(speeds))
                    speeds = zeros(n, 1);
                end

                set(obj.h_air_scatter, 'XData', x, 'YData', y, ...
                    'ZData', z, 'CData', speeds, 'SizeData', 28 * ones(n, 1));
                finite_speeds = speeds(isfinite(speeds));
                if ~isempty(finite_speeds)
                    clim(obj.ax_traj, [min(finite_speeds), max(finite_speeds) + eps]);
                end

                if n < 2
                    return;
                end

                max_arrows = 28;
                step = max(1, floor(n / max_arrows));
                idx = 1:step:n;
                if isempty(idx)
                    return;
                end

                speed_vals = speeds(idx);
                speed_vals(~isfinite(speed_vals)) = 0;
                max_speed = max(speed_vals);
                if max_speed <= 0
                    max_speed = 1;
                end
                span = max([range(x), range(y), range(z), 1]);
                arrow_len = span * 0.08 * (0.25 + 0.75 * speed_vals ./ max_speed);
                wind_norm = sqrt(sum(vecs(idx, :).^2, 2));
                valid = wind_norm > 1e-6;
                idx = idx(valid);
                arrow_len = arrow_len(:);
                arrow_len = arrow_len(valid);
                wind_norm = wind_norm(valid);
                if isempty(idx)
                    set(obj.h_air_quiver, 'XData', NaN, 'YData', NaN, 'ZData', NaN, ...
                        'UData', NaN, 'VData', NaN, 'WData', NaN);
                    return;
                end
                idx = idx(:);
                ux = vecs(idx, 1) ./ wind_norm(:) .* arrow_len(:);
                uy = vecs(idx, 2) ./ wind_norm(:) .* arrow_len(:);
                uz = vecs(idx, 3) ./ wind_norm(:) .* arrow_len(:);
                set(obj.h_air_quiver, 'XData', x(idx), 'YData', y(idx), ...
                    'ZData', z(idx), 'UData', ux(:), 'VData', uy(:), 'WData', uz(:));
            catch
            end
        end
    end
end
