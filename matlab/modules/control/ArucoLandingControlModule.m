classdef ArucoLandingControlModule < handle
    % ArucoLandingControlModule - search moving marker, generate descent waypoints, land.

    properties
        namespace = '/iris'
        verbose = true
        ros_node
        sub_odom
        sub_marker
        sub_visible
        sub_servo_error
        odom = []
        marker = []
        marker_visible = false
        marker_time_s = NaN
        servo_error = []
        servo_error_time_s = NaN
        marker_avg = []
        marker_samples = 0
        marker_history
        flight_log
        start_tic
    end

    methods
        function obj = ArucoLandingControlModule(varargin)
            p = inputParser;
            addParameter(p, 'namespace', '/iris', @ischar);
            addParameter(p, 'verbose', true, @islogical);
            parse(p, varargin{:});
            obj.namespace = p.Results.namespace;
            obj.verbose = p.Results.verbose;
            obj.flight_log = obj.empty_log();
            obj.marker_history = obj.empty_marker_history();
        end

        function obj = initialize(obj)
            obj.ros_node = ros2node('/aruco_landing_control');
            odom_topic = SimulationConfig.resolve_topic('odometry', obj.namespace);
            obj.sub_odom = ros2subscriber(obj.ros_node, odom_topic, ...
                'nav_msgs/Odometry', 'Reliability', 'besteffort', 'Durability', 'volatile');
            obj.sub_marker = ros2subscriber(obj.ros_node, '/landing_marker/odom', ...
                'nav_msgs/Odometry', 'Reliability', 'reliable', 'Durability', 'volatile');
            obj.sub_visible = ros2subscriber(obj.ros_node, '/landing_marker/visible', ...
                'std_msgs/Bool', 'Reliability', 'reliable', 'Durability', 'volatile');
            obj.sub_servo_error = ros2subscriber(obj.ros_node, '/landing_marker/visual_servo_error', ...
                'geometry_msgs/Vector3Stamped', 'Reliability', 'reliable', 'Durability', 'volatile');
            if obj.verbose
                fprintf('[ArucoLandingControlModule] subscribed: %s, /landing_marker/odom, /landing_marker/visible, /landing_marker/visual_servo_error\n', odom_topic);
            end
        end

        function success = execute(obj, varargin)
            p = inputParser;
            addParameter(p, 'takeoff_alt_m', 5, @isnumeric);
            addParameter(p, 'search_alt_step_m', 1.0, @isnumeric);
            addParameter(p, 'search_step_m', NaN, @isnumeric);
            addParameter(p, 'max_search_alt_m', 12.0, @isnumeric);
            addParameter(p, 'search_timeout_s', 35, @isnumeric);
            addParameter(p, 'min_marker_samples', 5, @isnumeric);
            addParameter(p, 'search_forward_step_m', 4.0, @isnumeric);
            addParameter(p, 'search_max_forward_m', 16.0, @isnumeric);
            addParameter(p, 'search_lateral_step_m', 3.0, @isnumeric);
            addParameter(p, 'tracking_timeout_s', 55, @isnumeric);
            addParameter(p, 'tracking_interval_s', 1.0, @isnumeric);
            addParameter(p, 'descent_step_m', 0.45, @isnumeric);
            addParameter(p, 'landing_alt_m', 0.8, @isnumeric);
            addParameter(p, 'landing_timeout_s', 45, @isnumeric);
            addParameter(p, 'approach_points', NaN, @isnumeric);
            addParameter(p, 'approach_timeout_s', NaN, @isnumeric);
            addParameter(p, 'servo_gain_m', 1.8, @isnumeric);
            addParameter(p, 'prediction_lead_s', 1.2, @isnumeric);
            addParameter(p, 'landing_close_m', 1.2, @isnumeric);
            parse(p, varargin{:});

            search_alt_step_m = p.Results.search_alt_step_m;
            if isfinite(p.Results.search_step_m)
                search_alt_step_m = p.Results.search_step_m;
            end
            tracking_timeout_s = p.Results.tracking_timeout_s;
            if isfinite(p.Results.approach_timeout_s)
                tracking_timeout_s = p.Results.approach_timeout_s;
            end
            descent_step_m = p.Results.descent_step_m;
            if isfinite(p.Results.approach_points) && p.Results.approach_points > 0
                descent_span_m = max(0.1, p.Results.takeoff_alt_m - p.Results.landing_alt_m);
                descent_step_m = max(0.1, descent_span_m / p.Results.approach_points);
            end

            success = false;
            obj.flight_log = obj.empty_log();
            obj.marker_history = obj.empty_marker_history();
            obj.marker_avg = [];
            obj.marker_samples = 0;
            obj.start_tic = tic;

            if ~obj.wait_for_odom(30)
                fprintf('[ArucoLandingControlModule] no odometry; aborting\n');
                return;
            end

            home = obj.odom;
            if obj.verbose
                fprintf('[ArucoLandingControlModule] home: x=%.2f y=%.2f z=%.2f\n', home.x, home.y, home.z);
            end

            if ~obj.run_mavlink('arm_takeoff', sprintf('%.3f', p.Results.takeoff_alt_m), 90)
                fprintf('[ArucoLandingControlModule] arm/takeoff failed\n');
                return;
            end
            pause(2.0);

            found = obj.ascend_until_marker( ...
                home, ...
                p.Results.takeoff_alt_m, ...
                p.Results.max_search_alt_m, ...
                search_alt_step_m, ...
                p.Results.min_marker_samples, ...
                p.Results.search_timeout_s, ...
                p.Results.search_forward_step_m, ...
                p.Results.search_max_forward_m, ...
                p.Results.search_lateral_step_m);
            if ~found
                fprintf('[ArucoLandingControlModule] marker not detected in camera during search window; landing safely\n');
                obj.run_mavlink('land', '', p.Results.landing_timeout_s);
                return;
            end

            tracked = obj.visual_servo_landing( ...
                tracking_timeout_s, ...
                p.Results.tracking_interval_s, ...
                descent_step_m, ...
                p.Results.landing_alt_m, ...
                p.Results.takeoff_alt_m, ...
                p.Results.servo_gain_m, ...
                p.Results.prediction_lead_s, ...
                p.Results.landing_close_m);
            if ~tracked
                fprintf('[ArucoLandingControlModule] visual servo tracking timeout; landing best-effort\n');
            end

            obj.run_mavlink('land', '', p.Results.landing_timeout_s);
            success = true;
            if obj.verbose
                fprintf('[ArucoLandingControlModule] landing mission complete\n');
            end
        end

        function log = get_flight_log(obj)
            log = obj.flight_log;
        end

        function cleanup(obj)
            try
                clear obj.sub_odom obj.sub_marker obj.sub_visible obj.sub_servo_error obj.ros_node;
            catch
            end
        end
    end

    methods (Access = private)
        function cb_odom(obj, msg)
            yaw = NaN;
            vx = NaN;
            vy = NaN;
            vz = NaN;
            try
                p = msg.pose.pose.position;
                if isfield(msg, 'twist') && isfield(msg.twist, 'twist') && isfield(msg.twist.twist, 'linear')
                    v = msg.twist.twist.linear;
                    vx = double(v.x);
                    vy = double(v.y);
                    vz = double(v.z);
                end
                if isfield(msg, 'pose') && isfield(msg.pose, 'pose') && isfield(msg.pose.pose, 'orientation')
                    q = msg.pose.pose.orientation;
                    yaw = obj.quat_to_yaw(double(q.x), double(q.y), double(q.z), double(q.w));
                end
                obj.odom = struct('x', double(p.x), 'y', double(p.y), 'z', double(p.z), ...
                    'vx', vx, 'vy', vy, 'vz', vz, 'yaw', yaw);
            catch
                try
                    p = msg.pose.pose.position;
                    obj.odom = struct('x', double(p.x), 'y', double(p.y), 'z', double(p.z), ...
                        'vx', vx, 'vy', vy, 'vz', vz, 'yaw', yaw);
                catch
                end
            end
        end

        function cb_marker(obj, msg)
            p = msg.pose.pose.position;
            v = msg.twist.twist.linear;
            if obj.is_zero_marker(p, v)
                return;
            end
            obj.marker = struct('x', double(p.x), 'y', double(p.y), 'z', double(p.z), ...
                'vx', double(v.x), 'vy', double(v.y), 'vz', double(v.z));
            obj.marker_time_s = obj.elapsed_s();
            obj.update_marker_average(obj.marker);
        end

        function cb_visible(obj, msg)
            try
                obj.marker_visible = logical(msg.data);
            catch
                obj.marker_visible = false;
            end
        end

        function poll_odom(obj, timeout_s)
            if nargin < 2 || isempty(timeout_s)
                timeout_s = 0.1;
            end
            if isempty(obj.sub_odom)
                return;
            end

            try
                [msg, ~] = receive(obj.sub_odom, timeout_s);
                if ~isempty(msg)
                    obj.cb_odom(msg);
                end
            catch
            end
        end

        function poll_marker(obj, timeout_s)
            if nargin < 2 || isempty(timeout_s)
                timeout_s = 0.02;
            end

            try
                if ~isempty(obj.sub_marker)
                    [msg, ~] = receive(obj.sub_marker, timeout_s);
                    if ~isempty(msg)
                        obj.cb_marker(msg);
                        if ~isempty(obj.marker)
                            obj.marker_visible = true;
                            return;
                        end
                    end
                end
            catch
            end

            try
                if ~isempty(obj.sub_visible)
                    [msg, ~] = receive(obj.sub_visible, timeout_s);
                    if ~isempty(msg)
                        obj.cb_visible(msg);
                    end
                end
            catch
            end

            obj.poll_servo_error(0.01);

        end

        function poll_servo_error(obj, timeout_s)
            if nargin < 2 || isempty(timeout_s)
                timeout_s = 0.01;
            end
            try
                if ~isempty(obj.sub_servo_error)
                    [msg, ~] = receive(obj.sub_servo_error, timeout_s);
                    if ~isempty(msg)
                        v = msg.vector;
                        obj.servo_error = struct('x', double(v.x), 'y', double(v.y), 'area', double(v.z));
                        obj.servo_error_time_s = obj.elapsed_s();
                    end
                end
            catch
            end
        end

        function ok = wait_for_odom(obj, timeout_s)
            ok = false;
            t0 = tic;
            while toc(t0) < timeout_s
                obj.poll_odom(0.25);
                obj.poll_marker(0.05);
                if ~isempty(obj.odom) && isfinite(obj.odom.x) && isfinite(obj.odom.z) && obj.odom.z > 0.05
                    ok = true;
                    return;
                end
                pause(0.2);
            end
        end

        function found = ascend_until_marker(obj, home, initial_alt_m, max_alt_m, alt_step_m, min_samples, timeout_s, forward_step_m, max_forward_m, lateral_step_m)
            found = false;
            t0 = tic;
            target_alt = max(initial_alt_m, home.z);
            search_idx = 0;
            scan_points = obj.build_search_points(home, forward_step_m, max_forward_m, lateral_step_m);
            while toc(t0) < timeout_s
                obj.poll_odom(0.1);
                obj.poll_marker(0.15);
                if obj.marker_visible && obj.marker_samples >= min_samples && ~isempty(obj.marker_avg)
                    found = true;
                    fprintf('[ArucoLandingControlModule] marker confirmed: avg=(%.2f, %.2f, %.2f), samples=%d\n', ...
                        obj.marker_avg.x, obj.marker_avg.y, obj.marker_avg.z, obj.marker_samples);
                    return;
                end

                if target_alt < max_alt_m
                    target_alt = min(max_alt_m, target_alt + alt_step_m);
                    target = [home.x, home.y, target_alt];
                    phase = 'search_ascend';
                else
                    search_idx = search_idx + 1;
                    target_xy = scan_points(mod(search_idx - 1, size(scan_points, 1)) + 1, :);
                    target = [target_xy(1), target_xy(2), target_alt];
                    phase = 'search_sweep';
                end

                obj.send_local_waypoint(target(1), target(2), target(3));
                obj.log_sample(phase, target);
                pause(1.0);
            end
        end

        function tracked = visual_servo_landing(obj, timeout_s, interval_s, descent_step_m, landing_alt_m, initial_alt_m, servo_gain_m, prediction_lead_s, landing_close_m)
            tracked = false;
            if isempty(obj.odom) || isempty(obj.marker_avg)
                return;
            end

            if nargin < 6 || isempty(initial_alt_m)
                initial_alt_m = landing_alt_m + 1.0;
            end
            current_alt = max([obj.odom.z, initial_alt_m, landing_alt_m + 1.0]);

            t0 = tic;
            last_sample_count = obj.marker_samples;
            while toc(t0) < timeout_s
                obj.poll_odom(0.1);
                obj.poll_marker(0.1);
                obj.poll_servo_error(0.02);
                if ~isempty(obj.marker)
                    target_xy = obj.predicted_marker_xy(prediction_lead_s);
                    if obj.marker_samples > last_sample_count
                        last_sample_count = obj.marker_samples;
                    end
                else
                    pause(interval_s);
                    continue;
                end

                if obj.has_recent_servo_error(2.0)
                    target_xy = obj.apply_visual_servo_correction(target_xy, servo_gain_m);
                end

                odom_alt_valid = ~isempty(obj.odom) && isfinite(obj.odom.z) && obj.odom.z > landing_alt_m + 0.2;
                if odom_alt_valid
                    current_alt = max(landing_alt_m, min(current_alt, obj.odom.z) - descent_step_m);
                else
                    current_alt = max(landing_alt_m, current_alt - descent_step_m);
                end

                target = [target_xy(1), target_xy(2), current_alt];
                obj.send_local_waypoint(target(1), target(2), target(3));
                obj.log_sample('visual_servo', target);

                if ~isempty(obj.odom) && odom_alt_valid
                    predicted_xy = obj.predicted_marker_xy(prediction_lead_s);
                    horizontal_err = hypot(obj.odom.x - predicted_xy(1), obj.odom.y - predicted_xy(2));
                    vertical_err = abs(obj.odom.z - target(3));
                    if current_alt <= landing_alt_m + 0.05 && horizontal_err <= landing_close_m && vertical_err <= 1.0
                        predicted_target = [predicted_xy(1), predicted_xy(2), landing_alt_m];
                        obj.send_local_waypoint(predicted_target(1), predicted_target(2), predicted_target(3));
                        obj.log_sample('predictive_landing', predicted_target);
                        tracked = true;
                        return;
                    end
                elseif current_alt <= landing_alt_m + 0.05 && obj.marker_samples >= last_sample_count
                    tracked = true;
                    return;
                end
                pause(interval_s);
            end
        end

        function target_xy = predicted_marker_xy(obj, lead_s)
            if nargin < 2 || isempty(lead_s) || ~isfinite(lead_s)
                lead_s = 0.0;
            end
            marker = obj.marker;
            if isempty(marker)
                marker = obj.marker_avg;
            end
            if isempty(marker)
                target_xy = [obj.odom.x, obj.odom.y];
                return;
            end
            target_xy = [marker.x + marker.vx * lead_s, marker.y + marker.vy * lead_s];
        end

        function target_xy = apply_visual_servo_correction(obj, target_xy, servo_gain_m)
            if isempty(obj.servo_error)
                return;
            end
            if nargin < 3 || isempty(servo_gain_m) || ~isfinite(servo_gain_m)
                servo_gain_m = 1.8;
            end
            err_x = max(-1.0, min(1.0, obj.servo_error.x));
            err_y = max(-1.0, min(1.0, obj.servo_error.y));
            % Down-looking optical frame: image x is lateral, image y is
            % approximately forward/back in the runway setup.
            target_xy(1) = target_xy(1) - servo_gain_m * err_y;
            target_xy(2) = target_xy(2) + servo_gain_m * err_x;
        end

        function recent = has_recent_servo_error(obj, max_age_s)
            if nargin < 2 || isempty(max_age_s)
                max_age_s = 2.0;
            end
            recent = ~isempty(obj.servo_error) && isfinite(obj.servo_error_time_s) && ...
                (obj.elapsed_s() - obj.servo_error_time_s) <= max_age_s;
        end

        function update_marker_average(obj, marker)
            if isempty(marker) || ~isfinite(marker.x) || ~isfinite(marker.y)
                return;
            end
            obj.marker_samples = obj.marker_samples + 1;
            if isempty(obj.marker_avg)
                obj.marker_avg = marker;
            else
                n = obj.marker_samples;
                obj.marker_avg.x = obj.marker_avg.x + (marker.x - obj.marker_avg.x) / n;
                obj.marker_avg.y = obj.marker_avg.y + (marker.y - obj.marker_avg.y) / n;
                obj.marker_avg.z = obj.marker_avg.z + (marker.z - obj.marker_avg.z) / n;
                obj.marker_avg.vx = obj.marker_avg.vx + (marker.vx - obj.marker_avg.vx) / n;
                obj.marker_avg.vy = obj.marker_avg.vy + (marker.vy - obj.marker_avg.vy) / n;
                obj.marker_avg.vz = obj.marker_avg.vz + (marker.vz - obj.marker_avg.vz) / n;
            end

            try
                t = toc(obj.start_tic);
            catch
                t = NaN;
            end
            obj.marker_history.time_s(end+1,1) = t;
            obj.marker_history.x(end+1,1) = marker.x;
            obj.marker_history.y(end+1,1) = marker.y;
            obj.marker_history.z(end+1,1) = marker.z;
            obj.marker_history.avg_x(end+1,1) = obj.marker_avg.x;
            obj.marker_history.avg_y(end+1,1) = obj.marker_avg.y;
            obj.marker_history.avg_z(end+1,1) = obj.marker_avg.z;
            obj.marker_history.samples(end+1,1) = obj.marker_samples;
        end

        function history = empty_marker_history(~)
            history = struct('time_s', [], 'x', [], 'y', [], 'z', [], ...
                'avg_x', [], 'avg_y', [], 'avg_z', [], 'samples', []);
        end

        function t = elapsed_s(obj)
            try
                t = toc(obj.start_tic);
            catch
                t = NaN;
            end
        end

        function avg = marker_average_or_nan(obj)
            if isempty(obj.marker_avg)
                avg = struct('x', NaN, 'y', NaN, 'z', NaN);
            else
                avg = obj.marker_avg;
            end
        end

        function is_zero = is_zero_marker(~, p, v)
            vals = [double(p.x), double(p.y), double(p.z), double(v.x), double(v.y), double(v.z)];
            is_zero = all(isfinite(vals)) && all(abs(vals) < 1e-9);
        end

        function send_local_waypoint(obj, x_world, y_world, z_up)
            % MATLAB / ROS / Gazebo logic uses world coordinates:
            %   x_world, y_world, z_up
            % ArduPilot MAVLink LOCAL_NED expects:
            %   north, east, down
            % The Gazebo runway convention used here maps world Y to north
            % and world X to east. Keep this conversion at the boundary so
            % path generation, logging, and visualization share one frame.
            north_m = y_world;
            east_m = x_world;
            obj.run_mavlink('local_wp', sprintf('%.6f %.6f %.6f', north_m, east_m, z_up), 25);
        end

        function points = build_search_points(obj, home, forward_step_m, max_forward_m, lateral_step_m)
            if nargin < 3 || ~isfinite(forward_step_m) || forward_step_m <= 0
                forward_step_m = 4.0;
            end
            if nargin < 4 || ~isfinite(max_forward_m) || max_forward_m <= 0
                max_forward_m = 16.0;
            end
            if nargin < 5 || ~isfinite(lateral_step_m) || lateral_step_m < 0
                lateral_step_m = 3.0;
            end

            [fx, fy] = obj.heading_unit(home);
            lx = -fy;
            ly = fx;
            forward_vals = 0:forward_step_m:max_forward_m;
            lateral_vals = [0, lateral_step_m, -lateral_step_m];
            points = zeros(numel(forward_vals) * numel(lateral_vals), 2);
            idx = 1;
            for fi = 1:numel(forward_vals)
                if mod(fi, 2) == 1
                    laterals = lateral_vals;
                else
                    laterals = fliplr(lateral_vals);
                end
                for li = 1:numel(laterals)
                    points(idx, 1) = home.x + forward_vals(fi) * fx + laterals(li) * lx;
                    points(idx, 2) = home.y + forward_vals(fi) * fy + laterals(li) * ly;
                    idx = idx + 1;
                end
            end
        end

        function [forward_x, forward_y] = heading_unit(~, state)
            % The target node initializes the pad in front of the vehicle
            % using this same yaw-to-world convention.
            if isfield(state, 'yaw') && isfinite(state.yaw)
                forward_x = cos(state.yaw);
                forward_y = sin(state.yaw);
            else
                forward_x = 1;
                forward_y = 0;
            end
            n = hypot(forward_x, forward_y);
            if n < 1e-6
                forward_x = 1;
                forward_y = 0;
            else
                forward_x = forward_x / n;
                forward_y = forward_y / n;
            end
        end

        function yaw = quat_to_yaw(~, x, y, z, w)
            siny_cosp = 2.0 * (w * z + x * y);
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
            yaw = atan2(siny_cosp, cosy_cosp);
        end

        function ok = run_mavlink(obj, action, value, timeout_s)
            ok = false;
            py_file = [tempname, '.py'];
            fid = fopen(py_file, 'w');
            if fid < 0, return; end
            cleaner = onCleanup(@() obj.delete_temp_file(py_file)); %#ok<NASGU>
            script = sprintf([ ...
                'import sys,time\n' ...
                'from pymavlink import mavutil\n' ...
                'action = sys.argv[1]\n' ...
                'value = " ".join(sys.argv[2:])\n' ...
                'm = None\n' ...
                'last_err = ""\n' ...
                'if action == "local_wp":\n' ...
                '    uris = ("udpin:127.0.0.1:14550", "udpin:127.0.0.1:14551")\n' ...
                'else:\n' ...
                '    uris = ("udpin:127.0.0.1:14550", "udpin:127.0.0.1:14551", "tcp:127.0.0.1:5760")\n' ...
                'for uri in uris:\n' ...
                '    try:\n' ...
                '        m = mavutil.mavlink_connection(uri, source_system=251, timeout=2)\n' ...
                '        heartbeat = m.wait_heartbeat(timeout=2 if action == "local_wp" else 4)\n' ...
                '        if heartbeat is None:\n' ...
                '            raise RuntimeError("heartbeat timeout")\n' ...
                '        print(f"[MAVLINK] connected uri={uri} target={m.target_system}:{m.target_component} mode={m.flightmode}")\n' ...
                '        break\n' ...
                '    except Exception as e:\n' ...
                '        last_err = f"{uri}: {e}"\n' ...
                '        m = None\n' ...
                'if m is None:\n' ...
                '    raise RuntimeError(f"no mavlink connection ({last_err})")\n' ...
                'def hb():\n' ...
                '    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE, 3)\n' ...
                'def statustext_drain(duration=1.0):\n' ...
                '    end = time.time() + duration\n' ...
                '    while time.time() < end:\n' ...
                '        msg = m.recv_match(type=["STATUSTEXT", "COMMAND_ACK", "HEARTBEAT"], blocking=True, timeout=0.2)\n' ...
                '        if msg is None:\n' ...
                '            continue\n' ...
                '        typ = msg.get_type()\n' ...
                '        if typ == "STATUSTEXT": print("STATUSTEXT:", msg.text)\n' ...
                '        elif typ == "COMMAND_ACK": print(f"COMMAND_ACK command={msg.command} result={msg.result}")\n' ...
                'def wait_armed(timeout=20):\n' ...
                '    deadline = time.time() + timeout\n' ...
                '    armed = False\n' ...
                '    while time.time() < deadline:\n' ...
                '        hb()\n' ...
                '        msg = m.recv_match(type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"], blocking=True, timeout=1)\n' ...
                '        if msg is None:\n' ...
                '            continue\n' ...
                '        if msg.get_type() == "HEARTBEAT":\n' ...
                '            armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0\n' ...
                '            if armed: return True\n' ...
                '        elif msg.get_type() == "STATUSTEXT": print("STATUSTEXT:", msg.text)\n' ...
                '        elif msg.get_type() == "COMMAND_ACK": print(f"COMMAND_ACK command={msg.command} result={msg.result}")\n' ...
                '    return armed\n' ...
                'hb()\n' ...
                'try:\n' ...
                '    m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1)\n' ...
                'except Exception:\n' ...
                '    pass\n' ...
                'if action == "arm_takeoff":\n' ...
                '    alt = float(value)\n' ...
                '    mapping = m.mode_mapping() or {}\n' ...
                '    if "GUIDED" in mapping:\n' ...
                '        m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping["GUIDED"])\n' ...
                '    else:\n' ...
                '        m.set_mode("GUIDED")\n' ...
                '    deadline = time.time() + 12\n' ...
                '    while time.time() < deadline and str(m.flightmode).upper() != "GUIDED":\n' ...
                '        hb(); m.recv_match(type="HEARTBEAT", blocking=True, timeout=1); time.sleep(0.2)\n' ...
                '    if str(m.flightmode).upper() != "GUIDED":\n' ...
                '        print(f"[ARM_MODE] WARNING: proceeding without confirmed guided mode (current={m.flightmode})")\n' ...
                '    statustext_drain(0.5)\n' ...
                '    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)\n' ...
                '    if not wait_armed(8):\n' ...
                '        print("[ARM_MODE] standard arm not confirmed; trying force arm")\n' ...
                '        m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196,0,0,0,0,0)\n' ...
                '    if not wait_armed(15):\n' ...
                '        raise RuntimeError("arm not confirmed")\n' ...
                '    print("[ARM_MODE] armed confirmed")\n' ...
                '    m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, alt)\n' ...
                '    rose = False\n' ...
                '    takeoff_sent_t = time.time()\n' ...
                '    deadline = time.time() + 40\n' ...
                '    while time.time() < deadline:\n' ...
                '        hb()\n' ...
                '        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED", "STATUSTEXT"], blocking=True, timeout=1)\n' ...
                '        if msg is None:\n' ...
                '            if time.time() - takeoff_sent_t > 5 and not rose:\n' ...
                '                m.mav.command_long_send(m.target_system, m.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,0,0,0,0,0, alt)\n' ...
                '                takeoff_sent_t = time.time()\n' ...
                '            continue\n' ...
                '        if msg.get_type() == "GLOBAL_POSITION_INT":\n' ...
                '            rel_alt_m = float(msg.relative_alt) / 1000.0\n' ...
                '            print(f"TAKEOFF rel_alt={rel_alt_m:.2f}")\n' ...
                '            if rel_alt_m >= max(0.8, alt * 0.08): rose = True; break\n' ...
                '        elif msg.get_type() == "LOCAL_POSITION_NED":\n' ...
                '            z = float(msg.z); print(f"TAKEOFF z={z:.2f}")\n' ...
                '            if z < -0.8: rose = True; break\n' ...
                '        elif msg.get_type() == "STATUSTEXT":\n' ...
                '            print("STATUSTEXT:", msg.text)\n' ...
                '    if not rose:\n' ...
                '        raise RuntimeError("takeoff climb not detected")\n' ...
                'elif action == "local_wp":\n' ...
                '    x,y,z = [float(v) for v in value.split()]\n' ...
                '    m.mav.set_position_target_local_ned_send(int(time.time()*1000)&0xffffffff, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, x, y, -z, 0,0,0, 0,0,0, 0,0)\n' ...
                '    time.sleep(0.1)\n' ...
                'elif action == "land":\n' ...
                '    m.set_mode("LAND")\n' ...
                '    time.sleep(3)\n' ...
                'else:\n' ...
                '    raise RuntimeError(f"unknown action: {action}")\n' ...
                'print("OK")\n' ...
            ]);
            fprintf(fid, '%s', script);
            fclose(fid);
            cmd = sprintf('timeout %d python3 "%s" "%s" "%s" 2>&1', ceil(timeout_s) + 10, py_file, action, value);
            [status, out] = system(cmd);
            ok = status == 0;
            if ~ok && obj.verbose
                fprintf('[ArucoLandingControlModule] pymavlink %s failed (status=%d): %s\n', ...
                    action, status, strtrim(out));
            end
        end

        function delete_temp_file(~, file_path)
            try
                if exist(file_path, 'file')
                    delete(file_path);
                end
            catch
            end
        end

        function log_sample(obj, phase, target)
            try
                t = toc(obj.start_tic);
                obj.flight_log.time_s(end+1,1) = t;
                obj.flight_log.phase{end+1,1} = phase;
                obj.flight_log.x(end+1,1) = obj.odom.x;
                obj.flight_log.y(end+1,1) = obj.odom.y;
                obj.flight_log.z(end+1,1) = obj.odom.z;
                obj.flight_log.target_x(end+1,1) = target(1);
                obj.flight_log.target_y(end+1,1) = target(2);
                obj.flight_log.target_z(end+1,1) = target(3);
                if ~isempty(obj.marker)
                    obj.flight_log.marker_x(end+1,1) = obj.marker.x;
                    obj.flight_log.marker_y(end+1,1) = obj.marker.y;
                    obj.flight_log.marker_z(end+1,1) = obj.marker.z;
                    obj.flight_log.marker_vx(end+1,1) = obj.marker.vx;
                    obj.flight_log.marker_vy(end+1,1) = obj.marker.vy;
                    obj.flight_log.marker_vz(end+1,1) = obj.marker.vz;
                else
                    obj.flight_log.marker_x(end+1,1) = NaN;
                    obj.flight_log.marker_y(end+1,1) = NaN;
                    obj.flight_log.marker_z(end+1,1) = NaN;
                    obj.flight_log.marker_vx(end+1,1) = NaN;
                    obj.flight_log.marker_vy(end+1,1) = NaN;
                    obj.flight_log.marker_vz(end+1,1) = NaN;
                end
                avg = obj.marker_average_or_nan();
                obj.flight_log.marker_avg_x(end+1,1) = avg.x;
                obj.flight_log.marker_avg_y(end+1,1) = avg.y;
                obj.flight_log.marker_avg_z(end+1,1) = avg.z;
                obj.flight_log.marker_samples(end+1,1) = obj.marker_samples;
            catch
            end
        end

        function log = empty_log(~)
            log = struct('time_s', [], 'phase', {{}}, 'x', [], 'y', [], 'z', [], ...
                'target_x', [], 'target_y', [], 'target_z', [], ...
                'marker_x', [], 'marker_y', [], 'marker_z', [], ...
                'marker_vx', [], 'marker_vy', [], 'marker_vz', [], ...
                'marker_avg_x', [], 'marker_avg_y', [], 'marker_avg_z', [], ...
                'marker_samples', []);
        end
    end
end
