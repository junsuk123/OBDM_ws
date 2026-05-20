classdef ArucoLandingControlModule < handle
    % ArucoLandingControlModule - search moving marker, generate descent waypoints, land.

    properties
        namespace = '/iris'
        verbose = true
        ros_node
        sub_odom
        sub_marker
        sub_visible
        odom = []
        marker = []
        marker_visible = false
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
            obj.sub_odom = ros2subscriber(obj.ros_node, sprintf('%s/odometry', obj.namespace), ...
                'nav_msgs/Odometry', @obj.cb_odom, 'Reliability', 'besteffort', 'Durability', 'volatile');
            obj.sub_marker = ros2subscriber(obj.ros_node, '/landing_marker/odom', ...
                'nav_msgs/Odometry', @obj.cb_marker, 'Reliability', 'besteffort', 'Durability', 'volatile');
            obj.sub_visible = ros2subscriber(obj.ros_node, '/landing_marker/visible', ...
                'std_msgs/Bool', @obj.cb_visible, 'Reliability', 'besteffort', 'Durability', 'volatile');
            if obj.verbose
                fprintf('[ArucoLandingControlModule] subscribed: %s/odometry, /landing_marker/odom\n', obj.namespace);
            end
        end

        function success = execute(obj, varargin)
            p = inputParser;
            addParameter(p, 'takeoff_alt_m', 5, @isnumeric);
            addParameter(p, 'search_alt_step_m', 1.0, @isnumeric);
            addParameter(p, 'max_search_alt_m', 12.0, @isnumeric);
            addParameter(p, 'search_timeout_s', 35, @isnumeric);
            addParameter(p, 'min_marker_samples', 5, @isnumeric);
            addParameter(p, 'tracking_timeout_s', 55, @isnumeric);
            addParameter(p, 'tracking_interval_s', 1.0, @isnumeric);
            addParameter(p, 'descent_step_m', 0.45, @isnumeric);
            addParameter(p, 'landing_alt_m', 0.8, @isnumeric);
            addParameter(p, 'landing_timeout_s', 45, @isnumeric);
            parse(p, varargin{:});

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

            if ~obj.run_mavlink('arm_takeoff', sprintf('%.3f', p.Results.takeoff_alt_m), 60)
                fprintf('[ArucoLandingControlModule] arm/takeoff failed\n');
                return;
            end
            pause(2.0);

            found = obj.ascend_until_marker( ...
                home, ...
                p.Results.takeoff_alt_m, ...
                p.Results.max_search_alt_m, ...
                p.Results.search_alt_step_m, ...
                p.Results.min_marker_samples, ...
                p.Results.search_timeout_s);
            if ~found
                fprintf('[ArucoLandingControlModule] marker not confirmed during ascending search; landing at current position\n');
                obj.run_mavlink('land', '', p.Results.landing_timeout_s);
                return;
            end

            tracked = obj.track_marker_average( ...
                p.Results.tracking_timeout_s, ...
                p.Results.tracking_interval_s, ...
                p.Results.descent_step_m, ...
                p.Results.landing_alt_m);
            if ~tracked
                fprintf('[ArucoLandingControlModule] marker-average tracking timeout; landing best-effort\n');
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
                clear obj.sub_odom obj.sub_marker obj.sub_visible obj.ros_node;
            catch
            end
        end
    end

    methods (Access = private)
        function cb_odom(obj, msg)
            p = msg.pose.pose.position;
            v = msg.twist.twist.linear;
            q = msg.pose.pose.orientation;
            yaw = obj.quat_to_yaw(double(q.x), double(q.y), double(q.z), double(q.w));
            obj.odom = struct('x', double(p.x), 'y', double(p.y), 'z', double(p.z), ...
                'vx', double(v.x), 'vy', double(v.y), 'vz', double(v.z), 'yaw', yaw);
        end

        function cb_marker(obj, msg)
            p = msg.pose.pose.position;
            v = msg.twist.twist.linear;
            obj.marker = struct('x', double(p.x), 'y', double(p.y), 'z', double(p.z), ...
                'vx', double(v.x), 'vy', double(v.y), 'vz', double(v.z));
            obj.update_marker_average(obj.marker);
        end

        function cb_visible(obj, msg)
            try
                obj.marker_visible = logical(msg.data);
            catch
                obj.marker_visible = false;
            end
        end

        function ok = wait_for_odom(obj, timeout_s)
            ok = false;
            t0 = tic;
            while toc(t0) < timeout_s
                if ~isempty(obj.odom) && isfinite(obj.odom.x)
                    ok = true;
                    return;
                end
                pause(0.2);
            end
        end

        function found = ascend_until_marker(obj, home, initial_alt_m, max_alt_m, alt_step_m, min_samples, timeout_s)
            found = false;
            t0 = tic;
            target_alt = max(initial_alt_m, home.z);
            while toc(t0) < timeout_s
                if obj.marker_visible && obj.marker_samples >= min_samples && ~isempty(obj.marker_avg)
                    found = true;
                    fprintf('[ArucoLandingControlModule] marker confirmed: avg=(%.2f, %.2f, %.2f), samples=%d\n', ...
                        obj.marker_avg.x, obj.marker_avg.y, obj.marker_avg.z, obj.marker_samples);
                    return;
                end
                target_alt = min(max_alt_m, target_alt + alt_step_m);
                target = [home.x, home.y, target_alt];
                obj.send_local_waypoint(target(1), target(2), target(3));
                obj.log_sample('search_ascend', target);
                pause(1.0);
            end
        end

        function tracked = track_marker_average(obj, timeout_s, interval_s, descent_step_m, landing_alt_m)
            tracked = false;
            if isempty(obj.odom) || isempty(obj.marker_avg)
                return;
            end

            current_alt = max(obj.odom.z, landing_alt_m + 1.0);
            t0 = tic;
            last_sample_count = obj.marker_samples;
            while toc(t0) < timeout_s
                if ~isempty(obj.marker_avg)
                    target_xy = [obj.marker_avg.x, obj.marker_avg.y];
                    if obj.marker_samples > last_sample_count
                        last_sample_count = obj.marker_samples;
                    end
                elseif ~isempty(obj.marker)
                    target_xy = [obj.marker.x, obj.marker.y];
                else
                    pause(interval_s);
                    continue;
                end

                if ~isempty(obj.odom)
                    current_alt = max(landing_alt_m, min(current_alt, obj.odom.z) - descent_step_m);
                else
                    current_alt = max(landing_alt_m, current_alt - descent_step_m);
                end

                target = [target_xy(1), target_xy(2), current_alt];
                obj.send_local_waypoint(target(1), target(2), target(3));
                obj.log_sample('track_marker_average', target);

                if ~isempty(obj.odom)
                    horizontal_err = hypot(obj.odom.x - target(1), obj.odom.y - target(2));
                    vertical_err = abs(obj.odom.z - target(3));
                    if current_alt <= landing_alt_m + 0.05 && horizontal_err <= 1.5 && vertical_err <= 1.0
                        tracked = true;
                        return;
                    end
                end
                pause(interval_s);
            end
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

        function avg = marker_average_or_nan(obj)
            if isempty(obj.marker_avg)
                avg = struct('x', NaN, 'y', NaN, 'z', NaN);
            else
                avg = obj.marker_avg;
            end
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
            obj.run_mavlink('local_wp', sprintf('%.6f %.6f %.6f', north_m, east_m, z_up), 10);
        end

        function [forward_x, forward_y] = heading_unit(~, state)
            if isfield(state, 'yaw') && isfinite(state.yaw)
                forward_x = cos(state.yaw);
                forward_y = sin(state.yaw);
            else
                forward_x = 0;
                forward_y = 1;
            end
            n = hypot(forward_x, forward_y);
            if n < 1e-6
                forward_x = 0;
                forward_y = 1;
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
                'for uri in ("udp:127.0.0.1:14550", "tcp:127.0.0.1:5760"):\n' ...
                '    try:\n' ...
                '        m = mavutil.mavlink_connection(uri, source_system=251, timeout=5)\n' ...
                '        m.wait_heartbeat(timeout=10)\n' ...
                '        break\n' ...
                '    except Exception:\n' ...
                '        m = None\n' ...
                'if m is None:\n' ...
                '    raise RuntimeError("no mavlink connection")\n' ...
                'def hb():\n' ...
                '    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE, 3)\n' ...
                'hb()\n' ...
                'if action == "arm_takeoff":\n' ...
                '    alt = float(value)\n' ...
                '    m.set_mode("GUIDED")\n' ...
                '    deadline = time.time() + 12\n' ...
                '    while time.time() < deadline and str(m.flightmode).upper() != "GUIDED":\n' ...
                '        hb(); m.recv_match(type="HEARTBEAT", blocking=True, timeout=1); time.sleep(0.2)\n' ...
                '    if str(m.flightmode).upper() != "GUIDED":\n' ...
                '        raise RuntimeError("GUIDED mode not confirmed")\n' ...
                '    m.arducopter_arm()\n' ...
                '    deadline = time.time() + 15\n' ...
                '    while time.time() < deadline and not m.motors_armed():\n' ...
                '        hb(); m.recv_match(type="HEARTBEAT", blocking=True, timeout=1); time.sleep(0.2)\n' ...
                '    if not m.motors_armed():\n' ...
                '        raise RuntimeError("arm not confirmed")\n' ...
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
            cmd = sprintf('timeout %d python3 "%s" "%s" "%s" 2>&1', ceil(timeout_s), py_file, action, value);
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
