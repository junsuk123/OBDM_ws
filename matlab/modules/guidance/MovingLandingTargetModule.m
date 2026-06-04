classdef MovingLandingTargetModule < ModuleBase
    % MovingLandingTargetModule starts the ROS2/Gazebo moving ArUco target helper.

    properties
        pid = ''
        detector_pid = ''
        camera_bridge_pid = ''
        guidance_streamer_pid = ''
        trajectory_generator_pid = ''
        guidance_path_visualizer_pid = ''
        rviz_tf_pid = ''
        rviz_pid = ''
        virtual_wind_pid = ''
        wind_arrow_pid = ''
        rover_bridge_pid = ''
        namespace = '/iris'
        world_name = 'runway'
        target_model_name = 'r1_rover'
        target_marker_z = 0.20
        target_model_z = 0.05
        target_spawn_forward_m = 1.0
        target_motion_range_m = 1.0
        target_min_forward_offset_m = 0.0
        target_max_forward_offset_m = 2.0
        target_initial_x = 0.0
        target_initial_y = 0.0
        target_initial_yaw_rad = 0.0
        target_motion_mode = 'line'
        target_circle_radius_m = 0.6
        target_circle_angular_speed = 0.24
        target_random_min_speed = 0.18
        target_random_max_speed = 0.42
        target_random_yawrate_max = 0.28
        target_random_change_period = 2.5
        target_random_lateral_range_m = 0.8
        wait_for_takeoff = true
        wait_for_motion_enable = false
        motion_enable_topic = '/obdm/start_pad_motion'
        takeoff_z_threshold = 0.8
        contact_arming_z_threshold = 1.5
        guidance_takeoff_altitude = 5.0
        touchdown_z_threshold = 0.35
        touchdown_xy_tolerance = 0.75
        touchdown_max_abs_vz = 1.0
        touchdown_ground_z_threshold = 0.85
        touchdown_ground_xy_tolerance = 1.35
        log_file = '/tmp/obdm_aruco_landing_target.log'
        detector_log_file = '/tmp/obdm_aruco_camera_detector.log'
        detection_mode = 'camera'
        use_realtime_trajectory_generator = true
    end

    methods
        function obj = MovingLandingTargetModule()
            obj@ModuleBase('MovingLandingTargetModule');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            if isfield(config, 'namespace'), obj.namespace = config.namespace; end
            if isfield(config, 'worldName'), obj.world_name = config.worldName; end
            if isfield(config, 'targetModelName'), obj.target_model_name = config.targetModelName; end
            if isfield(config, 'targetMarkerZ'), obj.target_marker_z = config.targetMarkerZ; end
            if isfield(config, 'targetModelZ'), obj.target_model_z = config.targetModelZ; end
            if isfield(config, 'targetSpawnForwardM'), obj.target_spawn_forward_m = config.targetSpawnForwardM; end
            if isfield(config, 'targetMotionRangeM'), obj.target_motion_range_m = config.targetMotionRangeM; end
            if isfield(config, 'targetMinForwardOffsetM'), obj.target_min_forward_offset_m = config.targetMinForwardOffsetM; end
            if isfield(config, 'targetMaxForwardOffsetM'), obj.target_max_forward_offset_m = config.targetMaxForwardOffsetM; end
            if isfield(config, 'targetInitialX'), obj.target_initial_x = config.targetInitialX; end
            if isfield(config, 'targetInitialY'), obj.target_initial_y = config.targetInitialY; end
            if isfield(config, 'targetInitialYawRad'), obj.target_initial_yaw_rad = config.targetInitialYawRad; end
            if isfield(config, 'targetMotionMode'), obj.target_motion_mode = config.targetMotionMode; end
            if isfield(config, 'targetCircleRadiusM'), obj.target_circle_radius_m = config.targetCircleRadiusM; end
            if isfield(config, 'targetCircleAngularSpeed'), obj.target_circle_angular_speed = config.targetCircleAngularSpeed; end
            if isfield(config, 'targetRandomMinSpeed'), obj.target_random_min_speed = config.targetRandomMinSpeed; end
            if isfield(config, 'targetRandomMaxSpeed'), obj.target_random_max_speed = config.targetRandomMaxSpeed; end
            if isfield(config, 'targetRandomYawrateMax'), obj.target_random_yawrate_max = config.targetRandomYawrateMax; end
            if isfield(config, 'targetRandomChangePeriod'), obj.target_random_change_period = config.targetRandomChangePeriod; end
            if isfield(config, 'targetRandomLateralRangeM'), obj.target_random_lateral_range_m = config.targetRandomLateralRangeM; end
            if isfield(config, 'waitForTakeoff'), obj.wait_for_takeoff = logical(config.waitForTakeoff); end
            if isfield(config, 'waitForMotionEnable'), obj.wait_for_motion_enable = logical(config.waitForMotionEnable); end
            if isfield(config, 'motionEnableTopic'), obj.motion_enable_topic = config.motionEnableTopic; end
            if isfield(config, 'takeoffZThreshold'), obj.takeoff_z_threshold = config.takeoffZThreshold; end
            if isfield(config, 'contactArmingZThreshold'), obj.contact_arming_z_threshold = config.contactArmingZThreshold; end
            if isfield(config, 'guidanceTakeoffAltitude'), obj.guidance_takeoff_altitude = config.guidanceTakeoffAltitude; end
            if isfield(config, 'touchdownZThreshold'), obj.touchdown_z_threshold = config.touchdownZThreshold; end
            if isfield(config, 'touchdownXYTolerance'), obj.touchdown_xy_tolerance = config.touchdownXYTolerance; end
            if isfield(config, 'touchdownMaxAbsVz'), obj.touchdown_max_abs_vz = config.touchdownMaxAbsVz; end
            if isfield(config, 'touchdownGroundZThreshold'), obj.touchdown_ground_z_threshold = config.touchdownGroundZThreshold; end
            if isfield(config, 'touchdownGroundXYTolerance'), obj.touchdown_ground_xy_tolerance = config.touchdownGroundXYTolerance; end
            mode = getenv('OBDM_ARUCO_DETECTION_MODE');
            if ~isempty(mode), obj.detection_mode = lower(strtrim(mode)); end
            if isfield(config, 'useRealtimeTrajectoryGenerator')
                obj.use_realtime_trajectory_generator = logical(config.useRealtimeTrajectoryGenerator);
            end
            realtimeEnv = getenv('OBDM_ENABLE_REALTIME_TRAJECTORY_GENERATOR');
            if ~isempty(realtimeEnv)
                obj.use_realtime_trajectory_generator = ~(strcmpi(strtrim(realtimeEnv), '0') || ...
                    strcmpi(strtrim(realtimeEnv), 'false'));
            end
        end

        function start(obj)
            repo_root = SimulationConfig.get_repo_root();
            script_path = fullfile(repo_root, 'user_stack', 'perception', 'aruco', 'aruco_landing_target_node.py');
            ardu_ws = SimulationConfig.get_ardu_ws_path();
            if strcmpi(obj.detection_mode, 'synthetic')
                marker_odom_topic = '/landing_marker/odom';
                marker_pose_topic = '/landing_marker/pose';
                marker_visible_topic = '/landing_marker/visible';
            else
                marker_odom_topic = '/landing_marker/ground_truth/odom';
                marker_pose_topic = '/landing_marker/ground_truth/pose';
                marker_visible_topic = '/landing_marker/ground_truth/visible';
            end
            cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
                'setsid python3 "%s" --namespace "%s" --world "%s" --model-name "%s" --marker-z "%.3f" --model-z "%.3f" ' ...
                '--initial-x "%.6f" --initial-y "%.6f" --initial-yaw-rad "%.6f" ' ...
                '--motion-mode "%s" --circle-radius-m "%.3f" --circle-angular-speed "%.3f" ' ...
                '--random-min-speed "%.3f" --random-max-speed "%.3f" --random-yawrate-max "%.3f" ' ...
                '--random-change-period "%.3f" --random-lateral-range-m "%.3f" ' ...
                '--spawn-forward-m "%.3f" --motion-range-m "%.3f" --min-forward-offset-m "%.3f" --max-forward-offset-m "%.3f" --update-gazebo ' ...
                '--gz-rate 5.0 --gz-service-timeout-ms 5000 --gz-log-period 10 ' ...
                '--takeoff-z-threshold "%.3f" --contact-arming-z-threshold "%.3f" ' ...
                '--touchdown-z-threshold "%.3f" --touchdown-xy-tolerance "%.3f" --touchdown-max-abs-vz "%.3f" ' ...
                '--touchdown-ground-z-threshold "%.3f" --touchdown-ground-xy-tolerance "%.3f" %s ' ...
                '%s --motion-enable-topic "%s" ' ...
                '--drone-odom-topic "%s" ' ...
                '--marker-odom-topic "%s" --marker-pose-topic "%s" --marker-visible-topic "%s" ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                ardu_ws, ardu_ws, script_path, obj.namespace, obj.world_name, obj.target_model_name, obj.target_marker_z, obj.target_model_z, ...
                obj.target_initial_x, obj.target_initial_y, obj.target_initial_yaw_rad, ...
                obj.target_motion_mode, obj.target_circle_radius_m, obj.target_circle_angular_speed, ...
                obj.target_random_min_speed, obj.target_random_max_speed, obj.target_random_yawrate_max, ...
                obj.target_random_change_period, obj.target_random_lateral_range_m, ...
                obj.target_spawn_forward_m, obj.target_motion_range_m, obj.target_min_forward_offset_m, obj.target_max_forward_offset_m, ...
                obj.takeoff_z_threshold, obj.contact_arming_z_threshold, ...
                obj.touchdown_z_threshold, obj.touchdown_xy_tolerance, obj.touchdown_max_abs_vz, ...
                obj.touchdown_ground_z_threshold, obj.touchdown_ground_xy_tolerance, obj.wait_for_takeoff_arg(), ...
                obj.wait_for_motion_enable_arg(), obj.motion_enable_topic, ...
                SimulationConfig.resolve_topic('odometry', obj.namespace), ...
                marker_odom_topic, marker_pose_topic, marker_visible_topic, obj.log_file);
            [status, output] = system(cmd);
            if status == 0
                obj.pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] started (PID: %s, log: %s)\n', obj.pid, obj.log_file);
            else
                fprintf('[MovingLandingTargetModule] start failed: %s\n', strtrim(output));
            end

            obj.ensure_rover_wheel_bridge(repo_root, ardu_ws);
            obj.start_rviz_tf_publisher(repo_root, ardu_ws);
            obj.start_guidance_streamer(repo_root, ardu_ws);
            if obj.use_realtime_trajectory_generator
                obj.start_realtime_trajectory_generator(repo_root, ardu_ws);
            else
                fprintf('[MovingLandingTargetModule] realtime trajectory generator disabled by configuration\n');
            end
            obj.start_guidance_path_visualizer(repo_root, ardu_ws);
            obj.start_wind_visualization(repo_root, ardu_ws);
            obj.configure_gazebo_follow_camera(ardu_ws);

            if ~strcmpi(obj.detection_mode, 'synthetic')
                obj.command_gimbal_down();
                obj.start_camera_bridge(ardu_ws);
                obj.set_camera_image_rate();
                obj.start_camera_detector(repo_root, ardu_ws);
                visual_ready = obj.wait_for_visual_topics(ardu_ws);
                if ~visual_ready
                    error('MovingLandingTargetModule:VisualTopicsNotReady', ...
                        'Camera image, camera_info, or odometry topics were not ready; holding takeoff instead of missing early frames.');
                end
            else
                fprintf('[MovingLandingTargetModule] using synthetic marker detection topics\n');
            end
            obj.start_rviz(repo_root, ardu_ws);
        end

        function configure_gazebo_follow_camera(~, ardu_ws)
            if strcmpi(getenv('OBDM_GZ_FOLLOW_CAMERA'), '0') || strcmpi(getenv('OBDM_GZ_FOLLOW_CAMERA'), 'false')
                return;
            end
            try
                cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
                    'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                    'export GZ_VERSION=harmonic; ' ...
                    ': > /tmp/obdm_gazebo_follow_camera.log; ' ...
                    'echo "[follow-camera] services:" >>/tmp/obdm_gazebo_follow_camera.log; ' ...
                    'gz service -l 2>/dev/null | grep -E "/gui/(follow|follow/offset|camera|move_to)" >>/tmp/obdm_gazebo_follow_camera.log || true; ' ...
                    'for i in $(seq 1 20); do ' ...
                    'echo "[follow-camera] attempt $i target=iris" >>/tmp/obdm_gazebo_follow_camera.log; ' ...
                    'gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean ' ...
                    '--timeout 700 --req "data: \\"iris\\"" >>/tmp/obdm_gazebo_follow_camera.log 2>&1 && break; ' ...
                    'echo "[follow-camera] attempt $i target=/iris" >>/tmp/obdm_gazebo_follow_camera.log; ' ...
                    'gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean ' ...
                    '--timeout 700 --req "data: \\"/iris\\"" >>/tmp/obdm_gazebo_follow_camera.log 2>&1 && break; ' ...
                    'gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean ' ...
                    '--timeout 700 --req "data: \\"model://iris\\"" >>/tmp/obdm_gazebo_follow_camera.log 2>&1 && break; ' ...
                    'sleep 0.5; ' ...
                    'done; ' ...
                    'gz service -s /gui/follow/offset --reqtype gz.msgs.Vector3d --reptype gz.msgs.Boolean ' ...
                    '--timeout 700 --req "x: -6.0 y: 0.0 z: 3.2" >>/tmp/obdm_gazebo_follow_camera.log 2>&1 || true; ' ...
                    'echo "[follow-camera] camera topics:" >>/tmp/obdm_gazebo_follow_camera.log; ' ...
                    'gz topic -i -t /gui/camera/pose >>/tmp/obdm_gazebo_follow_camera.log 2>&1 || true; ' ...
                    'gz topic -i -t /gui/camera/view_control >>/tmp/obdm_gazebo_follow_camera.log 2>&1 || true'''], ...
                    ardu_ws, ardu_ws);
                system(cmd);
            catch
            end
        end

        function stop(obj)
            obj.stop_rover_motion();
            if ~isempty(obj.detector_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.detector_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.detector_pid));
                fprintf('[MovingLandingTargetModule] camera detector stopped (PID: %s)\n', obj.detector_pid);
                obj.detector_pid = '';
            end
            if ~isempty(obj.camera_bridge_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.camera_bridge_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.camera_bridge_pid));
                fprintf('[MovingLandingTargetModule] camera bridge stopped (PID: %s)\n', obj.camera_bridge_pid);
                obj.camera_bridge_pid = '';
            end
            if ~isempty(obj.guidance_streamer_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.guidance_streamer_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.guidance_streamer_pid));
                fprintf('[MovingLandingTargetModule] guidance streamer stopped (PID: %s)\n', obj.guidance_streamer_pid);
                obj.guidance_streamer_pid = '';
            end
            if ~isempty(obj.trajectory_generator_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.trajectory_generator_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.trajectory_generator_pid));
                fprintf('[MovingLandingTargetModule] realtime trajectory generator stopped (PID: %s)\n', obj.trajectory_generator_pid);
                obj.trajectory_generator_pid = '';
            end
            if ~isempty(obj.guidance_path_visualizer_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.guidance_path_visualizer_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.guidance_path_visualizer_pid));
                fprintf('[MovingLandingTargetModule] guidance path visualizer stopped (PID: %s)\n', obj.guidance_path_visualizer_pid);
                obj.guidance_path_visualizer_pid = '';
            end
            if ~isempty(obj.rviz_tf_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.rviz_tf_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.rviz_tf_pid));
                fprintf('[MovingLandingTargetModule] RViz TF publisher stopped (PID: %s)\n', obj.rviz_tf_pid);
                obj.rviz_tf_pid = '';
            end
            if ~isempty(obj.rviz_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.rviz_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.rviz_pid));
                fprintf('[MovingLandingTargetModule] RViz stopped (PID: %s)\n', obj.rviz_pid);
                obj.rviz_pid = '';
            end
            if ~isempty(obj.wind_arrow_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.wind_arrow_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.wind_arrow_pid));
                fprintf('[MovingLandingTargetModule] wind visualizer stopped (PID: %s)\n', obj.wind_arrow_pid);
                obj.wind_arrow_pid = '';
            end
            if ~isempty(obj.virtual_wind_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.virtual_wind_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.virtual_wind_pid));
                fprintf('[MovingLandingTargetModule] virtual wind sensor stopped (PID: %s)\n', obj.virtual_wind_pid);
                obj.virtual_wind_pid = '';
            end
            if ~isempty(obj.rover_bridge_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.rover_bridge_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.rover_bridge_pid));
                fprintf('[MovingLandingTargetModule] rover wheel bridge stopped (PID: %s)\n', obj.rover_bridge_pid);
                obj.rover_bridge_pid = '';
            end
            if ~isempty(obj.pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.pid));
                fprintf('[MovingLandingTargetModule] stopped (PID: %s)\n', obj.pid);
                obj.pid = '';
            end
        end

        function arg = wait_for_takeoff_arg(obj)
            if obj.wait_for_takeoff
                arg = '--wait-for-takeoff';
            else
                arg = '--no-wait-for-takeoff';
            end
        end

        function arg = wait_for_motion_enable_arg(obj)
            if obj.wait_for_motion_enable
                arg = '--wait-for-motion-enable';
            else
                arg = '--no-wait-for-motion-enable';
            end
        end

        function release_motion(obj, ardu_ws)
            if nargin < 2 || isempty(ardu_ws)
                ardu_ws = SimulationConfig.get_ardu_ws_path();
            end
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; timeout 4 ros2 topic pub --once "%s" std_msgs/msg/Bool "{data: true}" ' ...
                '--qos-reliability reliable --qos-durability transient_local >/tmp/obdm_rover_motion_release.log 2>&1'''], ...
                obj.clean_env_command_prefix(), ardu_ws, ardu_ws, obj.motion_enable_topic);
            [status, ~] = system(cmd);
            if status == 0 || status == 124
                fprintf('[MovingLandingTargetModule] rover motion release published on %s\n', obj.motion_enable_topic);
            else
                fprintf('[MovingLandingTargetModule] rover motion release failed; check /tmp/obdm_rover_motion_release.log\n');
            end
        end

        function stop_rover_motion(~)
            try
                cmd = ['bash -lc ''source /opt/ros/humble/setup.bash 2>/dev/null || true; ' ...
                    'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
                    'ros2 topic pub --once /model/r1_rover/cmd_vel geometry_msgs/msg/Twist ' ...
                    '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true; ' ...
                    'for t in /model/r1_rover/joint/motor_0/cmd_force /model/r1_rover/joint/motor_1/cmd_force /model/r1_rover/joint/motor_2/cmd_force /model/r1_rover/joint/motor_3/cmd_force; do ' ...
                    'gz topic -t "$t" -m gz.msgs.Double -p "data: 0.0" >/dev/null 2>&1 || true; done'''];
                system(cmd);
            catch
            end
        end

        function cleanup(obj)
            obj.stop();
        end

        function start_camera_bridge(obj, ardu_ws)
            ns = obj.namespace;
            if ~startsWith(ns, '/')
                ns = ['/', ns];
            end
            ns = strip(ns, 'right', '/');
            gz_odom_topic = '/model/iris/odometry';
            gz_imu_topic = sprintf('/world/%s/model/iris/link/imu_link/sensor/imu_sensor/imu', obj.world_name);
            gz_navsat_topic = sprintf('/world/%s/model/iris/link/base_link/sensor/navsat_sensor/navsat', obj.world_name);
            gz_battery_topic = '/model/iris/battery/lipo_3500mAh/state';
            ros_image_topic = sprintf('%s/camera/image', ns);
            ros_info_topic = sprintf('%s/camera/camera_info', ns);
            ros_odom_topic = sprintf('%s/odometry', ns);
            ros_imu_topic = sprintf('%s/imu', ns);
            ros_navsat_topic = sprintf('%s/navsat/navsat0', ns);
            ros_battery_topic = sprintf('%s/battery', ns);

            if obj.ros_topic_has_publisher(ros_image_topic, ardu_ws) && ...
                    obj.ros_topic_has_publisher(ros_info_topic, ardu_ws) && ...
                    obj.ros_topic_has_publisher(ros_odom_topic, ardu_ws)
                fprintf('[MovingLandingTargetModule] camera/odom bridge already active for %s\n', ns);
                return;
            end

            topic_cmd = ['timeout 3 gz topic -l 2>/dev/null | ' ...
                'grep "/world/.*/model/iris.*/link/pitch_link/sensor/camera/image" | head -1'];
            [topic_status, topic_output] = system(topic_cmd);
            gz_image_topic = strtrim(topic_output);
            if topic_status == 0 && ~isempty(gz_image_topic)
                gz_info_topic = strrep(gz_image_topic, '/image', '/camera_info');
                fprintf('[MovingLandingTargetModule] using gimbal camera topic: %s\n', gz_image_topic);
            else
                error('Original gimbal camera topic not found');
            end

            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
                'setsid ros2 run ros_gz_bridge parameter_bridge ' ...
                '"%s@sensor_msgs/msg/Image[gz.msgs.Image" ' ...
                '"%s@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" ' ...
                '"%s@nav_msgs/msg/Odometry[gz.msgs.Odometry" ' ...
                '"%s@sensor_msgs/msg/Imu[gz.msgs.IMU" ' ...
                '"%s@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat" ' ...
                '"%s@sensor_msgs/msg/BatteryState[gz.msgs.BatteryState" ' ...
                '--ros-args -r "%s:=%s" -r "%s:=%s" -r "%s:=%s" ' ...
                '-r "%s:=%s" -r "%s:=%s" -r "%s:=%s" ' ...
                '> /tmp/obdm_camera_bridge.log 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ...
                ardu_ws, ardu_ws, ...
                gz_image_topic, gz_info_topic, gz_odom_topic, gz_imu_topic, gz_navsat_topic, gz_battery_topic, ...
                gz_image_topic, ros_image_topic, gz_info_topic, ros_info_topic, ...
                gz_odom_topic, ros_odom_topic, gz_imu_topic, ros_imu_topic, ...
                gz_navsat_topic, ros_navsat_topic, gz_battery_topic, ros_battery_topic);
            [status, output] = system(cmd);
            if status == 0
                obj.camera_bridge_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] camera bridge started (PID: %s, log: /tmp/obdm_camera_bridge.log)\n', obj.camera_bridge_pid);
            else
                fprintf('[MovingLandingTargetModule] camera bridge start failed: %s\n', strtrim(output));
            end
            pause(1.0);
        end

        function set_camera_image_rate(obj)
            try
                cmd = sprintf(['bash -lc ''export GZ_VERSION=harmonic; ' ...
                    'svc="/world/%s/model/iris/link/pitch_link/sensor/camera/image/set_rate"; ' ...
                    'gz service -s "$svc" --reqtype gz.msgs.Double --reptype gz.msgs.Empty ' ...
                    '--timeout 1200 --req "data: 10.0" >/tmp/obdm_camera_rate.log 2>&1 || true'''], ...
                    obj.world_name);
                system(cmd);
                fprintf('[MovingLandingTargetModule] requested Gazebo camera image rate 10 Hz (log: /tmp/obdm_camera_rate.log)\n');
            catch
            end
        end

        function has_pub = ros_topic_has_publisher(~, topic_name, ardu_ws)
            cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; ' ...
                'ros2 topic info "%s" -v 2>/dev/null | grep -c "Endpoint type: PUBLISHER"'''], ...
                ardu_ws, ardu_ws, topic_name);
            [status, output] = system(cmd);
            count = str2double(strtrim(output));
            has_pub = status == 0 && ~isnan(count) && count > 0;
        end

        function start_camera_detector(obj, repo_root, ardu_ws)
            detector_path = fullfile(repo_root, 'user_stack', 'perception', 'aruco', 'aruco_camera_detector_node.py');
            if obj.ros_topic_has_publisher('/landing_marker/annotated_image', ardu_ws)
                fprintf('[MovingLandingTargetModule] camera detector already active\n');
                return;
            end
            fov_fallback_arg = '--enable-fov-fallback ';
            if strcmpi(getenv('OBDM_ENABLE_ARUCO_FOV_FALLBACK'), '0')
                fov_fallback_arg = '';
            elseif strcmpi(getenv('OBDM_ENABLE_ARUCO_FOV_FALLBACK'), '1')
                fov_fallback_arg = '--enable-fov-fallback ';
            end
            cmd = sprintf(['%s bash -lc ''unset LD_LIBRARY_PATH; unset QT_PLUGIN_PATH; unset QML2_IMPORT_PATH; ' ...
                'unset QT_QPA_PLATFORM_PLUGIN_PATH; unset QT_STYLE_OVERRIDE; ' ...
                'source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --namespace "%s" --marker-size-m 1.0 --marker-id 0 ' ...
                '--marker-sizes "0:0.42,1:0.20,2:0.20,3:0.20,4:0.20,5:0.12,6:0.12,7:0.12,8:0.12" ' ...
                '--marker-center-offsets "0:0.000:0.000,1:0.420:0.000,2:-0.420:0.000,3:0.000:-0.420,4:0.000:0.420,5:0.471:-0.471,6:-0.471:-0.471,7:0.471:0.471,8:-0.471:0.471" ' ...
                '--odom-topic "%s" --visible-timeout-s 5.0 --marker-ground-z "%.3f" ' ...
                '--camera-pitch-down-deg 90 --camera-roll-deg 0 %s' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ...
                ardu_ws, ardu_ws, detector_path, obj.namespace, ...
                SimulationConfig.resolve_topic('odometry', obj.namespace), ...
                obj.target_marker_z, fov_fallback_arg, obj.detector_log_file);
            [status, output] = system(cmd);
            if status == 0
                obj.detector_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] camera detector started (PID: %s, log: %s)\n', ...
                    obj.detector_pid, obj.detector_log_file);
            else
                fprintf('[MovingLandingTargetModule] camera detector start failed: %s\n', strtrim(output));
            end
        end

        function start_guidance_streamer(obj, repo_root, ardu_ws)
            streamer_path = fullfile(repo_root, 'bridges', 'mavlink', 'mavlink_guidance_streamer.py');
            if ~exist(streamer_path, 'file')
                fprintf('[MovingLandingTargetModule] guidance streamer script not found: %s\n', streamer_path);
                return;
            end
            check_cmd = sprintf('ps -eo args | grep "[m]avlink_guidance_streamer.py" >/dev/null 2>&1');
            [active, ~] = system(check_cmd);
            if active == 0
                fprintf('[MovingLandingTargetModule] guidance streamer already active\n');
                return;
            end
            log_file_streamer = '/tmp/obdm_mavlink_guidance_streamer.log';
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --rate 30.0 --target-timeout 10.0 --tracking-alpha 0.35 --z-tracking-alpha 1.00 ' ...
                '--no-hold-last-target --descent-deadband 0.03 --min-descent-rate 0.15 --max-descent-rate 0.75 ' ...
                '--min-climb-rate 0.35 --max-climb-rate 1.40 --climb-target-lookahead 2.20 --climb-deadband 0.08 ' ...
                '--no-auto-takeoff-on-climb --auto-takeoff-max-altitude 0.80 --auto-takeoff-target-altitude "%.3f" ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ...
                ardu_ws, ardu_ws, streamer_path, obj.guidance_takeoff_altitude, log_file_streamer);
            [status, output] = system(cmd);
            if status == 0
                obj.guidance_streamer_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] guidance streamer started (PID: %s, log: %s)\n', ...
                    obj.guidance_streamer_pid, log_file_streamer);
            else
                fprintf('[MovingLandingTargetModule] guidance streamer start failed: %s\n', strtrim(output));
            end
            pause(0.5);
        end

        function start_wind_visualization(obj, repo_root, ardu_ws)
            obj.start_virtual_wind_sensor(repo_root, ardu_ws);
            obj.start_wind_arrow_visualizer(repo_root, ardu_ws);
            obj.verify_wind_topics(ardu_ws);
        end

        function start_virtual_wind_sensor(obj, repo_root, ardu_ws)
            sensor_path = fullfile(repo_root, 'bridges', 'ros_gz', 'gz_virtual_wind_sensor_to_ros2.py');
            if ~exist(sensor_path, 'file')
                fprintf('[MovingLandingTargetModule] virtual wind sensor script not found: %s\n', sensor_path);
                return;
            end
            if obj.ros_topic_has_publisher(sprintf('%s/wind_sensor', obj.normalized_namespace()), ardu_ws)
                fprintf('[MovingLandingTargetModule] virtual wind sensor already active\n');
                return;
            end
            log_file_wind = '/tmp/obdm_virtual_wind_sensor.log';
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --namespace "%s" --world "%s" --robot "iris" --rate 5.0 ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ardu_ws, ardu_ws, sensor_path, ...
                obj.namespace, obj.world_name, log_file_wind);
            [status, output] = system(cmd);
            if status == 0
                obj.virtual_wind_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] virtual wind sensor started (PID: %s, log: %s)\n', ...
                    obj.virtual_wind_pid, log_file_wind);
            else
                fprintf('[MovingLandingTargetModule] virtual wind sensor start failed: %s\n', strtrim(output));
            end
            pause(0.5);
        end

        function start_wind_arrow_visualizer(obj, repo_root, ardu_ws)
            marker_path = fullfile(repo_root, 'simulator', 'visualization', 'wind_arrow_marker_node.py');
            if ~exist(marker_path, 'file')
                fprintf('[MovingLandingTargetModule] wind visualizer script not found: %s\n', marker_path);
                return;
            end
            if obj.ros_topic_has_publisher(sprintf('%s/wind_arrow', obj.normalized_namespace()), ardu_ws)
                fprintf('[MovingLandingTargetModule] wind visualizer already active\n');
                return;
            end
            log_file_arrow = '/tmp/obdm_wind_visualizer.log';
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --namespace "%s" --scale 0.6 --max-len 3.0 --default-frame "iris/base_link" ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ardu_ws, ardu_ws, marker_path, ...
                obj.namespace, log_file_arrow);
            [status, output] = system(cmd);
            if status == 0
                obj.wind_arrow_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] wind visualizer started (PID: %s, log: %s)\n', ...
                    obj.wind_arrow_pid, log_file_arrow);
            else
                fprintf('[MovingLandingTargetModule] wind visualizer start failed: %s\n', strtrim(output));
            end
            pause(0.5);
        end

        function verify_wind_topics(obj, ardu_ws)
            ns = obj.normalized_namespace();
            sensor_topic = sprintf('%s/wind_sensor', ns);
            arrow_topic = sprintf('%s/wind_arrow', ns);
            sensor_ok = obj.wait_for_topic_message(sensor_topic, ardu_ws, 8);
            arrow_ok = obj.wait_for_topic_message(arrow_topic, ardu_ws, 8);
            if sensor_ok && arrow_ok
                fprintf('[MovingLandingTargetModule] wind RViz topics ready: %s, %s\n', sensor_topic, arrow_topic);
            else
                fprintf('[MovingLandingTargetModule] wind RViz topics not fully ready; check /tmp/obdm_virtual_wind_sensor.log and /tmp/obdm_wind_visualizer.log\n');
            end
        end

        function ns = normalized_namespace(obj)
            ns = obj.namespace;
            if ~startsWith(ns, '/')
                ns = ['/', ns];
            end
            ns = strip(ns, 'right', '/');
        end

        function start_realtime_trajectory_generator(obj, repo_root, ardu_ws)
            generator_path = fullfile(repo_root, 'user_stack', 'guidance', 'ontology_landing', 'realtime_ontology_trajectory_generator.py');
            if ~exist(generator_path, 'file')
                fprintf('[MovingLandingTargetModule] realtime trajectory generator script not found: %s\n', generator_path);
                return;
            end
            check_cmd = sprintf('ps -eo args | grep "[r]ealtime_ontology_trajectory_generator.py" >/dev/null 2>&1');
            [active, ~] = system(check_cmd);
            if active == 0
                fprintf('[MovingLandingTargetModule] realtime trajectory generator already active\n');
                return;
            end
            log_file_generator = '/tmp/obdm_realtime_ontology_trajectory_generator.log';
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --odom-topic "%s" --marker-topic "/landing_marker/odom" ' ...
                '--visible-topic "/landing_marker/visible" --origin-x 0.0 --origin-y 0.0 --origin-z-offset 0.0 ' ...
                '--takeoff-altitude "%.3f" --search-altitude "%.3f" ' ...
                '--landing-start-altitude "%.3f" --touchdown-altitude "%.3f" ' ...
                '--touchdown-height-above-pad "%.3f" ' ...
                '--marker-lost-timeout 1.0 --marker-lost-climb-rate 1.20 --marker-hold-timeout 6.50 ' ...
                '--rate 10.0 --descent-rate 0.35 --max-descent-rate-near 0.60 --near-descent-boost 1.45 ' ...
                '--far-descent-scale 0.35 --max-step 3.80 --step-gain 0.62 --fast-track-radius 0.65 ' ...
                '--fast-track-gain 0.70 --fast-track-extra-step 0.90 --xy-smoothing 0.45 ' ...
                '--min-lead-time 1.20 --lead-distance-gain 0.45 --max-lead-time 4.50 ' ...
                '--prediction-steps 8 --prediction-step-time 0.35 --prediction-horizon 6.00 --future-point-blend 0.55 ' ...
                '--direct-land-radius 0.35 --direct-land-altitude-above-pad 0.45 ' ...
                '--direct-land-stable-time 0.40 --direct-land-max-horizontal-speed 0.80 ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ardu_ws, ardu_ws, generator_path, ...
                SimulationConfig.resolve_topic('odometry', obj.namespace), ...
                obj.guidance_takeoff_altitude, obj.guidance_takeoff_altitude + 2.0, ...
                0.80 * obj.guidance_takeoff_altitude, ...
                obj.touchdown_z_threshold, max(0.0, obj.touchdown_z_threshold - obj.target_marker_z), ...
                log_file_generator);
            [status, output] = system(cmd);
            if status == 0
                obj.trajectory_generator_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] realtime trajectory generator started (PID: %s, log: %s)\n', ...
                    obj.trajectory_generator_pid, log_file_generator);
            else
                fprintf('[MovingLandingTargetModule] realtime trajectory generator start failed: %s\n', strtrim(output));
            end
            pause(0.5);
        end

        function start_guidance_path_visualizer(obj, repo_root, ardu_ws)
            visualizer_path = fullfile(repo_root, 'simulator', 'visualization', 'guidance_path_visualizer.py');
            if ~exist(visualizer_path, 'file')
                fprintf('[MovingLandingTargetModule] guidance path visualizer script not found: %s\n', visualizer_path);
                return;
            end
            if obj.ros_topic_has_publisher('/obdm/generated_trajectory_path', ardu_ws)
                fprintf('[MovingLandingTargetModule] guidance path visualizer already active\n');
                return;
            end
            log_file_visualizer = '/tmp/obdm_guidance_path_visualizer.log';
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --odom-topic "%s" --takeoff-altitude "%.3f" ' ...
                '--marker-topic "/landing_marker/ground_truth/odom" ' ...
                '--topdown-image-topic "/obdm/topdown_projection_image" ' ...
                '--no-current-generated-only --max-generated-points 450 --min-point-spacing 0.03 ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ardu_ws, ardu_ws, visualizer_path, ...
                SimulationConfig.resolve_topic('odometry', obj.namespace), obj.guidance_takeoff_altitude, log_file_visualizer);
            [status, output] = system(cmd);
            if status == 0
                obj.guidance_path_visualizer_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] guidance path visualizer started (PID: %s, log: %s)\n', ...
                    obj.guidance_path_visualizer_pid, log_file_visualizer);
            else
                fprintf('[MovingLandingTargetModule] guidance path visualizer start failed: %s\n', strtrim(output));
            end
            pause(0.5);
        end

        function start_rviz_tf_publisher(obj, repo_root, ardu_ws)
            tf_path = fullfile(repo_root, 'simulator', 'visualization', 'iris_rviz_tf_publisher.py');
            if ~exist(tf_path, 'file')
                fprintf('[MovingLandingTargetModule] RViz TF publisher script not found: %s\n', tf_path);
                return;
            end
            check_cmd = ['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; ' ...
                'timeout 3 ros2 topic echo /tf tf2_msgs/msg/TFMessage --once 2>/dev/null | grep -q "child_frame_id: base_link"'''];
            [has_tf, ~] = system(sprintf(check_cmd, obj.clean_env_command_prefix(), ardu_ws, ardu_ws));
            if has_tf == 0
                fprintf('[MovingLandingTargetModule] RViz odom->base_link TF already active\n');
                return;
            end
            tf_log = '/tmp/obdm_iris_rviz_tf_publisher.log';
            cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export PYTHONUNBUFFERED=1; ' ...
                'setsid python3 "%s" --odom-topic "%s" --fixed-frame odom > "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ...
                ardu_ws, ardu_ws, tf_path, SimulationConfig.resolve_topic('odometry', obj.namespace), tf_log);
            [status, output] = system(cmd);
            if status == 0
                obj.rviz_tf_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] RViz TF publisher started (PID: %s, log: %s)\n', ...
                    obj.rviz_tf_pid, tf_log);
            else
                fprintf('[MovingLandingTargetModule] RViz TF publisher start failed: %s\n', strtrim(output));
            end
            pause(0.5);
        end

        function command_gimbal_down(~)
            commands = { ...
                '/gimbal/cmd_roll', 0.0; ...
                '/gimbal/cmd_pitch', 0.0; ...
                '/gimbal/cmd_yaw', 0.0};
            for i = 1:size(commands, 1)
                try
                    system(sprintf('gz topic -t "%s" -m gz.msgs.Double -p "data: %.8f" >/dev/null 2>&1', ...
                        commands{i, 1}, commands{i, 2}));
                catch
                end
            end
            fprintf('[MovingLandingTargetModule] gimbal commanded to down-looking optical attitude\n');
        end

        function ensure_rover_wheel_bridge(obj, repo_root, ardu_ws)
            try
                rover_bridge_path = fullfile(repo_root, 'simulator', 'models', 'rover', 'rover_cmd_vel_to_wheels.py');
                check_process_cmd = sprintf('ps -eo args | grep "[r]over_cmd_vel_to_wheels.py" | grep -F "%s" >/dev/null 2>&1', rover_bridge_path);
                [process_active, ~] = system(check_process_cmd);
                if process_active == 0
                    fprintf('[MovingLandingTargetModule] rover wheel bridge already active (process found)\n');
                    return;
                end

                check_cmd = ['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                    'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                    'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
                    'ros2 topic info /model/r1_rover/cmd_vel -v 2>/dev/null | grep -c "Endpoint type: SUBSCRIPTION"'''];
                [status, output] = system(sprintf(check_cmd, ardu_ws, ardu_ws));
                subscriber_count = str2double(strtrim(output));
                if status ~= 0 || isnan(subscriber_count)
                    subscriber_count = 0;
                end
                if subscriber_count > 0
                    fprintf('[MovingLandingTargetModule] rover cmd_vel has %d subscriber(s), but wheel bridge process was not found; starting bridge\n', subscriber_count);
                end
                if ~exist(rover_bridge_path, 'file')
                    fprintf('[MovingLandingTargetModule] rover wheel bridge script not found: %s\n', rover_bridge_path);
                    return;
                end

                rb_log = '/tmp/obdm_rover_cmd_vel_bridge.log';
                cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                    'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                    'export ROS_DOMAIN_ID=0; export GZ_VERSION=harmonic; ' ...
                    'setsid python3 "%s" > "%s" 2>&1 < /dev/null & echo $!'''], ...
                    obj.clean_env_command_prefix(), ...
                    ardu_ws, ardu_ws, rover_bridge_path, rb_log);
                [status2, output2] = system(cmd);
                if status2 == 0
                    obj.rover_bridge_pid = strtrim(output2);
                    fprintf('[MovingLandingTargetModule] rover wheel bridge started (PID: %s, log: %s)\n', obj.rover_bridge_pid, rb_log);
                else
                    fprintf('[MovingLandingTargetModule] rover wheel bridge start failed: %s\n', strtrim(output2));
                end
            catch ME
                fprintf('[MovingLandingTargetModule] rover wheel bridge check failed: %s\n', ME.message);
            end
        end

        function ok = wait_for_visual_topics(obj, ardu_ws)
            ns = obj.namespace;
            if ~startsWith(ns, '/')
                ns = ['/', ns];
            end
            ns = strip(ns, 'right', '/');
            raw_topic = sprintf('%s/camera/image', ns);
            info_topic = sprintf('%s/camera/camera_info', ns);
            odom_topic = SimulationConfig.resolve_topic('odometry', obj.namespace);
            annotated_topic = '/landing_marker/annotated_image';
            raw_ok = obj.wait_for_topic_message(raw_topic, ardu_ws, 24);
            info_ok = obj.wait_for_topic_message(info_topic, ardu_ws, 16);
            odom_ok = obj.wait_for_topic_message(odom_topic, ardu_ws, 8);
            annotated_ok = obj.wait_for_topic_message(annotated_topic, ardu_ws, 8);
            if ~annotated_ok
                fprintf('[MovingLandingTargetModule] annotated image not ready yet; detector may publish it after first processed frame\n');
            end
            ok = raw_ok && info_ok && odom_ok;
        end

        function ok = wait_for_topic_message(obj, topic_name, ardu_ws, timeout_s)
            ok = false;
            t0 = tic;
            while toc(t0) < timeout_s
                cmd = sprintf(['%s bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                    'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                    'export ROS_DOMAIN_ID=0; ' ...
                    'timeout 3 ros2 topic echo "%s" "%s" --once --qos-reliability %s --qos-durability volatile >/dev/null 2>&1'''], ...
                    obj.clean_env_command_prefix(), ardu_ws, ardu_ws, topic_name, obj.topic_message_type(topic_name), ...
                    obj.topic_reliability(topic_name));
                [status, ~] = system(cmd);
                if status == 0
                    fprintf('[MovingLandingTargetModule] topic ready for RViz: %s\n', topic_name);
                    ok = true;
                    return;
                end
                pause(1.0);
            end
            fprintf('[MovingLandingTargetModule] topic not ready before RViz start: %s\n', topic_name);
        end

        function start_rviz(obj, repo_root, ardu_ws)
            if strcmpi(getenv('OBDM_DISABLE_RVIZ'), '1')
                fprintf('[MovingLandingTargetModule] RViz disabled by OBDM_DISABLE_RVIZ=1\n');
                return;
            end
            rviz_config = fullfile(repo_root, 'simulator', 'rviz', 'ruiz.rviz');
            if ~exist(rviz_config, 'file')
                fprintf('[MovingLandingTargetModule] RViz config not found: %s\n', rviz_config);
                return;
            end
            check_cmd = sprintf('ps -eo args | grep "[r]viz2" | grep -F "%s" >/dev/null 2>&1', rviz_config);
            [active, ~] = system(check_cmd);
            if active == 0
                fprintf('[MovingLandingTargetModule] RViz already active for %s\n', rviz_config);
                return;
            end
            rviz_log = '/tmp/obdm_rviz.log';
            cmd = sprintf(['%s bash -lc ''unset LD_LIBRARY_PATH; unset QT_PLUGIN_PATH; unset QML2_IMPORT_PATH; ' ...
                'unset QT_QPA_PLATFORM_PLUGIN_PATH; unset QT_STYLE_OVERRIDE; ' ...
                'source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; export QT_ACCESSIBILITY=0; export NO_AT_BRIDGE=1; ' ...
                'setsid rviz2 -d "%s" > "%s" 2>&1 < /dev/null & echo $!'''], ...
                obj.clean_env_command_prefix(), ardu_ws, ardu_ws, rviz_config, rviz_log);
            [status, output] = system(cmd);
            if status == 0
                obj.rviz_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] RViz started after visual topics (PID: %s, log: %s)\n', ...
                    obj.rviz_pid, rviz_log);
            else
                fprintf('[MovingLandingTargetModule] RViz start failed: %s\n', strtrim(output));
            end
        end

        function msg_type = topic_message_type(~, topic_name)
            name = lower(topic_name);
            if contains(name, 'image')
                msg_type = 'sensor_msgs/msg/Image';
            elseif contains(name, 'camera_info')
                msg_type = 'sensor_msgs/msg/CameraInfo';
            elseif contains(name, 'odometry') || contains(name, '/odom')
                msg_type = 'nav_msgs/msg/Odometry';
            elseif contains(name, 'visible')
                msg_type = 'std_msgs/msg/Bool';
            elseif contains(name, 'wind_sensor') || contains(name, 'wind_world')
                msg_type = 'geometry_msgs/msg/Vector3Stamped';
            elseif contains(name, 'wind_arrow')
                msg_type = 'visualization_msgs/msg/Marker';
            else
                msg_type = '';
            end
        end

        function reliability = topic_reliability(~, topic_name)
            name = lower(topic_name);
            if contains(name, 'camera')
                reliability = 'reliable';
            else
                reliability = 'best_effort';
            end
        end

        function prefix = clean_env_command_prefix(~)
            keys = {'HOME', 'USER', 'LOGNAME', 'SHELL', 'LANG', 'DISPLAY', ...
                'XAUTHORITY', 'XDG_RUNTIME_DIR', 'DBUS_SESSION_BUS_ADDRESS', ...
                'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI', ...
                'FASTRTPS_DEFAULT_PROFILES_FILE'};
            defaults = struct('SHELL', '/bin/bash', 'LANG', 'C.UTF-8');
            parts = {'env -i'};
            for i = 1:numel(keys)
                key = keys{i};
                value = getenv(key);
                if isempty(value) && isfield(defaults, key)
                    value = defaults.(key);
                end
                if ~isempty(value)
                    parts{end+1} = sprintf('%s="%s"', key, regexprep(value, '"', '\\"')); %#ok<AGROW>
                end
            end
            parts{end+1} = 'PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"';
            parts{end+1} = 'QT_ACCESSIBILITY=0';
            parts{end+1} = 'NO_AT_BRIDGE=1';
            parts{end+1} = 'XDG_DATA_DIRS="/usr/local/share:/usr/share"';
            parts{end+1} = 'XDG_CONFIG_DIRS="/etc/xdg"';
            if isempty(getenv('RMW_IMPLEMENTATION'))
                parts{end+1} = 'RMW_IMPLEMENTATION="rmw_fastrtps_cpp"';
            end
            if isempty(getenv('ROS_LOCALHOST_ONLY'))
                parts{end+1} = 'ROS_LOCALHOST_ONLY="0"';
            end
            prefix = strjoin(parts, ' ');
        end
    end
end
