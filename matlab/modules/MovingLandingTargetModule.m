classdef MovingLandingTargetModule < ModuleBase
    % MovingLandingTargetModule starts the ROS2/Gazebo moving ArUco target helper.

    properties
        pid = ''
        detector_pid = ''
        namespace = '/iris'
        world_name = 'runway'
        log_file = '/tmp/obdm_aruco_landing_target.log'
        detector_log_file = '/tmp/obdm_aruco_camera_detector.log'
        detection_mode = 'camera'
    end

    methods
        function obj = MovingLandingTargetModule()
            obj@ModuleBase('MovingLandingTargetModule');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            if isfield(config, 'namespace'), obj.namespace = config.namespace; end
            if isfield(config, 'worldName'), obj.world_name = config.worldName; end
            mode = getenv('OBDM_ARUCO_DETECTION_MODE');
            if ~isempty(mode), obj.detection_mode = lower(strtrim(mode)); end
        end

        function start(obj)
            repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            script_path = fullfile(repo_root, 'scripts', 'aruco_landing_target_node.py');
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
                'setsid python3 "%s" --namespace "%s" --world "%s" --update-gazebo ' ...
                '--marker-odom-topic "%s" --marker-pose-topic "%s" --marker-visible-topic "%s" ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                ardu_ws, ardu_ws, script_path, obj.namespace, obj.world_name, ...
                marker_odom_topic, marker_pose_topic, marker_visible_topic, obj.log_file);
            [status, output] = system(cmd);
            if status == 0
                obj.pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] started (PID: %s, log: %s)\n', obj.pid, obj.log_file);
            else
                fprintf('[MovingLandingTargetModule] start failed: %s\n', strtrim(output));
            end

            if ~strcmpi(obj.detection_mode, 'synthetic')
                obj.start_camera_detector(repo_root, ardu_ws);
            else
                fprintf('[MovingLandingTargetModule] using synthetic marker detection topics\n');
            end
        end

        function stop(obj)
            if ~isempty(obj.detector_pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.detector_pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.detector_pid));
                fprintf('[MovingLandingTargetModule] camera detector stopped (PID: %s)\n', obj.detector_pid);
                obj.detector_pid = '';
            end
            if ~isempty(obj.pid)
                system(sprintf('kill %s 2>/dev/null || true', obj.pid));
                pause(0.2);
                system(sprintf('kill -9 %s 2>/dev/null || true', obj.pid));
                fprintf('[MovingLandingTargetModule] stopped (PID: %s)\n', obj.pid);
                obj.pid = '';
            end
        end

        function cleanup(obj)
            obj.stop();
        end

        function start_camera_detector(obj, repo_root, ardu_ws)
            detector_path = fullfile(repo_root, 'scripts', 'aruco_camera_detector_node.py');
            cmd = sprintf(['bash -lc ''source /opt/ros/humble/setup.bash; ' ...
                'if [ -f "%s/install/setup.bash" ]; then source "%s/install/setup.bash"; fi; ' ...
                'export ROS_DOMAIN_ID=0; ' ...
                'setsid python3 "%s" --namespace "%s" --marker-size-m 1.0 --marker-id 0 --camera-pitch-down-deg 55 --camera-roll-deg 135 ' ...
                '> "%s" 2>&1 < /dev/null & echo $!'''], ...
                ardu_ws, ardu_ws, detector_path, obj.namespace, obj.detector_log_file);
            [status, output] = system(cmd);
            if status == 0
                obj.detector_pid = strtrim(output);
                fprintf('[MovingLandingTargetModule] camera detector started (PID: %s, log: %s)\n', ...
                    obj.detector_pid, obj.detector_log_file);
            else
                fprintf('[MovingLandingTargetModule] camera detector start failed: %s\n', strtrim(output));
            end
        end
    end
end
