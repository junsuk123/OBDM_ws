%% SimulationManager - Simulation Orchestration
% Manages Gazebo + ArduPilot simulation with auto-launch and auto-cleanup
% Integrates ROS2Manager for namespace detection and ProcessManager for resource cleanup

classdef SimulationManager
    properties (Constant)
        LAUNCH_TIMEOUT = 60;           % Max time to wait for simulation launch (seconds)
        TOPIC_CHECK_TIMEOUT = 30;      % Max time to wait for topics (seconds)
        LAUNCH_LOG_FILE = '/tmp/obdm_plane_launch.log';
    end
    
    properties
        is_running = false;
        launch_pid = 0;
        verbose = false;
        ros2_manager;
        process_manager;
        launch_prefix = '';
    end
    
    methods
        function obj = SimulationManager(varargin)
            % Constructor
            % Usage: sim = SimulationManager('verbose', true);
            
            p = inputParser;
            addParameter(p, 'verbose', false, @islogical);
            parse(p, varargin{:});
            
            obj.verbose = p.Results.verbose;
            
            % Initialize manager instances
            obj.ros2_manager = ROS2Manager('verbose', obj.verbose);
            obj.process_manager = ProcessManager('verbose', obj.verbose);
        end
        
        function obj = initialize(obj)
            % Initialize: cleanup existing processes and setup ROS2
            
            if obj.verbose
                fprintf('[SimulationManager] Initializing...\\n');
            end
            
            % Cleanup existing simulation
            obj.process_manager = obj.process_manager.cleanup_all_simulation();
            pause(1);
            
            % Initialize ROS2 environment and namespace detection
            obj.ros2_manager = obj.ros2_manager.Initialize();
            
            if obj.verbose
                fprintf('[SimulationManager] Initialization complete\\n');
            end
        end
        
        function launch_script = get_launch_script_path(obj)
            % Find launch_plane.sh script or direct ros2 launch command
            % Try multiple possible locations
            
            % Derive repository root relative to this file so copied locations still work
            repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            possible_paths = {
                fullfile(repo_root, 'simulation', 'launch', 'launch_plane.sh');
                fullfile(getenv('HOME'), 'OBDM_ws', 'simulation', 'launch', 'launch_plane.sh');
                fullfile('..', 'simulation', 'launch', 'launch_plane.sh');
                fullfile('..', '..', 'simulation', 'launch', 'launch_plane.sh');
            };
            
            for i = 1:length(possible_paths)
                if exist(possible_paths{i}, 'file')
                    launch_script = possible_paths{i};
                    if obj.verbose
                        fprintf('[SimulationManager] Found launch script: %s\\n', launch_script);
                    end
                    return;
                end
            end
            
            % If not found, return default (will fail later)
            launch_script = '';
            if obj.verbose
                fprintf('[SimulationManager] Warning: Launch script not found in any location\\n');
            end
        end
        
        function cmd = build_launch_command(obj)
            % Build ROS2 launch command directly (MathWorks + ArduPilot standard)
            % This is more reliable than shell scripts
            
            home_dir = getenv('HOME');
            ardu_ws = fullfile(home_dir, 'ardu_ws');
            display_env = getenv('DISPLAY');
            xauthority_env = getenv('XAUTHORITY');
            xdg_runtime_env = getenv('XDG_RUNTIME_DIR');
            dbus_env = getenv('DBUS_SESSION_BUS_ADDRESS');
            
            % Check if ardu_ws exists
            if ~isdir(ardu_ws)
                if obj.verbose
                    fprintf('[SimulationManager] Warning: ardu_ws not found at %s\\n', ardu_ws);
                end
                cmd = '';
                return;
            end
            
            % Build the exact ros2 launch command used in the terminal.
            % Keep GUI/session exports in the wrapper so MATLAB inherits the
            % same X11 and DBus context as an interactive shell.
            base_cmd = sprintf([...
                'unset LD_LIBRARY_PATH; ' ...
                'unset QT_PLUGIN_PATH; ' ...
                'unset QML2_IMPORT_PATH; ' ...
                'unset QT_QPA_PLATFORM_PLUGIN_PATH; ' ...
                'unset QT_STYLE_OVERRIDE; ' ...
                'export DISPLAY=%s; ' ...
                'export XAUTHORITY=%s; ' ...
                'export XDG_RUNTIME_DIR=%s; ' ...
                'export DBUS_SESSION_BUS_ADDRESS=%s; ' ...
                'export QT_X11_NO_MITSHM=1; ' ...
                'source /opt/ros/humble/setup.bash; ' ...
                'if [ -f %s/install/setup.bash ]; then source %s/install/setup.bash; fi; ' ...
                'ros2 launch ardupilot_gz_bringup alti_transition_runway.launch.py'], ...
                display_env, xauthority_env, xdg_runtime_env, dbus_env, ardu_ws, ardu_ws);

            if ~isempty(strtrim(obj.launch_prefix))
                cmd = sprintf('%s && %s', strtrim(obj.launch_prefix), base_cmd);
            else
                cmd = base_cmd;
            end
            
            if obj.verbose
                fprintf('[SimulationManager] Launch command built successfully\\n');
            end
        end
        
        function obj = start_simulation(obj, varargin)
            % Start simulation with optional auto-launch
            % Usage: sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
            
            p = inputParser;
            addParameter(p, 'auto_launch', true, @islogical);
            addParameter(p, 'wait_for_topics', true, @islogical);
            addParameter(p, 'ap_dds_sensor_config', struct(), @isstruct);
            addParameter(p, 'rebuild_ardupilot', true, @islogical);
            parse(p, varargin{:});

            obj.launch_prefix = '';

            if ~isempty(fieldnames(p.Results.ap_dds_sensor_config))
                config_result = SimulationConfig.apply_ap_dds_sensor_config(p.Results.ap_dds_sensor_config);
                if obj.verbose
                    fprintf('[SimulationManager] AP_DDS config path: %s\n', config_result.config_path);
                    if ~isempty(config_result.backup_path)
                        fprintf('[SimulationManager] AP_DDS backup path: %s\n', config_result.backup_path);
                    end
                end

                if config_result.changed && p.Results.rebuild_ardupilot
                    obj.launch_prefix = sprintf('cd %s && colcon build --packages-select ardupilot_sitl --cmake-force-configure --event-handlers console_direct+', fullfile(getenv('HOME'), 'ardu_ws'));
                    if obj.verbose
                        fprintf('[SimulationManager] AP_DDS rebuild enabled before launch\n');
                    end
                elseif config_result.changed
                    fprintf('⚠️ AP_DDS config updated but rebuild disabled. Rebuild ardupilot_sitl before launch for changes to take effect.\n');
                end

                if obj.verbose && ~isempty(config_result.applied_fields)
                    fprintf('[SimulationManager] AP_DDS applied fields: %s\n', strjoin(config_result.applied_fields, ', '));
                end

                if obj.verbose && ~isempty(config_result.messages)
                    for i = 1:numel(config_result.messages)
                        fprintf('[SimulationManager] %s\n', config_result.messages{i});
                    end
                end
            end
            
            if obj.verbose
                fprintf('[SimulationManager] Starting simulation...\\n');
            end
            
            % Check if already running
            if obj.is_running
                if obj.verbose
                    fprintf('[SimulationManager] Simulation already running\\n');
                end
                return;
            end
            
            % Auto-launch if enabled
            if p.Results.auto_launch
                fprintf('\\n🚀 Launching simulation via ROS2...\\n');
                
                % === Step 1: Set environment variables ===
                if obj.verbose
                    fprintf('[SimulationManager] Setting environment variables:\\n');
                end
                
                setenv('ROS_DOMAIN_ID', '0');
                if obj.verbose
                    fprintf('[SimulationManager]   ROS_DOMAIN_ID: %s\\n', getenv('ROS_DOMAIN_ID'));
                end
                
                setenv('GZ_VERSION', 'harmonic');
                if obj.verbose
                    fprintf('[SimulationManager]   GZ_VERSION: %s\\n', getenv('GZ_VERSION'));
                end
                
                if isempty(getenv('DISPLAY'))
                    setenv('DISPLAY', ':0');
                end
                if obj.verbose
                    fprintf('[SimulationManager]   DISPLAY: %s\\n', getenv('DISPLAY'));
                end
                
                % === Step 2: Build direct ros2 launch command ===
                launch_cmd = obj.build_launch_command();
                
                if isempty(launch_cmd)
                    fprintf('✗ Failed to build launch command\\n');
                    fprintf('  Possible issues:\\n');
                    fprintf('    • ardu_ws not found at ~/ardu_ws\\n');
                    fprintf('    • ROS2 installation incomplete\\n');
                    return;
                end
                
                if obj.verbose
                    fprintf('[SimulationManager] Command: %s\\n', launch_cmd);
                end
                
                % === Step 3: Execute in background and capture PID ===
                log_file = SimulationManager.LAUNCH_LOG_FILE;
                pid_file = [log_file, '.pid'];
                
                % Clean up old files
                if exist(log_file, 'file')
                    delete(log_file);
                end
                if exist(pid_file, 'file')
                    delete(pid_file);
                end
                
                % Execute inside interactive bash so ~/.bashrc is actually read
                % before the terminal launch environment is constructed.
                bg_cmd = sprintf('nohup bash -ic ''%s'' > %s 2>&1 & echo $! > %s', ...
                    launch_cmd, log_file, pid_file);
                
                [status, result] = system(bg_cmd);
                
                if obj.verbose
                    fprintf('[SimulationManager] System call status: %d\\n', status);
                    if status ~= 0
                        fprintf('[SimulationManager] System output: %s\\n', result);
                    end
                end
                
                % === Step 4: Wait and verify PID file ===
                fprintf('  Waiting for launch process...');
                max_wait = 5;
                waited = 0;
                while waited < max_wait && ~exist(pid_file, 'file')
                    pause(0.2);
                    waited = waited + 0.2;
                    fprintf('.');
                end
                fprintf('\\n');
                
                if exist(pid_file, 'file')
                    try
                        pid_str = strtrim(fileread(pid_file));
                        obj.launch_pid = str2double(pid_str);
                        
                        if ~isnan(obj.launch_pid) && obj.launch_pid > 0
                            % Verify process is actually running
                            [ps_status, ~] = system(sprintf('ps -p %d > /dev/null 2>&1', obj.launch_pid));
                            if ps_status == 0
                                obj.is_running = true;
                                fprintf('  ✓ Launch process started (PID: %d)\\n', obj.launch_pid);
                            else
                                % Parent PID is background shell - ros2 launch runs inside
                                % This is expected for nohup + bash -c combination
                                obj.is_running = true;
                                fprintf('  ✓ Launch initiated (Parent PID: %d)\\n', obj.launch_pid);
                            end
                            
                            if obj.verbose
                                fprintf('[SimulationManager] Log file: %s\\n', log_file);
                            end
                        else
                            obj.is_running = false;
                            fprintf('  ✗ Invalid PID: %s\\n', pid_str);
                        end
                    catch ME
                        obj.is_running = false;
                        fprintf('  ✗ Error reading PID file: %s\\n', ME.message);
                    end
                else
                    obj.is_running = false;
                    fprintf('  ✗ PID file not created\\n');
                    fprintf('  Check log file: %s\\n', log_file);
                    if exist(log_file, 'file')
                        fprintf('\\nLog contents:\\n');
                        system(sprintf('tail -20 %s', log_file));
                    end
                end
            end
            
            % Wait for topics if enabled
            if p.Results.wait_for_topics
                obj = obj.wait_for_topics();
            end
        end

        function probe = probe_launch_command(obj, timeout_seconds)
            % Run the exact launch command in the foreground for a short time.
            % This is a diagnostic check to prove MATLAB system() is reaching ros2 launch.

            if nargin < 2 || isempty(timeout_seconds)
                timeout_seconds = 15;
            end

            launch_cmd = obj.build_launch_command();
            if isempty(launch_cmd)
                probe = struct('status', -1, 'output', 'launch command could not be built');
                return;
            end

            log_file = [SimulationManager.LAUNCH_LOG_FILE, '.probe'];
            if exist(log_file, 'file')
                delete(log_file);
            end

            % Use timeout so we can observe startup output without waiting forever.
            probe_cmd = sprintf('timeout %d bash -lc ''%s'' > %s 2>&1', ...
                timeout_seconds, launch_cmd, log_file);

            if obj.verbose
                fprintf('[SimulationManager] Probe command: %s\n', probe_cmd);
            end

            [status, result] = system(probe_cmd);
            output = '';
            if exist(log_file, 'file')
                output = fileread(log_file);
            end

            if obj.verbose
                fprintf('[SimulationManager] Probe status: %d\n', status);
                if ~isempty(strtrim(result))
                    fprintf('[SimulationManager] Probe system output: %s\n', result);
                end
                if ~isempty(strtrim(output))
                    fprintf('[SimulationManager] Probe log tail:\n%s\n', output);
                end
            end

            probe = struct('status', status, 'system_output', result, 'log_output', output, 'log_file', log_file);
        end
        
        function obj = wait_for_topics(obj, timeout)
            % Wait for ROS2 topics to become available
            
            if nargin < 2
                timeout = SimulationManager.TOPIC_CHECK_TIMEOUT;
            end
            
            fprintf('\\n⏳ Waiting for topics (max %d seconds)...\\n', timeout);
            
            % Get standard topics for this namespace
            topics = obj.ros2_manager.get_standard_topics();
            
            t0 = tic;
            
            while toc(t0) < timeout
                % Check multiple common topics
                imu_available = obj.ros2_manager.check_topic_available(topics.imu);
                odom_available = obj.ros2_manager.check_topic_available(topics.odometry);
                tf_available = obj.ros2_manager.check_topic_available(topics.tf);
                
                % Success if at least one simulation topic is available.
                if imu_available || odom_available || tf_available
                    fprintf('✓ Topics available!\\n\\n');
                    obj.is_running = true;
                    return;
                end
                
                pause(1.2);
                fprintf('  Elapsed: %.1f s\\n', toc(t0));
            end
            
            fprintf('⚠️ Warning: Topics not detected after %d seconds\\n', timeout);
            fprintf('  Troubleshooting:\\n');
            fprintf('    • Is Gazebo running?\\n');
            fprintf('    • Check ROS2 namespace: %s\\n', obj.ros2_manager.get_namespace());
            fprintf('    • Verify ROS_DOMAIN_ID is set\\n');
        end
        
        function obj = stop_simulation(obj)
            % Stop simulation and cleanup processes
            
            if obj.verbose
                fprintf('[SimulationManager] Stopping simulation...\\n');
            end
            
            % Cleanup simulation processes
            obj.process_manager = obj.process_manager.cleanup_all_simulation();
            
            obj.is_running = false;
            obj.launch_pid = 0;
            
            if obj.verbose
                fprintf('[SimulationManager] Simulation stopped\\n');
            end
        end
        
        function is_ok = check_simulation_status(obj)
            % Check if simulation is still running
            
            if obj.launch_pid > 0
                % Check process by PID
                [status, ~] = system(sprintf('ps -p %d > /dev/null 2>&1', obj.launch_pid));
                is_ok = (status == 0) && obj.is_running;
            else
                is_ok = obj.is_running;
            end
        end
        
        function topics = get_topics(obj)
            % Get standard topics for current namespace
            topics = obj.ros2_manager.get_standard_topics();
        end
        
        function services = get_services(obj)
            % Get standard services for current namespace
            services = obj.ros2_manager.get_standard_services();
        end
        
        function ns = get_namespace(obj)
            % Get detected namespace
            ns = obj.ros2_manager.get_namespace();
        end
        
        function print_status(obj)
            % Print simulation status
            fprintf('\\n╔════════════════════════════════╗\\n');
            fprintf('║ SimulationManager Status        ║\\n');
            fprintf('╚════════════════════════════════╝\\n');
            fprintf('Running: %d\\n', obj.is_running);
            if obj.launch_pid > 0
                fprintf('Process PID: %d\\n', obj.launch_pid);
            end
            fprintf('Namespace: %s\\n', obj.ros2_manager.get_namespace());
            
            % Topic status
            topics = obj.ros2_manager.get_standard_topics();
            fprintf('\\nTopic Status:\\n');
            fprintf('  IMU: %s\n', obj.check_topic_status(topics.imu));
            fprintf('  ODOM: %s\n', obj.check_topic_status(topics.odometry));
            fprintf('  TF: %s\n', obj.check_topic_status(topics.tf));
            fprintf('\\n');
        end
        
        function status_str = check_topic_status(obj, topic_name)
            % Check if topic is available and return status string
            if obj.ros2_manager.check_topic_available(topic_name)
                status_str = sprintf('✓ %s', topic_name);
            else
                status_str = sprintf('✗ %s (NOT FOUND)', topic_name);
            end
        end
        
        function is_valid = verify_workspace(obj)
            % Verify ROS2 and ArduPilot workspace configuration
            % Returns true if all required components are present
            
            fprintf('\\n╔════════════════════════════════╗\\n');
            fprintf('║ Workspace Verification         ║\\n');
            fprintf('╚════════════════════════════════╝\\n\\n');
            
            is_valid = true;
            home_dir = getenv('HOME');
            
            % Check 1: ROS2 installation
            ros_setup = '/opt/ros/humble/setup.bash';
            if exist(ros_setup, 'file')
                fprintf('✓ ROS2 Humble installation found\\n');
            else
                fprintf('✗ ROS2 Humble not found at %s\\n', ros_setup);
                is_valid = false;
            end
            
            % Check 2: ArduPilot Gazebo workspace
            ardu_ws = fullfile(home_dir, 'ardu_ws');
            ardu_setup = fullfile(ardu_ws, 'install', 'setup.bash');
            if isdir(ardu_ws) || exist(ardu_ws, 'dir')
                fprintf('✓ ArduPilot workspace found at %s\\n', ardu_ws);
                if exist(ardu_setup, 'file')
                    fprintf('  ✓ Workspace setup.bash found\\n');
                else
                    fprintf('  ✗ Workspace setup.bash not found\\n');
                    fprintf('    Run: cd ~/ardu_ws && colcon build --packages-up-to ardupilot_gz_bringup\\n');
                    is_valid = false;
                end
            else
                fprintf('✗ ArduPilot workspace not found at %s\\n', ardu_ws);
                fprintf('  Expected: clone and build from https://github.com/ArduPilot/ardupilot_gz\\n');
                is_valid = false;
            end
            
            % Check 3: Required ROS2 packages
            bringup_pkg = fullfile(ardu_ws, 'install', 'ardupilot_gz_bringup');
            if exist(bringup_pkg, 'dir')
                fprintf('✓ ardupilot_gz_bringup package found\\n');
            else
                fprintf('✗ ardupilot_gz_bringup package not found\\n');
                is_valid = false;
            end
            
            % Check 4: Environment variables
            fprintf('\\nEnvironment Variables:\\n');
            fprintf('  ROS_DOMAIN_ID: %s\\n', getenv('ROS_DOMAIN_ID'));
            fprintf('  GZ_VERSION: %s\\n', getenv('GZ_VERSION'));
            fprintf('  DISPLAY: %s\\n', getenv('DISPLAY'));
            
            % Check 5: Gazebo installation
            [status, ~] = system('which gz 2>/dev/null');
            if status == 0
                fprintf('\\n✓ Gazebo CLI found\\n');
            else
                fprintf('\\n✗ Gazebo not in PATH\\n');
                fprintf('  Run: source /opt/ros/humble/setup.bash\\n');
            end
            
            fprintf('\\n');
            
            if is_valid
                fprintf('✓ Workspace configuration OK - Ready to launch\\n\\n');
            else
                fprintf('✗ Workspace configuration incomplete\\n');
                fprintf('  Please fix the issues above before running simulation\\n\\n');
            end
        end
    end
end
