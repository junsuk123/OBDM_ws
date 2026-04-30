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
            % Find launch_plane.sh script
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
        
        function obj = start_simulation(obj, varargin)
            % Start simulation with optional auto-launch
            % Usage: sim = sim.start_simulation('auto_launch', true, 'wait_for_topics', true);
            
            p = inputParser;
            addParameter(p, 'auto_launch', true, @islogical);
            addParameter(p, 'wait_for_topics', true, @islogical);
            parse(p, varargin{:});
            
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
                launch_script = obj.get_launch_script_path();
                
                if ~isempty(launch_script) && exist(launch_script, 'file')
                    fprintf('\\n🚀 Launching simulation...\\n');
                    fprintf('  Script: %s\\n', launch_script);
                    
                    % Create launch command with proper environment setup
                    % This ensures ROS2 and ArduPilot environments are loaded
                    cmd = sprintf('bash -c "source /opt/ros/humble/setup.bash && source $HOME/ardu_ws/install/setup.bash && export ROS_DOMAIN_ID=0 && export GZ_VERSION=harmonic && bash %s > %s 2>&1 &"', ...
                        launch_script, ...
                        SimulationManager.LAUNCH_LOG_FILE);
                    
                    if obj.verbose
                        fprintf('[SimulationManager] Environment setup:\\n');
                        fprintf('[SimulationManager]   ROS_HUMBLE: /opt/ros/humble\\n');
                        fprintf('[SimulationManager]   ARDU_WS: $HOME/ardu_ws\\n');
                        fprintf('[SimulationManager]   ROS_DOMAIN_ID: 0\\n');
                        fprintf('[SimulationManager]   GZ_VERSION: harmonic\\n');
                    end
                    
                    system(cmd);
                    
                    % Wait a moment for process to start
                    pause(3);
                    
                    % Extract PID
                    [~, pid_out] = system('pgrep -f "launch.*alti_transition" | head -1');
                    if ~isempty(strtrim(pid_out))
                        obj.launch_pid = str2double(strtrim(pid_out));
                        obj.is_running = true;
                        if obj.verbose
                            fprintf('[SimulationManager] Simulation launched (PID: %d)\\n', obj.launch_pid);
                        end
                    else
                        if obj.verbose
                            fprintf('[SimulationManager] Warning: Could not detect launch process PID\\n');
                            fprintf('[SimulationManager] Check log: %s\\n', SimulationManager.LAUNCH_LOG_FILE);
                        end
                    end
                else
                    fprintf('✗ Launch script not found!\\n');
                    if obj.verbose
                        fprintf('[SimulationManager] Checked locations:\n');
                        for i = 1:length(possible_paths)
                            fprintf('[SimulationManager]   %s\n', possible_paths{i});
                        end
                    end
                end
            end
            
            % Wait for topics if enabled
            if p.Results.wait_for_topics
                obj = obj.wait_for_topics();
            end
        end
        
        function obj = wait_for_topics(obj, timeout)
            % Wait for ROS2 topics to become available
            
            if nargin < 2
                timeout = SimulationManager.TOPIC_CHECK_TIMEOUT;
            end
            
            fprintf('\\n⏳ Waiting for topics (max %d seconds)...\\n', timeout);
            
            t0 = tic;
            gps_topic = obj.ros2_manager.get_topic('gps/sample');
            
            while toc(t0) < timeout
                if obj.ros2_manager.check_topic_available(gps_topic)
                    fprintf('✓ Topics available!\\n\\n');
                    obj.is_running = true;
                    return;
                end
                pause(1);
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
            fprintf('  IMU: %s\\n', obj.check_topic_status(topics.imu));
            fprintf('  GPS: %s\\n', obj.check_topic_status(topics.gps));
            fprintf('  TF: %s\\n', obj.check_topic_status(topics.tf));
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
    end
end
