function sim = initialize_landing_simulation(sim, cfg, projectRoot)
%INITIALIZE_LANDING_SIMULATION Apply integrated_main-style auto launch when enabled.

if nargin < 3
    projectRoot = fileparts(mfilename('fullpath'));
end

sim.launcher = [];
sim.launchInfo = struct('enabled', false, 'mode', cfg.simulation.runMode, 'namespace', sim.namespace, 'message', '');

if isfield(sim, 'backend') && strcmpi(strtrim(sim.backend), 'mock')
    sim.launchInfo.message = 'Mock backend selected; skipping ROS2 launch and using placeholder execution.';
    return;
end

if ~isfield(cfg.simulation, 'autoLaunch') || ~cfg.simulation.autoLaunch
    sim.launchInfo.message = 'Auto execution disabled; using mock simulator interface.';
    return;
end

try
    repoRoot = locate_repo_root_local(projectRoot);
    workspaceMatlabRoot = fullfile(repoRoot, 'matlab');
    if isfolder(workspaceMatlabRoot)
        addpath(genpath(workspaceMatlabRoot));
    end

    sanitize_runtime_environment_local();
    setenv('ROS_DOMAIN_ID', num2str(cfg.simulation.rosDomainId));
    if isempty(getenv('RMW_IMPLEMENTATION'))
        setenv('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp');
    end
    if isempty(getenv('ROS_LOCALHOST_ONLY'))
        setenv('ROS_LOCALHOST_ONLY', '0');
    end
    setenv('GZ_VERSION', cfg.simulation.gzVersion);
    if isempty(getenv('DISPLAY'))
        setenv('DISPLAY', ':0');
    end

    if exist('SimulationManager', 'class') ~= 8
        sim.launchInfo.message = 'SimulationManager class not found; falling back to mock execution.';
        return;
    end

    sim.launcher = SimulationManager('verbose', cfg.runtime.verbose);
    sim.launcher = sim.launcher.initialize( ...
        'reset_method', cfg.simulation.resetMethod, ...
        'world_name', cfg.simulator.gazeboWorldName, ...
        'soft_reset_config', cfg.simulator.reset);

    if strcmpi(strtrim(cfg.simulation.runMode), 'ap_dds_plane')
        if ~isfield(cfg.simulation, 'apDdsSensorConfig') || isempty(fieldnames(cfg.simulation.apDdsSensorConfig))
            cfg.simulation.apDdsSensorConfig = struct( ...
                'navsat', true, 'imu', true, 'battery', true, 'airspeed', true, ...
                'pose', true, 'twist', true, 'geopose', true, 'static_tf', true, ...
                'status', true, 'rc', true, 'cmd_gps_pose', true, 'arm_motors', true, ...
                'mode_switch', true, 'takeoff', true, 'prearm_check', true);
        end

        sim.launcher = sim.launcher.start_simulation( ...
            'auto_launch', true, ...
            'wait_for_topics', cfg.simulation.waitForTopics, ...
            'launch_mode', cfg.simulation.runMode, ...
            'ap_dds_sensor_config', cfg.simulation.apDdsSensorConfig);
    else
        sim.launcher = sim.launcher.start_simulation( ...
            'auto_launch', true, ...
            'wait_for_topics', cfg.simulation.waitForTopics, ...
            'launch_mode', cfg.simulation.runMode);
    end

    sim.launcher.ros2_manager = sim.launcher.ros2_manager.auto_detect_namespace();
    if cfg.simulation.postLaunchWaitSec > 0
        pause(cfg.simulation.postLaunchWaitSec);
    end

    sim.backend = 'integrated_main_flow';
    sim.isConnected = true;
    sim.namespace = sim.launcher.get_namespace();
    sim.launchInfo = struct( ...
        'enabled', true, ...
        'mode', cfg.simulation.runMode, ...
        'namespace', sim.namespace, ...
        'message', 'Simulation auto-launched using SimulationManager');
catch ME
    sim.launchInfo = struct( ...
        'enabled', false, ...
        'mode', cfg.simulation.runMode, ...
        'namespace', sim.namespace, ...
        'message', ME.message);

    if cfg.simulation.allowFallbackToMock
        sim.backend = 'mock';
        sim.isConnected = false;
    else
        rethrow(ME);
    end
end

end

function repoRoot = locate_repo_root_local(startDir)
repoRoot = startDir;
for idx = 1:12
    if isfolder(fullfile(repoRoot, 'matlab', 'modules')) && isfolder(fullfile(repoRoot, 'user_stack'))
        return;
    end
    parentDir = fileparts(repoRoot);
    if strcmp(parentDir, repoRoot)
        break;
    end
    repoRoot = parentDir;
end
repoRoot = startDir;
end

function sanitize_runtime_environment_local()
snapVars = { ...
    'SNAP', 'SNAP_ARCH', 'SNAP_COMMON', 'SNAP_CONTEXT', 'SNAP_COOKIE', ...
    'SNAP_DATA', 'SNAP_EUID', 'SNAP_INSTANCE_NAME', 'SNAP_LAUNCHER_ARCH_TRIPLET', ...
    'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_REAL_HOME', 'SNAP_REVISION', ...
    'SNAP_UID', 'SNAP_USER_COMMON', 'SNAP_USER_DATA', 'SNAP_VERSION', ...
    'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GTK_PATH', 'GTK_MODULES', ...
    'LOCPATH', 'QT_IM_MODULE', 'XDG_DATA_HOME'};
for idx = 1:numel(snapVars)
    setenv(snapVars{idx}, '');
end
setenv('XDG_DATA_DIRS', '/usr/local/share:/usr/share');
setenv('XDG_CONFIG_DIRS', '/etc/xdg');
setenv('QT_ACCESSIBILITY', '0');
setenv('NO_AT_BRIDGE', '1');
end
