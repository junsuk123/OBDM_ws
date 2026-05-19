% integrated_example - Demonstrates module-based startup and shutdown
cfg = struct();
cfg.namespace = '/iris';
cfg.verbose = true;
cfg.base_dir = fullfile(pwd,'matlab','data');

% Create modules
logger_mod = DataLoggerModule();
vis_mod = RealtimeVisualizerModule();
collector_mod = DataCollectorModuleAdapter();

% Initialize
logger_mod.init(cfg);
vis_mod.init(cfg);
collector_mod.init(cfg);

% Start background modules
logger_mod.start();
if usejava('desktop')
    vis_mod.start();
else
    fprintf('[integrated_example] Desktop not available; visualizer skipped\n');
end
collector_mod.start();

% Collect data for 5s; wind estimate is derived from the airspeed sensor.
data = collector_mod.collect_data(5);
collector_mod.stop();

% Persist collected airspeed rows into logger (if present)
if isfield(data,'airspeed') && ~isempty(data.airspeed)
    t0 = 0; % relative time placeholder
    for i=1:size(data.airspeed,1)
        row = data.airspeed(i,:);
        logger_mod.logger.log_wind(t0 + i, row(1), row(2), row(3));
    end
    fprintf('[integrated_example] Airspeed-derived wind samples recorded from collector: %d\n', size(data.airspeed,1));
end

% Stop and cleanup
vis_mod.stop();
logger_mod.stop();
logger_mod.cleanup();
collector_mod.cleanup();

fprintf('[integrated_example] Example run complete\n');
