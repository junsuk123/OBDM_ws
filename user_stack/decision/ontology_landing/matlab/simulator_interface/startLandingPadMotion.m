function sim = startLandingPadMotion(sim, cfg)
%STARTLANDINGPADMOTION Start or command landing pad motion.
% Reference integrated_main.m for the target motion control or the pad module.

sim.padMotion = struct();
sim.padMotion.enabled = true;
sim.landingStackReady = false;
sim.padMotion.amplitude = cfg.simulator.landingPadMotion.amplitude;
sim.padMotion.frequency = cfg.simulator.landingPadMotion.frequency;
sim.padMotion.speed = cfg.simulator.landingPadMotion.speed;

if isfield(sim, 'padModule') && ~isempty(sim.padModule)
    try
        sim.padModule.stop();
    catch
    end
end

moduleCfg = struct();
if isfield(sim, 'namespace') && ~isempty(sim.namespace)
    moduleCfg.namespace = sim.namespace;
else
    moduleCfg.namespace = cfg.simulator.namespace;
end
moduleCfg.worldName = 'runway';
moduleCfg.targetModelName = 'r1_rover';
moduleCfg.targetMarkerZ = 0.20;
moduleCfg.targetModelZ = 0.05;
moduleCfg.targetInitialYawRad = cfg.ontology.referenceYawDeg * pi / 180.0;
initialForwardOffsetM = 5.0;
moduleCfg.targetInitialX = initialForwardOffsetM * cos(moduleCfg.targetInitialYawRad);
moduleCfg.targetInitialY = initialForwardOffsetM * sin(moduleCfg.targetInitialYawRad);
moduleCfg.targetMotionMode = 'random';
moduleCfg.targetCircleRadiusM = 0.6;
moduleCfg.targetCircleAngularSpeed = 0.24;
moduleCfg.targetRandomMinSpeed = 0.18;
moduleCfg.targetRandomMaxSpeed = 0.42;
moduleCfg.targetRandomYawrateMax = 0.28;
moduleCfg.targetRandomChangePeriod = 2.5;
moduleCfg.targetRandomLateralRangeM = 0.8;
moduleCfg.targetSpawnForwardM = 0.0;
moduleCfg.targetMotionRangeM = 1.4;
moduleCfg.targetMinForwardOffsetM = 0.0;
moduleCfg.targetMaxForwardOffsetM = 2.2;
moduleCfg.waitForTakeoff = true;
moduleCfg.waitForMotionEnable = true;
moduleCfg.motionEnableTopic = '/obdm/start_pad_motion';
moduleCfg.takeoffZThreshold = cfg.simulator.takeoffDetectionAltitude;
moduleCfg.contactArmingZThreshold = get_labeling_value(cfg, 'contactArmingAltitude', 1.5);
moduleCfg.guidanceTakeoffAltitude = cfg.simulator.takeoffAltitude;
moduleCfg.touchdownZThreshold = cfg.labeling.touchdownAltitude;
moduleCfg.touchdownXYTolerance = cfg.labeling.touchdownPositionError;
moduleCfg.touchdownMaxAbsVz = max(get_labeling_value(cfg, 'touchdownMaxAbsVz', cfg.labeling.safeVerticalSpeed), 1.0);
moduleCfg.touchdownGroundZThreshold = get_labeling_value(cfg, 'touchdownGroundAltitude', cfg.labeling.touchdownAltitude);
moduleCfg.touchdownGroundXYTolerance = get_labeling_value(cfg, 'touchdownGroundPositionError', cfg.labeling.touchdownPositionError);

try
    sim.padModule = MovingLandingTargetModule();
    sim.padModule.init(moduleCfg);
    sim.padModule.start();
    append_startup_log('landing_stack_ready_rover_held', ...
        sprintf('motion_enable_topic=%s', moduleCfg.motionEnableTopic));
    sim.landingStackReady = true;
    fprintf('[simulator_interface] landing stack ready via MovingLandingTargetModule; rover held until release (amp %.2f, freq %.2f, speed %.2f)\n', ...
        sim.padMotion.amplitude, sim.padMotion.frequency, sim.padMotion.speed);
catch ME
    append_startup_log('landing_stack_failed', ME.message);
    if isfield(sim, 'padModule') && ~isempty(sim.padModule)
        try
            sim.padModule.stop();
        catch
        end
    end
    fprintf('[simulator_interface] startLandingPadMotion failed: %s\n', ME.message);
    sim.padModule = [];
    sim.landingStackReady = false;
end

end

function value = get_labeling_value(cfg, fieldName, defaultValue)
value = defaultValue;
if isfield(cfg, 'labeling') && isfield(cfg.labeling, fieldName) && ~isempty(cfg.labeling.(fieldName))
    value = cfg.labeling.(fieldName);
end
end

function append_startup_log(eventName, message)
try
    fid = fopen('/tmp/obdm_startup_sequence.log', 'a');
    if fid < 0
        return;
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, '%s event=%s message=%s\n', datestr(now, 'yyyy-mm-ddTHH:MM:SS.FFF'), eventName, message);
catch
end
end
