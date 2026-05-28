function sim = startLandingPadMotion(sim, cfg)
%STARTLANDINGPADMOTION Start or command landing pad motion.
% Reference integrated_main.m for the target motion control or the pad module.

sim.padMotion = struct();
sim.padMotion.enabled = true;
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
moduleCfg.targetInitialX = 0.0;
moduleCfg.targetInitialY = 0.0;
moduleCfg.targetInitialYawRad = cfg.ontology.referenceYawDeg * pi / 180.0;
moduleCfg.targetMotionMode = 'circle';
moduleCfg.targetCircleRadiusM = 0.6;
moduleCfg.targetCircleAngularSpeed = 0.24;
moduleCfg.targetSpawnForwardM = 0.0;
moduleCfg.targetMotionRangeM = 1.0;
moduleCfg.targetMinForwardOffsetM = 0.0;
moduleCfg.targetMaxForwardOffsetM = 2.0;
moduleCfg.waitForTakeoff = true;
moduleCfg.takeoffZThreshold = cfg.simulator.takeoffDetectionAltitude;
moduleCfg.guidanceTakeoffAltitude = cfg.simulator.takeoffAltitude;
moduleCfg.touchdownZThreshold = cfg.labeling.touchdownAltitude;
moduleCfg.touchdownXYTolerance = cfg.labeling.touchdownPositionError;
moduleCfg.touchdownMaxAbsVz = max(cfg.labeling.safeVerticalSpeed, 1.0);

try
    sim.padModule = MovingLandingTargetModule();
    sim.padModule.init(moduleCfg);
    sim.padModule.start();
    fprintf('[simulator_interface] landing pad motion started via MovingLandingTargetModule (amp %.2f, freq %.2f, speed %.2f)\n', ...
        sim.padMotion.amplitude, sim.padMotion.frequency, sim.padMotion.speed);
catch ME
    fprintf('[simulator_interface] startLandingPadMotion failed: %s\n', ME.message);
    sim.padModule = [];
end
