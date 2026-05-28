function [observation, sim] = readLandingPadObservation(sim, uavState, windState, cfg)
%READLANDINGPADOBSERVATION Read the AprilTag / landing-pad observation layer.

[observation, sim] = read_landing_observation(sim, uavState, windState, cfg);