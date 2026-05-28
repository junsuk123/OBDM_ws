function [uavState, sim] = readUAVState(sim, cfg)
%READUAVSTATE Read UAV state from the simulator wrapper.

timestamp = 0.0;
windVector = cfg.wind.meanVelocity;
[uavState, sim] = read_uav_state(sim, timestamp, windVector, cfg);