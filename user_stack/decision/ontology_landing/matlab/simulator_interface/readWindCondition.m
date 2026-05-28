function [windState, sim] = readWindCondition(sim, cfg)
%READWINDCONDITION Read or estimate wind condition from the simulator wrapper.

timestamp = 0.0;
windVector = cfg.wind.meanVelocity;
[windState, sim] = read_wind_condition(sim, timestamp, windVector, cfg);