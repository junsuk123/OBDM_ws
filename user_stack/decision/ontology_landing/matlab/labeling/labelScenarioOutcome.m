function label = labelScenarioOutcome(logData, cfg)
%LABELSCENARIOOUTCOME Automatically label safe landing vs unsafe landing.

uavState = logData.uavState;
observation = logData.observation;
[label, ~, ~] = label_landing_outcome(uavState, observation, cfg);