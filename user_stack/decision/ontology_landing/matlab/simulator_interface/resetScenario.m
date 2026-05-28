function sim = resetScenario(sim, cfg)
%RESETSCENARIO Reset one episode before takeoff and pad motion start.

scenarioIndex = 1;
if isfield(sim, 'currentScenario') && sim.currentScenario > 0
    scenarioIndex = sim.currentScenario + 1;
end
sim = reset_simulator_scenario(sim, scenarioIndex, cfg);