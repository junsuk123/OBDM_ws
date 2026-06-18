function sim = sendDecisionCommand(sim, decision)
%SENDDECISIONCOMMAND CamelCase wrapper for decision command dispatch.

cfg = decision.cfg;
if ~isfield(decision, 'landingIntentScore')
    decision.landingIntentScore = 0.5;
end
if ~isfield(decision, 'confidence')
    decision.confidence = 1.0;
end
if ~isfield(decision, 'uavState')
    decision.uavState = [];
end
if ~isfield(decision, 'observation')
    decision.observation = struct();
end
if ~isfield(decision, 'semantic')
    decision.semantic = [];
end

sim = send_decision_command(sim, decision.action, decision.landingIntentScore, ...
    decision.confidence, cfg, decision.uavState, decision.observation, decision.semantic);
end
