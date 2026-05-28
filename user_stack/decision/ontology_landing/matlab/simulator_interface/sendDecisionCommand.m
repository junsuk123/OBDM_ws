function sim = sendDecisionCommand(sim, decision)
%SENDDECISIONCOMMAND CamelCase wrapper for decision command dispatch.

cfg = decision.cfg;
if ~isfield(decision, 'classProbability')
    decision.classProbability = [0.5, 0.5];
end
if ~isfield(decision, 'confidence')
    decision.confidence = max(decision.classProbability);
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

sim = send_decision_command(sim, decision.action, decision.classProbability, ...
    decision.confidence, cfg, decision.uavState, decision.observation, decision.semantic);
end
