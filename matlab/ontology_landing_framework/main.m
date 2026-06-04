% Compatibility wrapper. The ontology landing stack now lives under
% user_stack/decision/ontology_landing/matlab.

wrapperDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(wrapperDir));
newMain = fullfile(repoRoot, 'user_stack', 'decision', 'ontology_landing', 'matlab', 'main.m');
if ~exist(newMain, 'file')
    error('Ontology landing main not found at expected path: %s', newMain);
end
run(newMain);