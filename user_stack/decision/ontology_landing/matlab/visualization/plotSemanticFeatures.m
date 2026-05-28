function plotSemanticFeatures(logData)
%PLOTSEMANTICFEATURES Plot semantic feature values.

figure('Name', 'Semantic Features');
plot(logData.timeVector, logData.semanticMatrix, 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Normalized value');