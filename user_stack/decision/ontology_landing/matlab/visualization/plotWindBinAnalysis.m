function plotWindBinAnalysis(results)
%PLOTWINDBINANALYSIS Plot wind-speed-bin decision statistics.

figure('Name', 'Wind Bin Analysis');
bar([results.windBinAnalysis.proposed].recall);
grid on; xlabel('Bin'); ylabel('Recall');