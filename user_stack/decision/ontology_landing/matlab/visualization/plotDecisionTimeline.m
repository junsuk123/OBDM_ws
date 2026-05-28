function plotDecisionTimeline(logData)
%PLOTDECISIONTIMELINE Plot LAND/HOVER decisions over time.

figure('Name', 'Decision Timeline');
stairs(logData.timeVector, logData.decisionOnlineVector, 'LineWidth', 1.4);
grid on; xlabel('Time (s)'); ylabel('Decision');