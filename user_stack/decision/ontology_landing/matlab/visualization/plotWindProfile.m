function plotWindProfile(logData)
%PLOTWINDPROFILE Plot wind profile over time.

figure('Name', 'Wind Profile');
plot(logData.timeVector, logData.windSpeedVector, 'LineWidth', 1.4);
xlabel('Time (s)'); ylabel('Wind speed (m/s)'); grid on;