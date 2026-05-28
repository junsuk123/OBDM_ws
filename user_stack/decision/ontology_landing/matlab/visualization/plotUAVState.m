function plotUAVState(logData)
%PLOTUAVSTATE Plot altitude, velocity, and attitude.

figure('Name', 'UAV State');
subplot(3,1,1); plot(logData.timeVector, logData.altitudeVector); grid on; ylabel('Altitude');
subplot(3,1,2); plot(logData.timeVector, logData.positionErrorVector); grid on; ylabel('Position error');
subplot(3,1,3); plot(logData.timeVector, rad2deg(logData.rollVector)); hold on; plot(logData.timeVector, rad2deg(logData.pitchVector)); grid on; ylabel('Attitude');