%% data quality verify 
% CSV log data quality 

clear; clc;

fprintf('\n');
fprintf('╔════════════════════════════════════════════════╗\n');
fprintf('║ data quality verify (log ) ║\n');
fprintf('╚════════════════════════════════════════════════╝\n\n');

%% 1. log optional
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 1: log optional\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% log check
log_dir = './logs/';
if ~exist(log_dir, 'dir')
 fprintf('✗ logs \n');
 fprintf(' → verify_ros2_connection.m data \n');
 return;
end

% log 
log_files = dir(fullfile(log_dir, 'obdm_flight_log_*.csv'));

if isempty(log_files)
 fprintf('✗ log \n');
 return;
end

% optional
[~, idx] = max([log_files.datenum]);
log_file = fullfile(log_dir, log_files(idx).name);

fprintf('optional log : %s\n', log_files(idx).name);
fprintf(' : %.1f KB\n', log_files(idx).bytes/1024);

%% 2. log data 
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 2: log data \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

try
 % CSV 
 opts = delimitedTextImportOptions('NumHeaderLines', 1);
 opts.Delimiter = ',';
 
 data_table = readtable(log_file, opts);
 
 fprintf('✓ log complete/done\n');
 fprintf(' : %d\n', height(data_table));
 fprintf(' : %d\n', width(data_table));
 
 % check
 fprintf('\n:\n');
 var_names = data_table.Properties.VariableNames;
 for i = 1:length(var_names)
 fprintf(' [%2d] %s\n', i, var_names{i});
 end
 
catch ME
 fprintf('✗ failed: %s\n', ME.message);
 return;
end

%% 3. 
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 3: \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% Info
fprintf(' :\n');
fprintf(' Start: %s\n', data_table.timestamp(1));
fprintf(' terminate: %s\n', data_table.timestamp(end));

% sample 
if height(data_table) > 1
 % ( )
 avg_interval = (height(data_table) - 1) / 60; % 1 
 fprintf(' : ~1\n');
 fprintf(' sample : %d\n', height(data_table));
end

%% 4. data verify
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 4: data verify\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

lat = data_table.lat;
lon = data_table.lon;
alt = data_table.alt;

fprintf(' (LAT):\n');
fprintf(' : %.6f ~ %.6f\n', min(lat), max(lat));
fprintf(' : %.6f\n', mean(lat));
fprintf(' : %.6f\n', std(lat));

fprintf('\n (LON):\n');
fprintf(' : %.6f ~ %.6f\n', min(lon), max(lon));
fprintf(' : %.6f\n', mean(lon));
fprintf(' : %.6f\n', std(lon));

fprintf('\n (ALT) [m]:\n');
fprintf(' : %.1f ~ %.1f m\n', min(alt), max(alt));
fprintf(' : %.1f m\n', mean(alt));
fprintf(' : %.1f m\n', std(alt));
fprintf(' : %.2f m/\n', (max(alt) - min(alt))/60);

%% 5. data verify
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 5: data verify\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

vx = data_table.vx;
vy = data_table.vy;
vz = data_table.vz;

% 
speed = sqrt(vx.^2 + vy.^2 + vz.^2);

fprintf('X (VX) [m/s]:\n');
fprintf(' : %.2f ~ %.2f\n', min(vx), max(vx));
fprintf(' : %.2f\n', mean(vx));

fprintf('\nY (VY) [m/s]:\n');
fprintf(' : %.2f ~ %.2f\n', min(vy), max(vy));
fprintf(' : %.2f\n', mean(vy));

fprintf('\nZ (VZ) [m/s]:\n');
fprintf(' : %.2f ~ %.2f\n', min(vz), max(vz));
fprintf(' : %.2f\n', mean(vz));

fprintf('\n (Speed) [m/s]:\n');
fprintf(' : %.2f ~ %.2f\n', min(speed), max(speed));
fprintf(' : %.2f\n', mean(speed));
fprintf(' : %.2f\n', std(speed));

%% 6. data verify
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 6: data verify\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

roll = data_table.roll;
pitch = data_table.pitch;
yaw = data_table.yaw;

fprintf(' (Roll) [°]:\n');
fprintf(' : %.1f ~ %.1f\n', min(roll), max(roll));
fprintf(' : %.1f\n', mean(roll));

fprintf('\n (Pitch) [°]:\n');
fprintf(' : %.1f ~ %.1f\n', min(pitch), max(pitch));
fprintf(' : %.1f\n', mean(pitch));

fprintf('\n (Yaw) [°]:\n');
fprintf(' : %.1f ~ %.1f\n', min(yaw), max(yaw));
fprintf(' : %.1f\n', mean(yaw));

%% 7. IMU data verify
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 7: IMU data verify\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

ax = data_table.accel_x;
ay = data_table.accel_y;
az = data_table.accel_z;

fprintf('X [m/s²]:\n');
fprintf(' : %.3f ~ %.3f\n', min(ax), max(ax));
fprintf(' : %.3f\n', mean(ax));

fprintf('\nY [m/s²]:\n');
fprintf(' : %.3f ~ %.3f\n', min(ay), max(ay));
fprintf(' : %.3f\n', mean(ay));

fprintf('\nZ [m/s²]:\n');
fprintf(' : %.3f ~ %.3f\n', min(az), max(az));
fprintf(' : %.3f\n', mean(az));

% verify ( 9.81 m/s²)
total_accel = sqrt(ax.^2 + ay.^2 + az.^2);
fprintf('\n [m/s²]:\n');
fprintf(' : %.3f ~ %.3f\n', min(total_accel), max(total_accel));
fprintf(' : %.3f ( 9.81 )\n', mean(total_accel));

%% 8. data verify
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 8: data verify\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

bat_pct = data_table.battery_percent;
bat_vol = data_table.battery_voltage;

fprintf(' [%%]:\n');
fprintf(' : %.1f ~ %.1f\n', min(bat_pct), max(bat_pct));
fprintf(' : %.1f\n', mean(bat_pct));
fprintf(' : %.2f%%\n', bat_pct(end) - bat_pct(1));
if height(data_table) > 1
 drain_rate = abs(bat_pct(end) - bat_pct(1)) / (height(data_table) / 60);
 fprintf(' : %.2f%%/\n', drain_rate);
end

fprintf('\n [V]:\n');
fprintf(' : %.2f ~ %.2f\n', min(bat_vol), max(bat_vol));
fprintf(' : %.2f\n', mean(bat_vol));
fprintf(' : %.2f\n', std(bat_vol));

%% 9. data quality 
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 9: data quality \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% detect (3σ )
z_scores_alt = abs(zscore(alt));
outliers = sum(z_scores_alt > 3);

fprintf(' (3σ ):\n');
fprintf(' : %d (%.2f%%)\n', outliers, (outliers/length(alt))*100);

% 
position_jumps = sqrt(diff(lat).^2 + diff(lon).^2) * 111; % km 
max_jump = max(position_jumps);
avg_jump = mean(position_jumps);

fprintf('\n :\n');
fprintf(' : %.3f km\n', avg_jump);
fprintf(' : %.3f km\n', max_jump);

% data 
miss = sum(isnan(lat) | isnan(speed) | isnan(bat_pct));
completeness = ((height(data_table) - miss) / height(data_table)) * 100;

fprintf('\ndata :\n');
fprintf(' : %d/%d (%.1f%%)\n', ...
 height(data_table) - miss, height(data_table), completeness);

% 
fprintf('\n :\n');
if completeness > 95 && outliers == 0 && max_jump < 0.1
 fprintf(' 🟢 excellent data quality\n');
 quality_score = 'A';
elseif completeness > 90 && outliers < 2 && max_jump < 1
 fprintf(' 🟢 data quality\n');
 quality_score = 'B';
elseif completeness > 80
 fprintf(' 🟡 data quality\n');
 quality_score = 'C';
else
 fprintf(' 🔴 data quality\n');
 quality_score = 'D';
end

fprintf('\n : %s\n', quality_score);

%% 10. (optional )
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('Step 10: data \n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

% 
try
 figure('Name', 'OBDM data quality ', 'NumberTitle', 'off');
 
 % 
 subplot(2,3,1);
 plot(alt, 'b-', 'LineWidth', 1.5);
 ylabel(' (m)'); xlabel('sample');
 title(' ');
 grid on;
 
 % 
 subplot(2,3,2);
 plot(speed, 'g-', 'LineWidth', 1.5);
 ylabel(' (m/s)'); xlabel('sample');
 title(' ');
 grid on;
 
 % 
 subplot(2,3,3);
 plot(bat_pct, 'r-', 'LineWidth', 1.5);
 ylabel(' (%)'); xlabel('sample');
 title(' ');
 grid on;
 ylim([0 100]);
 
 % 
 subplot(2,3,4);
 plot(roll, 'r-', 'DisplayName', 'Roll');
 hold on;
 plot(pitch, 'g-', 'DisplayName', 'Pitch');
 plot(yaw, 'b-', 'DisplayName', 'Yaw');
 ylabel(' (°)'); xlabel('sample');
 title(' ');
 legend;
 grid on;
 
 % 3D 
 subplot(2,3,5);
 plot(lon, lat, 'b-', 'LineWidth', 1.5);
 xlabel(''); ylabel('');
 title(' (2D)');
 grid on;
 axis equal;
 
 % 
 subplot(2,3,6);
 plot(total_accel, 'k-', 'LineWidth', 1.5);
 hold on;
 yline(9.81, 'r--', 'LineWidth', 1, 'DisplayName', '');
 ylabel(' (m/s²)'); xlabel('sample');
 title(' ');
 legend;
 grid on;
 
 fprintf('✓ complete/done\n');
 
catch ME
 fprintf('✗ failed: %s\n', ME.message);
end

%% 
fprintf('\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n');
fprintf('verify complete/done\n');
fprintf('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n');

fprintf('📊 data quality grade: %s\n', quality_score);
fprintf('✓ log : %s\n', log_file);
fprintf('\n :\n');
fprintf(' • \n');
fprintf(' • set\n');
fprintf(' • API \n\n');
