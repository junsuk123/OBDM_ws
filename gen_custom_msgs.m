%% Generate MATLAB custom messages for ardupilot_msgs
% This script generates MATLAB bindings for custom ROS messages

clear all; close all; clc;

ws_root = pwd; % Current workspace directory
msg_gen_path = fullfile(ws_root, 'ardu_ws_local', 'src', 'ardupilot', 'Tools', 'ros2', 'matlab_msg_gen');
pkg_names = {'ardupilot_msgs'};

fprintf('═════════════════════════════════════\n');
fprintf('Generating MATLAB Custom Messages\n');
fprintf('═════════════════════════════════════\n');
fprintf('Workspace: %s\n', ws_root);
fprintf('Target path: %s\n', msg_gen_path);
fprintf('Packages: %s\n', strjoin(pkg_names, ', '));
fprintf('\n');

% Create target directory if it doesn't exist
if ~isfolder(msg_gen_path)
    mkdir(msg_gen_path);
    fprintf('✓ Created message generation path\n');
end

try
    % Generate messages
    fprintf('⏳ Generating MATLAB messages...\n');
    ros2genmsg(msg_gen_path, pkg_names);
    fprintf('✓ Message generation complete\n\n');
    
    % Check generated files
    inst_path = fullfile(msg_gen_path, 'glnxa64', 'install');
    if isfolder(inst_path)
        fprintf('Generated files location: %s\n', inst_path);
        system(sprintf('find %s -type f -name "*.m" | wc -l', inst_path));
        fprintf('\n✓ Custom messages ready for use\n');
    else
        warning('Installation path not found: %s', inst_path);
    end
    
catch ME
    fprintf('[ERROR] Message generation failed:\n%s\n', ME.message);
    rethrow(ME);
end

fprintf('═════════════════════════════════════\n\n');
exit(0);
