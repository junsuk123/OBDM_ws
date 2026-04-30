%% ProcessManager - Process cleanup and safe termination
% Manages MATLAB and external process cleanup, resource release

classdef ProcessManager
 properties
 verbose = false;
 cleanup_log = {};
 end
 
 methods
 function obj = ProcessManager(varargin)
 % 
 % Usage: pm = ProcessManager('verbose', true);
 
 p = inputParser;
 addParameter(p, 'verbose', false, @islogical);
 parse(p, varargin{:});
 
 obj.verbose = p.Results.verbose;
 end
 
 function obj = cleanup_orphaned_matlab_processes(obj)
 % MATLAB process cleanup ( process exclude)
 
 if obj.verbose
 fprintf('[ProcessManager] MATLAB process cleanup...\n');
 end
 
 % process PID exclude MATLAB process 
 cmd = sprintf('ps aux | grep matlab | grep -v "grep" | grep -v "%d" | awk ''{print $2}''', feature('getpid'));
 [status, output] = system(cmd);
 
 if status == 0 && ~isempty(strtrim(output))
 pids = strsplit(strtrim(output), newline);
 for i = 1:length(pids)
 pid = strtrim(pids{i});
 if ~isempty(pid) && str2double(pid) > 0
 kill_cmd = sprintf('kill -9 %s 2>/dev/null', pid);
 system(kill_cmd);
 msg = sprintf(' ✓ MATLAB process terminate: PID %s', pid);
 obj.cleanup_log{end+1} = msg;
 if obj.verbose
 fprintf('%s\n', msg);
 end
 end
 end
 else
 if obj.verbose
 fprintf('[ProcessManager] No orphaned MATLAB processes found\n');
 end
 end
 end
 
 function obj = cleanup_gazebo_processes(obj)
 % Gazebo process cleanup
 
 if obj.verbose
 fprintf('[ProcessManager] Gazebo process cleanup...\n');
 end
 
 % gz/gazebo process terminate
 [status, output] = system('ps aux | grep -E "gz |gazebo" | grep -v "grep" | awk ''{print $2}''');
 
 if status == 0 && ~isempty(strtrim(output))
 pids = strsplit(strtrim(output), newline);
 count = 0;
 for i = 1:length(pids)
 pid = strtrim(pids{i});
 if ~isempty(pid) && str2double(pid) > 0
 kill_cmd = sprintf('kill -9 %s 2>/dev/null', pid);
 system(kill_cmd);
 count = count + 1;
 end
 end
 if count > 0
 msg = sprintf(' ✓ %d Gazebo process terminate', count);
 obj.cleanup_log{end+1} = msg;
 if obj.verbose
 fprintf('%s\n', msg);
 end
 end
 else
 if obj.verbose
 fprintf('[ProcessManager] No Gazebo processes found\n');
 end
 end
 end
 
 function obj = cleanup_ardupilot_processes(obj)
 % ArduPilot related process cleanup
 
 if obj.verbose
 fprintf('[ProcessManager] ArduPilot process cleanup...\n');
 end
 
 % SITL and ROS2 process terminate
 [status, output] = system('ps aux | grep -E "arduplane|ardupilot|ros2.*launch" | grep -v "grep" | awk ''{print $2}''');
 
 if status == 0 && ~isempty(strtrim(output))
 pids = strsplit(strtrim(output), newline);
 count = 0;
 for i = 1:length(pids)
 pid = strtrim(pids{i});
 if ~isempty(pid) && str2double(pid) > 0
 kill_cmd = sprintf('kill -9 %s 2>/dev/null', pid);
 system(kill_cmd);
 count = count + 1;
 end
 end
 if count > 0
 msg = sprintf(' ✓ %d ArduPilot process terminate', count);
 obj.cleanup_log{end+1} = msg;
 if obj.verbose
 fprintf('%s\n', msg);
 end
 end
 else
 if obj.verbose
 fprintf('[ProcessManager] No ArduPilot processes found\n');
 end
 end
 end
 
 function obj = cleanup_all_simulation(obj)
 % simulation related process cleanup
 
 if obj.verbose
 fprintf('[ProcessManager] Simulation process cleanup started\n');
 end
 
 obj = obj.cleanup_gazebo_processes();
 pause(0.5);
 obj = obj.cleanup_ardupilot_processes();
 pause(0.5);
 
 if obj.verbose
 fprintf('[ProcessManager] Simulation process cleanup complete\n');
 end
 end
 
 function obj = cleanup_ros2_nodes(obj, namespace)
 % namespace ROS2 cleanup
 
 if nargin < 2
 namespace = '';
 end
 
 if obj.verbose
 fprintf('[ProcessManager] ROS2 node cleanup...\n');
 end
 
 % ros2 node list and cleanup
 [status, output] = system('ros2 node list 2>/dev/null');
 if status == 0 && ~isempty(output)
 nodes = strsplit(strtrim(output), newline);
 count = 0;
 for i = 1:length(nodes)
 node = strtrim(nodes{i});
 if ~isempty(node)
 if isempty(namespace) || contains(node, namespace)
 cmd = sprintf('ros2 node kill %s 2>/dev/null', node);
 system(cmd);
 count = count + 1;
 end
 end
 end
 if count > 0
 msg = sprintf(' ✓ %d ROS2 nodes terminated', count);
 obj.cleanup_log{end+1} = msg;
 if obj.verbose
 fprintf('%s\n', msg);
 end
 end
 end
 end
 
 function obj = cleanup_temp_files(obj)
 % temporary cleanup
 
 if obj.verbose
 fprintf('[ProcessManager] temporary cleanup...\n');
 end
 
 % /tmp OBDM related cleanup
 [status, output] = system('ls -1 /tmp/obdm_* 2>/dev/null');
 if status == 0 && ~isempty(strtrim(output))
 files = strsplit(strtrim(output), newline);
 count = 0;
 for i = 1:length(files)
 file = strtrim(files{i});
 if ~isempty(file)
 system(sprintf('rm -f %s', file));
 count = count + 1;
 end
 end
 if count > 0
 msg = sprintf(' ✓ %d temporary files cleaned', count);
 obj.cleanup_log{end+1} = msg;
 if obj.verbose
 fprintf('%s\n', msg);
 end
 end
 end
 end
 
 function obj = full_cleanup(obj)
 % cleanup ( process and )
 
 fprintf('\n╔════════════════════════════════╗\n');
 fprintf('║ Full Cleanup Started           ║\n');
 fprintf('╚════════════════════════════════╝\n\n');
 
 % simulation cleanup
 obj = obj.cleanup_all_simulation();
 pause(1);
 
 % ROS2 cleanup
 obj = obj.cleanup_ros2_nodes();
 pause(0.5);
 
 % temporary cleanup
 obj = obj.cleanup_temp_files();
 
 % MATLAB process cleanup
 obj = obj.cleanup_orphaned_matlab_processes();
 
 fprintf('\n╔════════════════════════════════╗\n');
 fprintf('║ Full Cleanup Complete!         ║\n');
 fprintf('╚════════════════════════════════╝\n\n');
 end
 
 function print_cleanup_log(obj)
 % cleanup 
 fprintf('\n📋 Cleanup Log:\n');
 if isempty(obj.cleanup_log)
 fprintf(' (No cleanup operations performed)\n');
 else
 for i = 1:length(obj.cleanup_log)
 fprintf('%s\n', obj.cleanup_log{i});
 end
 end
 fprintf('\n');
 end
 end
end
