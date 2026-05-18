%% ProcessManager - Process cleanup and safe termination
% Manages MATLAB and external process cleanup, resource release

classdef ProcessManager
	properties
		verbose = false;
		cleanup_log = {};
	end

	methods
		function obj = ProcessManager(varargin)
			% Usage: pm = ProcessManager('verbose', true);
			p = inputParser;
			addParameter(p, 'verbose', false, @islogical);
			parse(p, varargin{:});

			obj.verbose = p.Results.verbose;
		end

		function obj = terminate_process_patterns(obj, patterns, label, grace_s)
			if nargin < 4 || isempty(grace_s)
				grace_s = 2;
			end
			if isempty(patterns)
				return;
			end

			if obj.verbose
				fprintf('[ProcessManager] %s termination requested...\n', label);
			end

			for i = 1:length(patterns)
				system(sprintf('pkill -TERM -f "%s" 2>/dev/null || true', patterns{i}));
			end

			pause(grace_s);

			for i = 1:length(patterns)
				system(sprintf('pkill -KILL -f "%s" 2>/dev/null || true', patterns{i}));
			end

			msg = sprintf(' ✓ %s cleanup signal sequence sent', label);
			obj.cleanup_log{end+1} = msg;
			if obj.verbose
				fprintf('%s\n', msg);
			end
		end

		function obj = cleanup_orphaned_matlab_processes(obj)
			if obj.verbose
				fprintf('[ProcessManager] MATLAB process cleanup...\n');
			end

			cmd = sprintf('ps aux | grep matlab | grep -v "grep" | grep -v "%d" | awk ''{print $2}''', feature('getpid'));
			[status, output] = system(cmd);

			if status == 0 && ~isempty(strtrim(output))
				pids = strsplit(strtrim(output), newline);
				count = 0;
				for i = 1:length(pids)
					pid = strtrim(pids{i});
					if ~isempty(pid) && str2double(pid) > 0
						system(sprintf('kill -9 %s 2>/dev/null', pid));
						count = count + 1;
					end
				end
				if count > 0
					msg = sprintf(' ✓ %d MATLAB process terminate', count);
					obj.cleanup_log{end+1} = msg;
					if obj.verbose
						fprintf('%s\n', msg);
					end
				end
			elseif obj.verbose
				fprintf('[ProcessManager] No orphaned MATLAB processes found\n');
			end
		end

		function obj = cleanup_gazebo_processes(obj)
			if obj.verbose
				fprintf('[ProcessManager] Gazebo process cleanup...\n');
			end

			gazebo_patterns = {
				'gz sim';
				'gzserver';
				'gzclient';
				'ign gazebo';
				'gazebo';
				'libgz';
			};
			obj = obj.terminate_process_patterns(gazebo_patterns, 'Gazebo', 3);
		end

		function obj = cleanup_ardupilot_processes(obj)
			if obj.verbose
				fprintf('[ProcessManager] ArduPilot process cleanup...\n');
			end

			ardupilot_patterns = {
				'arducopter';
				'arduplane';
				'ardupilot';
				'mavproxy';
				'micro_ros_agent';
				'ros2 launch';
			};
			obj = obj.terminate_process_patterns(ardupilot_patterns, 'ArduPilot', 3);
		end

		function obj = cleanup_relay_processes(obj)
			if obj.verbose
				fprintf('[ProcessManager] ROS2 relay process cleanup...\n');
			end

			relay_patterns = {
				'relay';
				'topic_tools';
			};
			obj = obj.terminate_process_patterns(relay_patterns, 'ROS2 relay', 1);
		end

		function obj = cleanup_sim_ports(obj)
			if obj.verbose
				fprintf('[ProcessManager] Checking simulation ports...\n');
			end

			ports = {'5760', '5501', '9002', '2019'};

			for i = 1:length(ports)
				port = ports{i};
				cmd = sprintf('netstat -tulpn 2>/dev/null | grep ":%s " | awk ''{print $NF}'' | cut -d/ -f1', port);
				[status, pids_str] = system(cmd);

				if status == 0 && ~isempty(strtrim(pids_str))
					pids = strsplit(strtrim(pids_str), newline);
					for j = 1:length(pids)
						pid = strtrim(pids{j});
						if ~isempty(pid) && str2double(pid) > 0
							system(sprintf('kill -9 %s 2>/dev/null', pid));
							if obj.verbose
								fprintf('[ProcessManager] Killed process using port %s (PID: %s)\n', port, pid);
							end
						end
					end
				end
			end

			pause(1);
		end

		function obj = cleanup_all_simulation(obj)
			if obj.verbose
				fprintf('[ProcessManager] Simulation process cleanup started\n');
			end

			obj = obj.cleanup_relay_processes();
			pause(0.3);
			obj = obj.cleanup_gazebo_processes();
			pause(0.3);
			obj = obj.cleanup_ardupilot_processes();
			pause(0.3);
			obj = obj.cleanup_sim_ports();
			pause(0.5);
			obj = obj.cleanup_temp_files();
			pause(0.5);

			if obj.verbose
				fprintf('[ProcessManager] Simulation process cleanup complete\n');
			end
		end

		function obj = cleanup_ros2_nodes(obj, namespace)
			if nargin < 2
				namespace = '';
			end

			if obj.verbose
				fprintf('[ProcessManager] ROS2 node cleanup...\n');
			end

			[status, output] = system('ros2 node list 2>/dev/null');
			if status == 0 && ~isempty(output)
				nodes = strsplit(strtrim(output), newline);
				count = 0;
				for i = 1:length(nodes)
					node = strtrim(nodes{i});
					if ~isempty(node) && (isempty(namespace) || contains(node, namespace))
						system(sprintf('ros2 node kill %s 2>/dev/null', node));
						count = count + 1;
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
			if obj.verbose
				fprintf('[ProcessManager] temporary cleanup...\n');
			end

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
			fprintf('\n╔════════════════════════════════╗\n');
			fprintf('║ Full Cleanup Started           ║\n');
			fprintf('╚════════════════════════════════╝\n\n');

			obj = obj.cleanup_all_simulation();
			pause(1);

			obj = obj.cleanup_ros2_nodes();
			pause(0.5);

			obj = obj.cleanup_temp_files();
			obj = obj.cleanup_orphaned_matlab_processes();

			fprintf('\n╔════════════════════════════════╗\n');
			fprintf('║ Full Cleanup Complete!         ║\n');
			fprintf('╚════════════════════════════════╝\n\n');
		end

		function print_cleanup_log(obj)
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
