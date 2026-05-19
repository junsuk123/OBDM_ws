classdef DataLoggerTemplate < ModuleBase
    % DataLoggerTemplate - 로그 저장 템플릿

    properties
        logdir
        traj_fid = -1
        wind_fid = -1
    end

    methods
        function obj = DataLoggerTemplate()
            obj@ModuleBase('DataLogger');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            obj.logdir = fullfile('matlab','data','test',datestr(now,'yyyymmdd_HHMMSS'));
            if ~exist(obj.logdir,'dir'), mkdir(obj.logdir); end
            obj.traj_fid = fopen(fullfile(obj.logdir,'trajectory.csv'),'w');
            obj.wind_fid = fopen(fullfile(obj.logdir,'wind.csv'),'w');
        end

        function log_trajectory(obj,row)
            if obj.traj_fid > 0
                fprintf(obj.traj_fid, "%s\n", row);
            end
        end

        function log_wind(obj,row)
            if obj.wind_fid > 0
                fprintf(obj.wind_fid, "%s\n", row);
            end
        end

        function cleanup(obj)
            if obj.traj_fid > 0, fclose(obj.traj_fid); obj.traj_fid = -1; end
            if obj.wind_fid > 0, fclose(obj.wind_fid); obj.wind_fid = -1; end
        end
    end
end
