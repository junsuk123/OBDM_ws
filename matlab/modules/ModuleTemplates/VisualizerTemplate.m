classdef VisualizerTemplate < ModuleBase
    % VisualizerTemplate - 실시간 시각화 템플릿

    properties
        fig
        axs
    end

    methods
        function obj = VisualizerTemplate()
            obj@ModuleBase('Visualizer');
        end

        function init(obj, config)
            init@ModuleBase(obj, config);
            if usejava('desktop')
                obj.fig = figure('Name', 'Realtime Visualizer');
                obj.axs = axes(obj.fig);
            end
        end

        function update_once(obj, data)
            % data는 구조체형식으로 전달 (예: data.airspeed, data.pose)
            if ~isempty(obj.axs)
                cla(obj.axs);
                plot(obj.axs, data.timestamp, data.airspeed);
                drawnow limitrate
            end
        end

        function cleanup(obj)
            if ~isempty(obj.fig) && isvalid(obj.fig)
                close(obj.fig);
            end
        end
    end
end
