classdef Cup < RobotBaseClass
%% Robot class for Cup Model

    properties(Access = public)              
        plyFileNameStem = 'Cup'; 
    end
    
    methods
%% Define robot Function 
        function self = Cup(baseTr) 
			self.CreateModel();
            if nargin < 1			
				baseTr = transl(0,0,-0.1);			
            end
            self.model.base = baseTr;
            self.PlotAndColourRobot();
        end
        
%% Create the Cup model
        function CreateModel(self)
            link(1) = Link('alpha', 0, 'a', 0, 'd', 0);
            self.model = SerialLink(link,'name',self.name);
        end

     %% InitiliseRobotPlot
        % First and only time to plot the robot
        function h = InitiliseRobotPlot(self)
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();
            % Display robot
            [ax,by] = view;
            
            self.workspace = [-1 1 -1 1 -1 1];

            self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by],'nowrist');%,'notiles');            

            % Check if a single surface has been added by plot3d
            if self.CountTiledFloorSurfaces() - initialSurfaceCount == 1
                self.surfaceAdded = true;
            end

            % Check if a light needs to be added
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
                camlight
                self.lightAdded = true;
            end

            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
        end
    end
end