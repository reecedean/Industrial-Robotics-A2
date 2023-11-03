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
<<<<<<< HEAD
        function CreateModel(self)
            link(1) = Link('alpha', 0, 'a', 0, 'd', 0);
            self.model = SerialLink(link,'name',self.name);
        end
    %% InitiliseRobotPlot
=======
    function CreateModel(self)
        link(1) = Link('alpha', 0, 'a', 0, 'd', 0);
        self.model = SerialLink(link,'name',self.name);
    end

     %% InitiliseRobotPlot
>>>>>>> 712a29e7619b89192b6eb94c27adbecbf8c66656
        % First and only time to plot the robot
        function h = InitiliseRobotPlot(self)
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();
            % Display robot
            [ax,by] = view;
            
            self.workspace = [-1 1 -1 1 -1 1];

<<<<<<< HEAD
            self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by],'nowrist');%,'notiles');            
=======
            self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by], 'nowrist');%,'notiles');            
>>>>>>> 712a29e7619b89192b6eb94c27adbecbf8c66656

            % Check if a single surface has been added by plot3d
            if self.CountTiledFloorSurfaces() - initialSurfaceCount == 1
                self.surfaceAdded = true;
            end

            % Check if a light needs to be added
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
<<<<<<< HEAD
                camlight;
=======
                camlight
>>>>>>> 712a29e7619b89192b6eb94c27adbecbf8c66656
                self.lightAdded = true;
            end

            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
        end
    end
end