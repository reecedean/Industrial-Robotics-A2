classdef TM5700 < handle
    %% TM5-700 robot

    properties(Access = public)              
        % plyFileNameStem = 'LinearUR3'; % .ply files pulled from LinearUR5 & UR3
    end
    
    methods
        %% Constructor
        function self = TM5700(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
<<<<<<< HEAD
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
=======
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot(); 
        end
>>>>>>> 20af6bd013a8f9b733c0b03cdfc21a893d6a53de

            drawnow
        end
%% Create the robot model
        function CreateModel(self)   
            % Create the TM5-700 model 
            link(1) = Link('d',0.1451,'a',0,'alpha',-pi/2,'qlim',deg2rad([-270 270]), 'offset',0);
            link(2) = Link('d',0,'a',0329,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',-pi/2);
            link(3) = Link('d',0,'a',0.3115,'alpha',0,'qlim', deg2rad([-155 155]), 'offset', 0);
            link(4) = Link('d',-0.1222,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            link(5) = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            link(6) = Link('d',0.1144,'a',0,'alpha',0,'qlim',deg2rad([-270,270]), 'offset', 0);
            
            self.model = SerialLink(link,'name',self.name);

        end
    end
end
