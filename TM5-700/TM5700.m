classdef TM5700 < handle
    %% TM5-700 robot

    properties(Access = public)              
        stlFileNameStem = 'TM5-700'; % .ply files pulled from LinearUR5 & UR3
    end
    
    methods
        %% Constructor
        function self = TM5700(baseTr)
            disp('Starting');
            self.CreateModel();
            if nargin < 1			
				baseTr = eye(4)
                
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            disp('plotting');
            self.PlotAndColourRobot();    
            disp('finished plotting')
        end
           

%% Create the robot model
        function CreateModel(self)   
            % Create the TM5-700 model 
            link(1) = Link('d',0.1451,'a',0,'alpha',-pi/2,'qlim',deg2rad([-270 270]), 'offset',0);
            link(2) = Link('d',0,'a',0.329,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',-pi/2);
            link(3) = Link('d',0,'a',0.3115,'alpha',0,'qlim', deg2rad([-155 155]), 'offset', 0);
            link(4) = Link('d',-0.1222,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            link(5) = Link('d',0.106,'a',0,'alpha',pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            link(6) = Link('d',0.1144,'a',0,'alpha',0,'qlim',deg2rad([-270,270]), 'offset', 0);
            
            self.model = SerialLink(link);
            disp('created model')
        end
    end
end
