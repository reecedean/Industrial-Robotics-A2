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
            self.model.base = baseTr
            self.PlotAndColourRobot();
        end
        
%% Create the Cup model
    function CreateModel(self)
        link(1) = Link('alpha', 0, 'a', 0, 'd', 0);
        self.model = SerialLink(link,'name',self.name);
    end
    end
end