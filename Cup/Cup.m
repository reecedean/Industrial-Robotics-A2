classdef Cup < RobotBaseClass
%% LinearUR3 UR3 on a non-standard linear rail created by a student

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