classdef Person < RobotBaseClass
%% Robot model for person to demonstrate light curtain sensing

    properties(Access = public)              
        plyFileNameStem = 'personMaleCasual'; 
    end
    
    methods
%% Define robot Function 
        function self = Person(baseTr) 
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);			
            end
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();
        end
        
%% Create the Cup model
    function CreateModel(self)
        link(1) = Link('alpha', 0, 'a', 0, 'd', 0);
        self.model = SerialLink(link,'name',self.name);
    end
    end
end