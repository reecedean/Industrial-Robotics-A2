classdef YaskawaGP4 < RobotBaseClass
    %% LinearUR3 UR3 on a non-standard linear rail created by a student

    properties(Access = public)
        plyFileNameStem = 'yaskawaGP4';
    end

    methods
        %% Define robot Function
        function self = YaskawaGP4(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = transl(0.8,0.5,0.8)

            end
            self.model.base = self.model.base.T * baseTr;
            self.PlotAndColourRobot();

        end


        %% Create the robot model
        function CreateModel(self)
            % Create the UR3 model mounted on a linear rail
            link(1) = Link('d',0.165,'a',0,'alpha', 0, 'offset',0); % 165mm from the base TR
            link(2) = Link('d',0.165,'a',0,'alpha', 0, 'offset',0); % 165mm from the joint 1 TR
            % 99.45 + 131.1/2
            link(3) = Link('d',0.260,'a',0,'alpha', 0, 'offset',0);
            link(4) = Link('d',0.015,'a',-0.0955,'alpha', 0, 'offset',0); 
            link(5) = Link('d',0,'a',-0.1945,'alpha', 0, 'offset',0); % .1945
            link(6) = Link('d',0,'a',-0.0625,'alpha', 0, 'offset',0); % .0625
            % link(5) = Link([0      0.11235   0         pi/2    0]);
            % link(6) = Link([0      0.08535   0         -pi/2   0]);

            % Incorporate joint limits
            link(1).qlim = [-170 170]*pi/180;
            link(2).qlim = [-110 130]*pi/180;
            link(3).qlim = [-65 200]*pi/180;
            link(4).qlim = [-200 200]*pi/180;
            link(5).qlim = [-123 123]*pi/180;
            link(6).qlim = [-455 4555]*pi/180;

            self.model = SerialLink(link,'name',self.name);
            

        end

    end
end