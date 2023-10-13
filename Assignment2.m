classdef Assignment2 < handle
    properties
        ur5
        yaskawa
        environment
        cups
        cupsEnd
        cupsStart
        cupNumber = 1
        cupsMid
    end
    
    methods 
        function self = Assignment2()
            % Set up the workspace environment
            self.environment = CafeEnvironment()
            
            % Hold on the have more plots in the figure
            hold on

            % Create linear UR5 object
            self.ur5 = LinearUR5custom();
            startJoint = [0 0 0 0 270*pi/180 -pi/2 0];
            self.ur5.model.animate(startJoint);

            self.ur5.model.teach(startJoint)

            % Create the Yaskawa
            self.yaskawa = YaskawaGP4();


            % Load the starting position of the cups
            self.loadCups()
            
            self.
            % Run UR5 to place the cup
            %self.yaskawaMove()
           
        end
        function ur5Move(self)
           
        % Get the number of cups
        cupNum = length(self.cupsStart);
        % intial state of cup picked up is false
        cuppickedUp_ur5 = false;
            for i = 1:1
                % Loop twice for the pickup of the cup, then the drop off
                % of the cup
                for n = 1:2
                    % Define the brick start and end locations for target end effector
                    cupPos = self.cupsStart{1}(1:3);
                    cupPosend = self.cupsEnd{1}(1:3);
                    % If the brick is picked up, get to the end brick,
                    % otherwise get transform for the starting brick
                    if cuppickedUp
                        % Define the transformation matrix for the target end effector location
                        T = transl(cupPosend) * rpy2tr(0, 180, self.cupsEnd{1}(6), 'deg');
                    else
                        T = transl(cupPos) * rpy2tr(0, 180, self.cupsStart{1}(6), 'deg');
                    end
                    % Use a common mid transform between the start and end
                    % brick position 
                    % T_mid = transl(0, 0, 0.7) * rpy2tr(0, 180, 0, 'deg');

                    % Loop twice to get to common midpoint and then brick
                    % location
                    for l = 1:2
                        % intial joint config
                        q0 = self.ur5.model.getpos();
                        % On the first iteration, get the midpoint
                        % joint angles. On the second iteration, get the
                        % brick transform joint angles
                        if l == 1
                            q1 = [-0.0073 0.9076 -0.1902 1.2182 6.2832 -1.5621 -1.4888];
                            % q1 = self.robot.model.ikcon(T_mid, q0)
                        else
                            q1 = self.ur5.model.ikcon(T, q0)
                        end
        
                        % Use trapezoidal Velocity profile to move from q0 to q1
                        steps = 25;
                        s = lspb(0,1,steps);
                        qMatrix = nan(steps,7);
                        for j = 1:steps
                            qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
                        end
                        
                        % Iterate through the trajectory and animate the robot's motion
                        for j = 1:steps
                            disp('animating');
                            endEff = self.ur5.model.fkine(qMatrix(j, :));
                            % Animate the robot to the next joint configuration
                            self.ur5.model.animate(qMatrix(j, :));
                            if cuppickedUp_ur5
                                CupTr(self, endEff);
                            end
                            drawnow();
                        end
                        
                    end

                    cuppickedUp_ur5 = ~cuppickedUp_ur5;
                end
                self.cupNumber = self.cupNumber + 1;
            end

        
           
        end

        function yaskawaMove(self)
            % Get the number of cups
            cupNum = length(self.cupsStart);
            % set bool to false
            cuppickedUp_yask = false;
            for i = 1:1
            % Loop twice for the pickup of the cup, then the drop off
            % of the cup
                for n = 1:2
                    % Define the brick start and end locations for target end effector
                    cupPos = self.cupsStart{1}(1:3);
                    cupPosend = self.cupsMid(1:3);
                    % If the brick is picked up, get to the end brick,
                    % otherwise get transform for the starting brick
                    if cuppickedUp_yask
                        % Define the transformation matrix for the target end effector location
                        T = transl(cupPosend) * rpy2tr(0, 180, self.cupsMid(6), 'deg');
                    else
                        T = transl(cupPos) * rpy2tr(0, 180, self.cupsStart{1}(6), 'deg');
                    end
                    % Use a common mid transform between the start and end
                    % brick position 
                    % T_mid = transl(0, 0, 0.7) * rpy2tr(0, 180, 0, 'deg');
    
                    % Loop twice to get to common midpoint and then brick
                    % location
                    for l = 1:2
                        % intial joint config
                        q0 = self.ur5.model.getpos();
                        % On the first iteration, get the midpoint
                        % joint angles. On the second iteration, get the
                        % brick transform joint angles
                        if l == 1
                            q1 = [-0.0073 0.9076 -0.1902 1.2182 6.2832 -1.5621 -1.4888];
                            % q1 = self.robot.model.ikcon(T_mid, q0)
                        else
                            q1 = self.ur5.model.ikcon(T, q0)
                        end
    
                        % Use trapezoidal Velocity profile to move from q0 to q1
                        steps = 25;
                        s = lspb(0,1,steps);
                        qMatrix = nan(steps,6);
                        for j = 1:steps
                            qMatrix(j,:) = (1-s(j))*q0 + s(j)*q1;
                        end
                        
                        % Iterate through the trajectory and animate the robot's motion
                        for j = 1:steps
                            disp('animating');
                            endEff = self.yaskawa.model.fkine(qMatrix(j, :));
                            % Animate the robot to the next joint configuration
                            self.yaskawa.model.animate(qMatrix(j, :));
                            if cuppickedUp_yask
                                CupTr(self, endEff);
                            end
                            drawnow();
                        end
                    end
                    cuppickedUp_yask = ~cuppickedUp_yask;
                end

                
            end
            self.cupNumber = self.cupNumber + 1;
        end
        
        function CupTr(self, endEffTr)
            CupPos = self.cups{self.cupNumber}.model.getpos()
            self.cups{self.cupNumber}.model.base = endEffTr.T * trotx(pi)
            self.cups{self.cupNumber}.model.animate(CupPos)
        end 


        function loadCups(self)
            % Set the start location of the bricks 
            self.cupsStart = {
                [0.7 0.5 1.25,0,0,0]
                [0.9,0.5,1.25,0,0,0]
                [1.1,0.5,1.25,0,0,0]
                [1.3,0.5,1.25,0,0,0]
                };

            % Plot each of the start cups at their location 
            for i = 1:length(self.cupsStart)
                cupPos = transl(self.cupsStart{i}(1:3));
                self.cups{i} = Cup(cupPos);
            end
            self.cupsEnd = {
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                };
            self.cupsMid = [0.75,-0.5,0.8,0,0,0]
        end
    end
end