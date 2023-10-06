classdef Assignment2 < handle
    properties
        ur5
        yaskawa
        environment
        cups
        cupsEnd
    end
    
    methods 
        function self = Assignment2()
            % Set up the workspace environment
            self.environment = CafeEnvironment()
            
            % Hold on the have more plots in the figure
            hold on
            % Create linear UR3 object
           
            self.ur5 = LinearUR5custom();
            startJoint = [0 0 0 0 270*pi/180 -pi/2 0];
            self.ur5.model.animate(startJoint);

            self.ur5.model.teach(startJoint)

            % Create the Yaskawa
            self.yaskawa = YaskawaGP4();


            % Load the starting position of the cups
            self.loadCups()

            % Run UR5 to place the cup
            %self.PlaceCup()
           
        end
        function PlaceCup(self)
        % Get the number of cups
        cupNum = length(self.cups);
        % intial state of cup picked up is false
        cuppickedUp = false;
        currentCupHandle = []; % Handle for graphical cup object
            for i = 1:5
                % Loop twice for the pickup of the brick, then the drop off
                % of the brick
                for n = 1:2
                    % Define the brick start and end locations for target end effector
                    cupPos = self.cups{1}(1:3);
                    cupPosend = self.cupsEnd{1}(1:3);
                    % If the brick is picked up, get to the end brick,
                    % otherwise get transform for the starting brick
                    if cuppickedUp
                        % Define the transformation matrix for the target end effector location
                        T = transl(cupPosend) * rpy2tr(0, 180, self.cupsEnd{1}(6), 'deg');
                    else
                        T = transl(cupPos) * rpy2tr(0, 180, self.cups{1}(6), 'deg');
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
                            % Animate the robot to the next joint configuration
                            self.ur5.model.animate(qMatrix(j, :))
        
                            % Update the position of the cup when it's picked up
                            if cuppickedUp
                                effPos = self.ur5.model.fkine(qMatrix(j, :)).t;
                                effPos(3) = effPos(3) - 0.2;  % Offset in Z direction
                                % Delete the previous graphical representation of the cup
                                if ~isempty(currentCupHandle)
                                    delete(currentCupHandle);
                                end
                                currentCupHandle = PlaceObject('Cup4.ply', effPos');
                            end
                            drawnow();
                        end
                        
                    end
                                % If placing the cup down, remove the currentCupHandle
                    if cuppickedUp
                        pause(3);
                        delete(currentCupHandle);
                        currentCupHandle = [];
                    end

                    cuppickedUp = ~cuppickedUp
                end
            end
        end
        



        function loadCups(self)
            % Set the start location of the bricks 
            self.cups = {
                [0.7 1.0 1.25,0,0,0]
                [0.9,1.0,1.25,0,0,0]
                [1.1,1.0,1.25,0,0,0]
                [1.3,1.0,1.25,0,0,0]
                [0.75,-0.5,0.8,0,0,0]
                };

            % Plot each of the start cups at their location 
            for i = 1:length(self.cups)
                cupPos = self.cups{i};
                
                cupxyz = cupPos(1:3);
                cuprpy = cupPos(6);
                % Convert to radians for MATLAB trig functions
                cuprpy = deg2rad(cuprpy);
                
                h = PlaceObject('Cup4.ply', cupxyz);
                % self.brick_handles{i} = h;
                % 
                % 
                % verts = get(h, 'Vertices');
                % verts = verts - brickxyz;  
                % rotationMatrix = trotz(brickrpy);
                % verts = [verts, ones(size(verts, 1), 1)] * rotationMatrix';
                % verts = verts(:, 1:3) + brickxyz;  
                % set(h, 'Vertices', verts);
            end
            self.cupsEnd = {
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                };
        end

    end
end