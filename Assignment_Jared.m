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
            self.environment = CafeEnvironment();
            
            % Hold on the have more plots in the figure
            hold on

            % Create linear UR5 object
            self.ur5 = LinearUR5custom();
            startJoint = [0 0 0 0 270*pi/180 -pi/2 0];
            self.ur5.model.animate(startJoint);

            self.ur5.model.teach(startJoint);
    
            
            % Create the Yaskawa
            self.yaskawa = YaskawaGP4();

            startJoint2 = [2.9671 -0.4346 0.3700  3.1416 0 0];
            self.yaskawa.model.animate(startJoint2);
            % self.yaskawa.model.teach(startJoint2);


            % Load the starting position of the cups
            self.loadCups();
            
            % self.yaskawa.model.teach()
            % Run UR5 to place the cup
            self.yaskawaMove();

            self.ur5Move();
           
        end
        function ur5Move(self)
         % Set the offset of the gripper
        gripperOffset = 0.085;
        % Set the offset of the robot
        robotOffset = 0.15;
        % Set a cup offset;
        cupOffset = 0.2;
        % Get the number of cups
        cupNum = length(self.cupsStart);
        % intial state of cup picked up is false
        cuppickedUp_ur5 = false;
            for i = 1:1
                % Loop twice for the pickup of the cup, then the drop off
                % of the cup
                for n = 1:2
                    % Define the brick start and end locations for target end effector
                    currentCupPos = self.cups{self.cupNumber}.model.base().T
                    currentCupPos = currentCupPos(1:3, 4)'
                    currentCupPos(3) = currentCupPos(3) + cupOffset;
                    % cupPos = self.cupsStart{1}(1:3);
                    cupPosend = self.cupsEnd{1}(1:3);
                    % Apply gripper and robot offset to the bricks Z coordinate
                    currentCupPos(3) = currentCupPos(3) + gripperOffset + robotOffset;
                    cupPosend(3) = cupPosend(3) + gripperOffset + robotOffset;
                    % If the brick is picked up, get to the end brick,
                    % otherwise get transform for the starting brick
                    if cuppickedUp_ur5
                        % Define the transformation matrix for the target end effector location
                        T = transl(cupPosend) * rpy2tr(0, 180, self.cupsEnd{1}(6), 'deg');
                    else
                        T = transl(currentCupPos) * rpy2tr(0, 180, self.cupsStart{1}(6), 'deg');
                    end

                    % Loop twice to get to common midpoint and then brick
                    % location
                    for l = 1:2
                        % intial joint config
                        q0 = self.ur5.model.getpos();
                        % On the first iteration, get the midpoint
                        % joint angles. On the second iteration, get the
                        % brick transform joint angles
                        T_mid = transl(0, 0, 0) * rpy2tr(0, 180, 0, 'deg');
                        if l == 1
                            %0.9076
                            q1 = [-0.0073 0.6283 -0.1902 1.2182 6.2832 -1.5621 -1.4888];
                        else
                            %q1 = self.cups{self.cupNumber}.model.base()
                            q1 = self.ur5.model.ikcon(T, q0);
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
                            %disp('animating');
                            endEff = self.ur5.model.fkine(qMatrix(j, :));
                            % Animate the robot to the next joint configuration
                            self.ur5.model.animate(qMatrix(j, :));
                            if cuppickedUp_ur5
                                CupTr(self, endEff, 2);
                            end
                            drawnow();
                        end

                    end
                    % Loop twice for the downward and upward animation of
                    % the endeffector
                    for j = 1:2
                        % on the first iteration, end effector moves down
                        if j == 1
                            cupPosend(3) = cupPosend(3) - robotOffset;
                            currentCupPos(3) = currentCupPos(3) - robotOffset;
                        % on the second iteration, end effector move up
                        elseif j == 2
                            cupPosend(3) = cupPosend(3) + robotOffset;
                            currentCupPos(3) = currentCupPos(3) + robotOffset;
                        end
                        % If the brick is picked up, get the end brick
                        % transform, otherwise get the start transform
                        if n == 2
                            T = transl(cupPosend) * rpy2tr(0, 180, 0, 'deg');
                        else
                            T = transl(currentCupPos) * rpy2tr(0, 180, 0, 'deg');
                        end
                        % Get current joint angles
                        q3 = self.ur5.model.getpos();
                        % Get the joint angles required to get to brick
                        q4 = self.ur5.model.ikcon(T, q3);
                        % Use quintic polynomial trajectory for the up and
                        % downward movements
                        tg = jtraj(q3, q4, steps);
    
                        for k = 1:steps
                            % Animate the robot to the next joint configuration
                            self.ur5.model.animate(tg(k, :));
                            endEff = self.ur5.model.fkine(tg(k, :));
                            % Display end effector transform as it reaches
                            % the brick
                            if k == steps && j == 1
                                % endEff = endEff.T
                                disp('End Effector Transfor: ')
                                disp(endEff.T);
                            end
                            if cuppickedUp_ur5
                            CupTr(self, endEff, 2);
                            end
                            drawnow();
                        end
                        if j == 1
                            cuppickedUp_ur5 = ~cuppickedUp_ur5;
                        end
                    end
                    
                end
                self.cupNumber = self.cupNumber + 1;
            end
           
        end

        % function yaskawaMove(self)
        %     % Get the number of cups
        %     cupNum = length(self.cupsStart);
        %     % Initialize bool to false
        %     cuppickedUp_yask = false;
        % 
        %     % Define two sets of waypoints
        %     Waypoint1 = [2.9671 -0.4346 0.3700 3.1416 0 0]
        %     Waypoint2 = [1.2514 2.2689 -2.2864 3.1416 0 0]
        % 
        %     for i = 1:2
        %         % Loop for the pickup of the cup and the drop-off of the cup
        %         % Toggle between Waypoint1 and Waypoint2
        %         if cuppickedUp_yask
        %             WaypointStart = Waypoint1;
        %             WaypointEnd = Waypoint2;
        %         else
        %             WaypointStart = Waypoint2;
        %             WaypointEnd = Waypoint1;
        %         end
        % 
        %         % Define the brick start and end locations for the target end effector
        %         cupPos = self.cupsStart{1}(1:3);
        %         cupPosend = self.cupsMid(1:3);
        % 
        %         % If the brick is picked up, get to the end brick,
        %         % otherwise get the transform for the starting brick
        %         if cuppickedUp_yask
        %             % Define the transformation matrix for the target end effector location
        %             T = transl(cupPosend) * rpy2tr(0, 90, 0, 'deg');
        %         else
        %             T = transl(cupPos) * rpy2tr(0, 90, 0, 'deg');
        %         end
        % 
        %         % Use a common mid transform between the start and end brick position 
        %         T_mid = transl(1, 0, 1.5) * rpy2tr(0, 180, 0, 'deg');
        % 
        %         % Move to the starting waypoint
        %         q0 = self.yaskawa.model.getpos();
        %         q_waypoint_start = WaypointStart %self.yaskawa.model.ikcon(transl(WaypointStart) * rpy2tr(WaypointStart(4:6), 'deg'), q0);
        % 
        %         % Use trapezoidal Velocity profile to move from q0 to q_waypoint_start
        %         steps = 25;
        %         s = lspb(0, 1, steps);
        %         qMatrix = nan(steps, 6);
        %         for j = 1:steps
        %             qMatrix(j, :) = (1 - s(j)) * q0 + s(j) * q_waypoint_start;
        %         end
        % 
        %         % Iterate through the trajectory and animate the robot's motion
        %         for j = 1:steps
        %             disp('animating');
        %             endEff = self.yaskawa.model.fkine(qMatrix(j, :));
        %             % Animate the robot to the next joint configuration
        %             self.yaskawa.model.animate(qMatrix(j, :));
        %             if cuppickedUp_yask
        %                 CupTr(self, endEff);
        %             end
        %             drawnow();
        %         end
        % 
        %         % Move to the cup start or end position
        %         q0 = q_waypoint_start;
        %         q_cup_position = self.yaskawa.model.ikcon(T, q0);
        % 
        %         % Use trapezoidal Velocity profile to move from q0 to q_cup_position
        %         steps = 25;
        %         s = lspb(0, 1, steps);
        %         qMatrix = nan(steps, 6);
        %         for j = 1:steps
        %             qMatrix(j, :) = (1 - s(j)) * q0 + s(j) * q_cup_position;
        %         end
        % 
        %         % Iterate through the trajectory and animate the robot's motion
        %         for j = 1:steps
        %             disp('animating');
        %             endEff = self.yaskawa.model.fkine(qMatrix(j, :));
        %             % Animate the robot to the next joint configuration
        %             self.yaskawa.model.animate(qMatrix(j, :));
        %             if cuppickedUp_yask
        %                 CupTr(self, endEff);
        %             end
        %             drawnow();
        %         end
        % 
        %         % Toggle the cuppickedUp_yask state for the next iteration
        %         cuppickedUp_yask = ~cuppickedUp_yask;
        %     end
        %     self.cupNumber = self.cupNumber + 1;
        % end
        function yaskawaMove(self)
            % Get the number of cups
            cupNum = length(self.cupsStart);
            % set bool to false
            cuppickedUp_yask = false;
            % Loop for number of cups
            for i = 1:1
            % Loop twice for the pickup of the cup, then the drop off
            % of the cup
                for n = 1:2
                    % Define the cup start and end locations for target end effector
                    cupPos = self.cupsStart{1}(1:3);
                    cupPosend = self.cupsMid(1:3);
                    % If the brick is picked up, get to the end brick,
                    % otherwise get transform for the starting brick
                    if cuppickedUp_yask
                        % Define the transformation matrix for the target end effector location
                        T = transl(cupPosend) * rpy2tr(0, 180, 0, 'deg');
                    else
                        T = transl(cupPos) * rpy2tr(0, 180, 0, 'deg');
                    end

                    % Loop multiple times for waypoints
                    for l = 1:5
                        % intial joint config
                        q0 = self.yaskawa.model.getpos();
                        % We are using joint angles to get to the waypoint
                        % transforms
                        if n == 1
                            if l == 1
                                %First way point
                                q1 = [2.9671 -0.4346 0.3700 3.1416 0 0];
                            elseif l == 2
                                % Get to cup start transform
                                q1 = self.yaskawa.model.ikcon(T, q0);
                                q1 = [q1(1:3) 3.1416 0 0];
                                %q1 = [1.2514 2.2689 -2.2864 3.1416 0 0];
                            elseif l == 3
                                % Toggle the pick up cup bool
                                cuppickedUp_yask = ~cuppickedUp_yask;
                                % Move back to first waypoint
                                q1 = [2.9671 -0.4346 0.3700 3.1416 0 0];
                            elseif l == 4
                                %Tilt base around to new position 2nd
                                %waypoint
                                q1 = [1.2863 -0.4346 0.3700 3.1416 0 0];
                            end
                        elseif n == 2
                            if l == 1
                                % Move down to next 3rd waypoint
                                q1 = [1.2514 2.2689 -1.9373 3.1416 -0.3438 0];
                            elseif l == 2
                                % Move to 4th waypoint under the coffee
                                % machine
                                q1 = [2.0420 2.2689 -1.9373 3.1416 -0.3438 0];
                            elseif l == 3
                                % Move back to 3rd waypoint
                                q1 = [1.2514 2.2689 -1.9373 3.1416 -0.3438 0];
                            elseif l == 4
                                % Move down to place the cup 
                                q1 = [1.2514 2.2689 -1.7628 3.1416 -0.5149 0];
                            else
                                % Move back to 2nd waypoint
                                q1 = [1.2863 -0.4346 0.3700 3.1416 0 0];
                            end
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
                            %disp('animating');
                            endEff = self.yaskawa.model.fkine(qMatrix(j, :));
                            % Animate the robot to the next joint configuration
                            self.yaskawa.model.animate(qMatrix(j, :));
                            if cuppickedUp_yask
                                CupTr(self, endEff, 1);
                            end
                            drawnow();
                        end
                        if l == 4  && n == 2
                            cuppickedUp_yask = ~cuppickedUp_yask;
                        end
                        if l == 2 && n == 2
                            disp("PREPARING COFFEE, PLEASE WAIT :)");
                            pause(0);
                        end
                    end
                end
            end
        end

        function CupTr(self, endEffTr, robot)
            if robot == 1
                CupPos = endEffTr.T;
                zDir = CupPos(1:3, 3);
                xDir = CupPos(1:3, 1);
                globalOffset = zDir * 0.07;
                globalOffset = globalOffset + (xDir * 0.05);
                CupPos(1:3, 4) = CupPos(1:3, 4) + globalOffset;
                self.cups{self.cupNumber}.model.base = CupPos * troty(-pi/2);
                self.cups{self.cupNumber}.model.animate(CupPos);
            elseif robot == 2
                CupPos = endEffTr.T;
                CupPos(3,4) = CupPos(3,4) - 0.3;
                self.cups{self.cupNumber}.model.base = CupPos * trotx(pi);
                self.cups{self.cupNumber}.model.animate(CupPos);

            end 
        end


        function loadCups(self)
            % Set the start location of the bricks 
            self.cupsStart = {
                [1.25 -0.1 1.25,0,0,0]
                [1.25,0.1,1.25,0,0,0]
                [1.25,0.3,1.25,0,0,0]
                [1.25,0.5,1.25,0,0,0]
                };

            % Plot each of the start cups at their location 
            for i = 1:length(self.cupsStart)
                cupPos = transl(self.cupsStart{i}(1:3));
                self.cups{i} = Cup(cupPos);
            end
            self.cupsEnd = {
                [-0.3,-1.2,0.95,0,0,0]
                [-0.5,-1.2,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                };
            self.cupsMid = [1.0,-0.7,0.8,0,0,0];
        end
    end
end