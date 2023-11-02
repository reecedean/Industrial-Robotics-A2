classdef Assignment2 < handle
    properties
        ur5
        yaskawa
        gui
        environment
        cups
        cupsEnd
        cupsStart
        cupNumber = 1
        ur5Mid
        cupPick
        yaskState
        wayPoints
        cuppickedUp_yask
        cuppickedUp_ur5
        ur5State
    end
    
    methods 
        function self = Assignment2()
            % Set up the workspace environment
            self.environment = CafeEnvironment();
            
            % Hold on the have more plots in the figure
            hold on

            % Create linear UR5 object
            self.ur5 = LinearUR5custom();
            startJoint = [-0.5 0 0 0 270*pi/180 -pi/2 0];
            self.ur5.model.animate(startJoint);

            self.ur5.model.teach(startJoint);
    
            
            % Create the Yaskawa
            self.yaskawa = YaskawaGP4();

            startJoint2 = [2.9671 -0.4346 0.3700  3.1416 0 0];
            self.yaskawa.model.animate(startJoint2);

            % Initialise the user interface
            self.gui = GUI(self.ur5, self.yaskawa);
            self.gui.assignment2 = self
            self.gui.updateEndEffectorPositionLabel()

            % Load the starting position of the cups
            self.loadCups();

            % Run yaskawa
            self.yaskawaMove();
           
           
        end

        function ur5Move(self)
            % Get the number of cups
            cupNum = length(self.cupsStart);

            % Set bool to false
            self.cuppickedUp_ur5 = false;
            % Set the offset of the gripper
            gripperOffset = 0.085;
            % Set the offset of the robot
            robotOffset = 0.15;
            % Set a cup offset;
            cupOffset = 0.2;
            % Get the current position of the cup
            currentCupPos = self.cups{self.cupNumber}.model.base().T
            % Switch the transform
            currentCupPos = currentCupPos(1:3, 4)'
            % apply the offset for the cup
            currentCupPos(3) = currentCupPos(3) + cupOffset + gripperOffset;
            cupPosend = self.cupsEnd(1:3);
            % Apply gripper and robot offset to the bricks Z coordinate
            cupPosend(3) = cupPosend(3) + gripperOffset;
            % Set to state of the ur5 to 1
            self.ur5State = 1;

            for i = 1:8
                switch self.ur5State
                    case 1
                        % Joint Angles for common midpoint
                        q1 = [-0.0073 0.6283 -0.1902 1.2182 6.2832 -1.5621 -1.4888];
                    case 2
                        % Get to position above the current cup position
                        currentCupPos(3) = currentCupPos(3) + robotOffset;
                        targetPos = transl(currentCupPos) * rpy2tr(0, 180, 0, 'deg');
                    case 3
                        % Move down onto the cup
                        currentCupPos(3) = currentCupPos(3) - robotOffset;
                        targetPos = transl(currentCupPos) * rpy2tr(0, 180, 0, 'deg');
                    case 4
                        % Switch cup flag
                        self.cuppickedUp_ur5 = ~self.cuppickedUp_ur5
                        currentCupPos(3) = currentCupPos(3) + robotOffset;
                        targetPos = transl(currentCupPos) * rpy2tr(0, 180, 0, 'deg');
                    case 5
                        % Joint Angles for common midpoint
                        q1 = [-0.0073 0.6283 -0.1902 1.2182 6.2832 -1.5621 -1.4888];
                    case 6
                        % Move to high end position placement of cup 
                        cupPosend(3) = cupPosend(3) + robotOffset;
                        targetPos = transl(cupPosend) * rpy2tr(0, 180, 0, 'deg');
                    case 7
                        % Move down to place cup
                        cupPosend(3) = cupPosend(3) - robotOffset;
                        targetPos = transl(cupPosend) * rpy2tr(0, 180, 0, 'deg');
                    case 8
                        % Move back up
                        self.cuppickedUp_ur5 = ~self.cuppickedUp_ur5
                        cupPosend(3) = cupPosend(3) + robotOffset;
                        targetPos = transl(cupPosend) * rpy2tr(0, 180, 0, 'deg');
                end
                if self.ur5State == 1 || self.ur5State == 5
                    steps = 20
                    % Get current joint angles
                    q0 = self.ur5.model.getpos();
                    qFinal = q1;
                    qMatrix = jtraj(q0, qFinal, steps);
                else
                    steps = 20
                    % Get current joint angles
                    q0 = self.ur5.model.getpos();
                    qFinal = self.ur5.model.ikcon(targetPos, q0);
                    qMatrix = jtraj(q0, qFinal, steps);
                end
                Animate(self, qMatrix,1)
                % Add one to the state
                self.ur5State = self.ur5State + 1;
            end

        end
        function yaskawaMove(self)
            % Get the number of cups
            cupNum = length(self.cupsStart);
            
            % Set bool to false
            self.cuppickedUp_yask = false;
            
            % Loop for the number of cups
            for i = 1:cupNum
                % Define the cup start and end locations for the target end effector
                cupPos = self.cupsStart{i}(1:3);
                cupPos(1) = cupPos(1) - 0.1;
                cupPos(3) = cupPos(3) + 0.1;
                self.yaskState = 1;
                for j = 1:10
                    if self.yaskState == 10
                        self.yaskState = 1
                    end
                    switch self.yaskState
                        case 1
                            % Move to its starting point
                            targetPos = transl(self.wayPoints{1}) * rpy2tr(0, 90, 0, 'deg');
                        case 2
                            % Move to the cup position
                            targetPos = transl(cupPos) * rpy2tr(0, 90, 0, 'deg');
                            for r = 1:2
                                if r == 1
                                    % First move to point in yz
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'yz', 2);
                                else
                                    % then move to point in x
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'x', 2);
                                end
                                Animate(self,qMatrix,2)
                            end
                        case 3
                            % Pick up the cup
                            self.cuppickedUp_yask = ~self.cuppickedUp_yask
                            % Move back to starting waypoint position
                            targetPos = transl(self.wayPoints{1}) * rpy2tr(0, 90, 0, 'deg');
                            for r = 1:2
                                if r == 1
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'x', 2);
                                else
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'yz', 2);
                                end
                                Animate(self,qMatrix,2)
                            end
                        case 4
                            % Move to safe waypoint (move base around)
                            targetPos = transl(self.wayPoints{2}) * rpy2tr(0, 90, 0, 'deg');
                        case 5
                            % Move to safe waypoint (move down)
                            targetPos = transl(self.wayPoints{3}) * rpy2tr(90, 90, 0, 'deg');
                        case 6
                            % Move under the coffe machine
                            targetPos = transl(self.wayPoints{4}) * rpy2tr(90, 90, 0, 'deg');
                            for r = 1:2
                                if r == 1
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'yz', 2);
                                else
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'x', 2);
                                end
                                Animate(self,qMatrix,2)
                            end
                        case 7
                            % Move back to waypoint 3
                            targetPos = transl(self.wayPoints{3}) * rpy2tr(90, 90, 0, 'deg');
                        case 8
                            % Move down to place cup
                            targetPos = transl(self.wayPoints{5}) * rpy2tr(90, 90, 0, 'deg');
                        case 9
                            % Place Cup
                            self.cuppickedUp_yask = ~self.cuppickedUp_yask
                            % Move back to waypoint 2 (first in yz, then in
                            % x to avoid hitting the cup
                            targetPos = transl(self.wayPoints{2}) * rpy2tr(90, 90, 0, 'deg');
                            for r = 1:2
                                if r == 1
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'yz', 2);
                                else
                                    qMatrix = ResolvedMotionRateControl(self, targetPos, 'x', 2);
                                end
                                Animate(self,qMatrix,2)
                            end
                            
                          
                    end
        
                    if self.yaskState == 1 || self.yaskState == 5 || self.yaskState == 8 || self.yaskState == 9
                        steps = 20
                        % Get current joint angles
                        q0 = self.yaskawa.model.getpos();
                        qFinal = self.yaskawa.model.ikcon(targetPos, q0);
                        qMatrix = jtraj(q0, qFinal, steps);
                    else
                        qMatrix = ResolvedMotionRateControl(self, targetPos, 'horizontal', 2);
                    end
                    if self.yaskState ~= 2 || self.yaskState ~= 3 || self.yaskState ~= 6 || self.yaskState ~=9 
                        % Animate the robot to the RMRC trajectory
                        Animate(self, qMatrix,2)
                    end
                    self.yaskState = self.yaskState + 1;
                end
                ur5Move(self)
                self.cupNumber = self.cupNumber + 1;
            end  
        end
        function Animate(self, qMatrix, robotType)
            if robotType == 1
                robot = self.ur5;
                cupPicked = self.cuppickedUp_ur5
            elseif robotType == 2
                robot = self.yaskawa;
                cupPicked = self.cuppickedUp_yask
            end
            for j = 1:size(qMatrix, 1)
                endEff = robot.model.fkine(qMatrix(j, :));
                % Animate the robot to the next joint configuration
                robot.model.animate(qMatrix(j, :));
                self.gui.updateEndEffectorPositionLabel();
                if cupPicked
                    CupTr(self, endEff, robotType);
                end
                drawnow();
                
            end
        end
        function CupTr(self, endEffTr, robot)
            if robot == 1
                CupPos = endEffTr.T;
                CupPos(3,4) = CupPos(3,4) - 0.3;
                self.cups{self.cupNumber}.model.base = CupPos * trotx(pi);
                self.cups{self.cupNumber}.model.animate(CupPos);
            elseif robot == 2
                CupPos = endEffTr.T;
                zDir = CupPos(1:3, 3);
                xDir = CupPos(1:3, 1);
                globalOffset = zDir * 0.07;
                globalOffset = globalOffset + (xDir * 0.05);
                CupPos(1:3, 4) = CupPos(1:3, 4) + globalOffset;
                self.cups{self.cupNumber}.model.base = CupPos * troty(-pi/2);
                self.cups{self.cupNumber}.model.animate(CupPos);

            end 
        end


        function loadCups(self)
            % Set the start location of the bricks 
            self.cupsStart = {
                [1.25 -0.1 1.25,0,0,0]
                [1.25,0.1,1.25,0,0,0]
                [1.25,0.3,1.25,0,0,0]
                };

            % Plot each of the start cups at their location 
            for i = 1:length(self.cupsStart)
                cupPos = transl(self.cupsStart{i}(1:3));
                self.cups{i} = Cup(cupPos);
            end
            self.cupsEnd = [-0.3,-1.2,0.95,0,0,0]

            self.ur5Mid = [0.140 -0.682 1.442];

            self.wayPoints = {
                [1.047 0.05646 1.404]
                [0.7296 -0.1407 1.404]
                [0.70 -0.3903 0.8835]
                [1.05 -0.37 0.92]
                [0.70 -0.40 0.80]
                }
        end
        function updateUR5(self, jointAngles)
            disp(jointAngles)
            self.ur5.model.animate(jointAngles)
            self.gui.updateEndEffectorPositionLabel()
        end
        function updateYaskawa(self, jointAngles)
            disp(jointAngles)
            self.yaskawa.model.animate(jointAngles)
            self.gui.updateEndEffectorPositionLabel()
        end
        function handleSystemState(self)
            while ~self.gui.systemRunning
                pause(0.1); 
            end
        end
        function robotJogging(self, val, dir, robot)
            if dir == 1
                plane = 'x'
            elseif dir == 2
                plane = 'y'
            else
                plane = 'z'
            end
            
            if robot == 1
                q0 = self.ur5.model.getpos()
                endEff = self.ur5.model.fkine(q0);
                endEff = endEff.T
                disp(endEff(dir,4))
                endEff(dir,4) = endEff(dir,4) + val
                qMatrix = ResolvedMotionRateControl(self, endEff, plane, robot);
                self.ur5.model.animate(qMatrix)
                self.gui.updateEndEffectorPositionLabel()
            else
                q0 = self.yaskawa.model.getpos()
                endEff = self.yaskawa.model.fkine(q0);
                endEff = endEff.T
                disp(endEff(dir,4))
                endEff(dir,4) = endEff(dir,4) + val
                qMatrix = ResolvedMotionRateControl(self, endEff, plane, robot);
                self.yaskawa.model.animate(qMatrix)
                self.gui.updateEndEffectorPositionLabel()
            end

        end
    end
end