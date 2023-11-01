classdef Gripper < RobotBaseClass
    %% LinearUR3 gripper FINGER
    properties(Access = public)   
        plyFileNameStem = 'gripper';
        finger1
        finger2
    end
    
    methods
%% Constructor
        function self = Gripper(baseTr)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                % self.useTool = useTool;
                % toolTrData = load([toolFilename,'.mat']);
                % self.toolTr = toolTrData.tool;
                % self.toolFilename = [toolFilename,'.ply'];
            end
            self.workspace = [-10 10 -10 10 -10 10];
            
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr
            %self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
            axis equal;
            axis auto;
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            % Set DH Parameters for the gripper model
            link(1) = Link([0     0       0       0    1]); % PRISMATIC Link
            link(2) = Link([0      0       0       pi       1]);

            link(1).qlim = [0 0.06];
            link(2).qlim = [-0.12 0];


            
            self.model = SerialLink(link,'name',self.name);
        end 

        function closeGripper(self)
            q_open = [0, 0];
            % Animate the gripper to the open position
            self.model.animate(q_open);
            self.model.animate(q_open);

        end

        %% InitiliseRobotPlot
        % First and only time to plot the robot
        function h = InitiliseRobotPlot(self)
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();
            % Display robot
            [ax,by] = view;
            
            roughMinMax = sum(abs(self.model.d) + abs(self.model.a));
            self.workspace = [-1 1 -1 1 -1 1];

            self.model.plot3d(self.homeQ,'noarrow','workspace',self.workspace,'view',[ax,by]);%,'notiles');            

            % Check if a single surface has been added by plot3d
            if self.CountTiledFloorSurfaces() - initialSurfaceCount == 1
                self.surfaceAdded = true;
            end

            % Check if a light needs to be added
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
                self.lightAdded = true;
            end

            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');
        end

        %% Modified PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and colour them in if data is available 
        % Modified by changing the workspace dimensions
        function PlotAndColourRobot(self)
            if isempty(self.homeQ)
                self.homeQ = zeros(1,self.model.n);
            end

            if exist([self.plyFileNameStem,'.mat'],'file') == 2 && exist([self.plyFileNameStem,'Link0.ply'],'file') ~= 2
                warning('There are no ply files for the links but there is a mat file. You should use PlotAndColourRobotMat to create and colour a 3D robot model plot. I am doing this for you now.')
                self.PlotAndColourRobotMat()
                return;
            end

            for linkIndex = 0:self.model.n
                if self.useTool && linkIndex == self.model.n
                    if ~isempty(self.toolFilename)
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.toolFilename],'tri'); %#ok<AGROW>
                    else
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'Tool.ply'],'tri'); %#ok<AGROW>
                    end
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,'Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                end
                
                % Obtain faceData and vertexData for the current link and save to the cell.
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            h = self.InitiliseRobotPlot();
            if 1 < length(h)
                self.MultirobotWarningMessage();
                h = h{1};
            end

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
                try 
                     vertexColours = [plyData{linkIndex+1}.vertex.red ...
                                     , plyData{linkIndex+1}.vertex.green ...
                                     , plyData{linkIndex+1}.vertex.blue]/255;
                    
                catch ME_1
                    disp(ME_1);
                    disp('No vertex colours in plyData');
                    try 
                         vertexColours = [plyData{linkIndex+1}.face.red ...
                                     , plyData{linkIndex+1}.face.green ...
                                     , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        disp(['Also, no face colours in plyData, so using a default colour: ',num2str(vertexColours)]);
                    end
                end
                
                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
            drawnow();
        end

    end
end