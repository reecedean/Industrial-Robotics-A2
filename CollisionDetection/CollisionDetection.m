classdef CollisionDetection 

    % Define an empty struct array
    prisms = struct('TopCorner', {}, 'BottomCorner', {});
    z
    % Add prisms to the struct array
    prisms(end+1) = struct('TopCorner', [x1, y1, z1], 'BottomCorner', [x2, y2, z2]);
    prisms(end+1) = struct('TopCorner', [x3, y3, z3], 'BottomCorner', [x4, y4, z4]);
    % ... add more prisms as needed
    
    % Accessing the prisms
    firstPrismTop = prisms(1).TopCorner;
    firstPrismBottom = prisms(1).BottomCorner;

%     prisms = {
%     struct('vertices', table1Vertices, 'faces', table1Faces, 'faceNormals', table1FaceNormals),
%     struct('vertices', table2Vertices, 'faces', table2Faces, 'faceNormals', table2FaceNormals),
%     % Add more tables as needed
% };

    % Get the transform of every joint (i.e. start and end of every link)
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) *
    transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

end

%% Test
classdef CollisionDetect < handle

    properties
        rectangles
    end

methods

    function self = CollisionDetect
        self.rectangles = { 
            struct('lower', [0.55,-1,0.6], 'upper', [1.5,1.5,0.8]), ... Bench 1
            struct('lower', [-1,-1.8,0.6], 'upper', [1.5,-1,0.8]), ... Bench 2
            struct('lower', [1.5,1.5,0], 'upper', [1.55,-2.5,2.5]), ... Wall
            struct('lower', [1.2,-0.2,1.25], 'upper', [1.5,1.5,1.3]), ... Shelf
            };
    end

    function draw(self, axis_h)
        if nargin < 2
            axis_h = gca;
        end
        hold(axis_h, 'on');
    
        for i = 1 :length(CollisionDetect)
            rectangle = self.rectangles{i};
            self.RectangularPrism(rectangle.lower,rectangle.upper,rectangle.plotOptions,axis_h)
        end
    end

            for i = 1: length(self.collisionRectangles)
                [tempVertex, tempFace, tempFaceNormals] = RectangularPrism(self.collisionRectangles{i}.lower, self.collisionRectangles{i}
                self.rectangularPrismData{i,1} = tempVertex;
                self.rectangularPrismData{i,2} = tempFace;
                self.rectangularPrismData{i,3} = tempFaceNormals;
            end


