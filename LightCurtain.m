classdef LightCurtain < handle

    properties
        normals;
        midPoints;
        xLims = [-0.6, -0.6]; 
        yLims = [-2.4, 0.4];
        zLims = [0.7, 1.7];  
    end

    methods 
        function self = LightCurtain()
            self.normals = zeros(1, 3);
            self.midPoints = zeros(1, 3);

            [tempX, tempY, tempZ] = self.normalVector(self.xLims(1), self.xLims(2), self.yLims(1), self.yLims(2));

            self.normals(1,1) = tempX;
            self.normals(1,2) = tempY;
            self.normals(1,3) = tempZ;

            [X, Y, Z] = self.MidPoint(self.xLims(1), self.xLims(2), self.yLims(1), self.yLims(2), self.zLims(1), self.zLims(2));

            self.midPoints(1,1) = X;
            self.midPoints(1,2) = Y;
            self.midPoints(1,3) = Z;
        end    

        % Calculates the normal of the light curtains
        function [Nx, Ny, Nz] = normalVector(self, px1, px2, py1, py2)
            Nx = px1 - px2;
            Ny = py1 - py2;
            Nz = 0;
        end

        function [x, y, z] = MidPoint(self, px1, px2, py1, py2, pz1, pz2)
            % make array of midpoints
            % Calculate midpoints for each set of limits
            x = (px1 + px2)/2;
            y = (py1 + py2)/2;
            z = (pz1 + pz2)/2;
        end
    end
end