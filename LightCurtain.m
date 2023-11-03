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

            self.normals(1) = tempX;
            self.normals(2) = tempY;
            self.normals(3) = tempZ;

            [X, Y, Z] = self.MidPoint(self.xLims(1), self.xLims(2), self.yLims(1), self.yLims(2), self.zLims(1), self.zLims(2));

            self.midPoints(1) = X;
            self.midPoints(2) = Y;
            self.midPoints(3) = Z;

            self.plotLightCurtain();
            
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
        function plotLightCurtain(self)
    % Define corners of the light curtain plane based on xLims, yLims, and zLims
    x = [self.xLims(1), self.xLims(2), self.xLims(2), self.xLims(1)];
    y = [self.yLims(1), self.yLims(1), self.yLims(2), self.yLims(2)];
    z = [self.zLims(1), self.zLims(1), self.zLims(2), self.zLims(2)]; % Adjusted to form a plane in x-y frame spanning zLims range
    
    % Plot the light curtain using fill3
    % figure; % Create a new figure
    fill3(x, y, z, 'c'); % Fill the 3D polygon. 'c' denotes cyan color, you can change this.
    alpha(0.5); % Make it slightly transparent for better visualization
        end
    end
end