classdef InitialiseCups < handle
    properties(Access = public)
        cupsPlace
    end
    
    methods
        function self = InitialiseCups()
            self.loadCups();
            self.cupPositions(); 

        end

        function loadCups(self)
            % Set the start location of the bricks 
            self.cupsPlace = {
                [1.25 -0.1 1.25,0,0,0]
                % [1.25,0.1,1.25,0,0,0]
                % [1.25,0.3,1.25,0,0,0]
                % [1.25,0.5,1.25,0,0,0]
                };

            % Plot each of the start cups at their location 
            for i = 1:length(self.cupsPlace)
                cupPos = transl(self.cupsPlace{i}(1:3));
                self.cups{i} = Cup(cupPos);
            end
            self.cupsEnd = {
                [-0.3,-1.2,0.95,0,0,0]
                [-0.5,-1.2,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                [-0.5,-1.5,0.8,0,0,0]
                };
        end
    end
end
