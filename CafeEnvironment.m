classdef CafeEnvironment < handle
    properties
        tableHandle % Store a handle to the table object
    end
    methods
        function self = CafeEnvironment()
            self.placeTable()
        end
        
        function placeTable(self)
            self.tableHandle = PlaceObject('Assignment2_ws3.ply', [0,0,0;0,0,0]);
        end
    end
end