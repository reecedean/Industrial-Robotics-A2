classdef CafeEnvironment < handle
    properties
        tableHandle % Store a handle to the table object
    end
    methods
        function main = CafeEnvironment()
            axis("equal")
            main.placeBartop;
            main.placeTables;
            main.placeShelves;
            main.placeGate;
            main.placeEmergencyStop;
            main.placeCoffeeMachine;
            main.placeWalls;
        end
        
        function placeBartop(main)
            hold on;
            bartop = PlaceObject('Ply Files/Bartop_Silver_Navy.ply', [0,0,0;0,0,0]);
        end

        function placeTables(main)
            table1 = PlaceObject('Ply Files/Table_Silver.ply', [0,0,0;0,0,0]);
            table2 = PlaceObject('Ply Files/Table2_Silver.ply', [0,0,0;0,0,0]);
        end

        function placeShelves(main)
            shelf1 = PlaceObject('Ply Files/Shelf_grey.ply', [0,0,0;0,0,0]);
        end

        function placeGate(main)
            gate = PlaceObject('Ply Files/Gate_Black.ply', [0,0,0;0,0,0]);
        end

        function placeEmergencyStop(main)
            eStop = PlaceObject('Ply Files/Emergency_Stop.ply', [0,0,0;0,0,0]);
        end

        function placeCoffeeMachine(main)
            CoffeeMachine = PlaceObject('Ply Files/Coffee_Machine.ply', [0,0,0;0,0,0]);
        end

        function placeWalls(main)
            surf([-3,-3;1.5,1.5] ...
                ,[-2.5, 1.6; -2.5, 1.6] ...
                ,[0.01,0.01;0.01,0.01] ...
                ,'CData',imread('concrete.jpg') ...
                ,'FaceColor','texturemap');

            surf([1.5,1.5; 1.5,1.5] ...
                ,[-2.5, 1.6; -2.5, 1.6] ...
                ,[0.01, 0.01;2.5,2.5] ...
                ,'CData',imread('brick_wall_black.jpg') ...
                ,'FaceColor','texturemap');

        end
    end
end