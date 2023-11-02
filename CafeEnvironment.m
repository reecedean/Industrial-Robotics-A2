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
            main.placeLightCurtain;
        end
        
        function placeBartop(main)
            hold on;
            bartop = PlaceObject('@Ply Files/Bartop_Silver_Navy.ply', [0,0,0;0,0,0]);
        end

        function placeTables(main)
            table1 = PlaceObject('@Ply Files/Table_Silver.ply', [0,0,0;0,0,0]);
            table2 = PlaceObject('@Ply Files/Table2_Silver.ply', [0,0,0;0,0,0]);
        end

        function placeShelves(main)
            shelf1 = PlaceObject('@Ply Files/Shelf_grey.ply', [0,0,0;0,0,0]);
        end

        function placeGate(main)
            gate = PlaceObject('@Ply Files/Gate_Black.ply', [0,0,0;0,0,0]);
        end

        function placeLightCurtain(main)
            lightcurtain1 = PlaceObject('@Ply Files/lightcurtain.ply', [-0.6,0.4,0.7]);
            lightcurtain2 = PlaceObject('@Ply Files/lightcurtain2.ply', [-0.6,-2.4,0.7]);
        end

        function placeEmergencyStop(main)
            eStop = PlaceObject('@Ply Files/Emergency_Stop.ply', [0,0,0;0,0,0]);
        end

        function placeCoffeeMachine(main)
            CoffeeMachine = PlaceObject('@Ply Files/Coffee_Machine.ply', [0,0,0;0,0,0]);
        end

        function placeWalls(main)
            % Concrete Floor
            surf([-3,-3;1.5,1.5] ...
                ,[-2.5, 1.6; -2.5, 1.6] ...
                ,[0.01,0.01;0.01,0.01] ...
                ,'CData',imread('@Images/concrete.jpg') ...
                ,'FaceColor','texturemap');
            
            % Brick Wall behind coffee machine
            surf([1.5,1.5; 1.5,1.5] ...
                ,[-2.5, 1.6; -2.5, 1.6] ...
                ,[0.01, 0.01;2.5,2.5] ...
                ,'CData',imread('@Images/brick_wall_black.jpg') ...
                ,'FaceColor','texturemap');

            % Wall next to gate
            surf([-3,1.5; -3, 1.5] ...
                ,[1.6, 1.6; 1.6, 1.6] ...
                ,[0.01, 0.01; 2.5, 2.5] ...
                ,'CData',imread('@Images/GreenWall.jpg') ...
                ,'FaceColor','texturemap');

        end
    end
end