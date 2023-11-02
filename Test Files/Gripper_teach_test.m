clear all 
close all


robot = Gripper(true)

% robot = UR3(true)

axis([-0.5 0.5 -0.5 0.5 -0.75 0.75]);
axis equal;
robot.model.teach([0, 0]);

for i=1:10
    robot.gripperControl("close")

    pause(2)
    robot.gripperControl("open")
    pause(2)
end
