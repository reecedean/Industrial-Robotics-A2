clear all 
close all


robot = YaskawaGP4(true)

% robot = UR3(true)

axis([-0.5 0.5 -0.5 0.5 0 0.75]);
axis equal;
robot.model.teach();