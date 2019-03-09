clc; clear all; close all; format short;

rb = Robot();

%rb.moveRobotToPosition([0 0 0]);
while true

  %rb.anglePlatform = rb.vrep.getOrientationObject(rb.vrep.idPlatform);

  %disp('Here');
  %disp(rb.anglePlatform);
  %disp(90 - rb.anglePlatform);
  %disp(mod(rb.anglePlatform + 180, 360) - 180);

  rb.foe

end
% rb.vrep.stopSimulator();
