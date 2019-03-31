clc; clear all; close all; format short;

rb = Robot();

while true

  rb.moveRobotToPosition();

  % rb.vrep.getOrientationObject(rb.vrep.idPlatform)

  %rb.positionTargetPlatform = rb.vrep.getPositionObject(rb.vrep.idDisc);
  %beta = rb.calculateAngleBetweenPoints([0 0 0], rb.positionTargetPlatform)

end
% rb.vrep.stopSimulator();
