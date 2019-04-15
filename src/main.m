clc; clear all; close all;

rb = Robot();

while true

  rb.mobilePlatform.moveRobotToPosition();
  if (rb.mobilePlatform.velocityModulePlatform == 0)
    break;
  end

end

rb.vrep.stopSimulator();
disp('movido!!');
