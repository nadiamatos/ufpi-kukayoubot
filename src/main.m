clc; clear all; close all;

rb = Robot();

rb.mobilePlatform.moveRobotToPosition();

rb.vrep.stopSimulator();
disp('movido!!');
