clc; clear all; close all;

format long;

vrep = Vrep();



a = [0 0 0 0 0]

for i = 1 : 5

  vrep.setPositionJoint(vrep.objJoints(i), a(i))
  pause(1)

  vrep.getPositionJoint(vrep.objJoints(i))
  
end
