function sendPositionJointsArm(self)
  for i = 1 : length(self.joint)
    self.vrep.setPositionJoint(self.vrep.idJoints(i), self.joint(i));
  end
end
