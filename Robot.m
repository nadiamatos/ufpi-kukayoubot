classdef Robot < handle

  properties

    joint = zeros(1, 5);
    wheel = zeros(1, 4);

  end

  methods

    function self = Robot()

    end


    function forwardKinematicArm(self)

    end


    function inverseKinematicArm(self)

    end


    function forwardKinematicPlatform(self)

    end


    function inverseKinematicPlatform(self)

    end


    function moveRobotToPosition(self)

    end


end
