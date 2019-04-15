classdef Robot < handle

  properties
    vrep = [];
    mobilePlatform = [];
    manipulator = [];
  end

  methods

    function self = Robot()
      self.vrep = Vrep();
      self.mobilePlatform = MobilePlatform(self.vrep);
      % ma = Manipulator(self.vrep);
    end

  end

end
