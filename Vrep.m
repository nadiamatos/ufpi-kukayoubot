classdef Vrep < handle

  properties

    vrep = []; clientID = -1;
    objCamera = 0; objJoints = zeros(1, 6); % 6 é do efetuador.
    objCubes = zeros(1, 3);
    objAreas = zeros(1, 3);
    objWheel = zeros(1, 4);

  end

  methods


    function self = Vrep()

      [self.vrep, self.clientID] = self.startCommunicationSimulator();
      self.getObjectsSimulation();
      self.playSimulator();

    end


    function [vrep, clientID] = startCommunicationSimulator(self)

      % Function to start communication between MATLAB and V-rep.

      CHANNEL = 19997;

      vrep = remApi('remoteApi');
      vrep.simxFinish(-1);
      clientID = vrep.simxStart('127.0.0.1', CHANNEL, true, true, 5000, 5);

      if (~clientID) disp('Connected to remote API server'); end

    end


    function stopCommunicationSimulator(self)

    	% Function to stop communication between MATLAB and V-rep.

      %self.vrep.simxSynchronousTrigger(self.clientID);
      self.vrep.simxFinish(-1);
      self.vrep.delete();

    end


    function playSimulator(self)

      if (~self.clientID)

        %self.vrep.simxSynchronous(self.clientID, true);
        self.vrep.simxStartSimulation(self.clientID, self.vrep.simx_opmode_oneshot);

      end

    end


    function stopSimulator(self)

      if (~self.clientID)

        self.vrep.simxStopSimulation(self.clientID, self.vrep.simx_opmode_oneshot);
        self.stopCommunicationSimulator();

      end

    end


    function getObjectsSimulation(self)

      [~, self.objCamera] = self.vrep.simxGetObjectHandle(self.clientID, 'camera', self.vrep.simx_opmode_oneshot_wait);

      [~, self.objCubes(1)] = self.vrep.simxGetObjectHandle(self.clientID, 'cube_blue', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objCubes(2)] = self.vrep.simxGetObjectHandle(self.clientID, 'cube_green', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objCubes(3)] = self.vrep.simxGetObjectHandle(self.clientID, 'cube_red', self.vrep.simx_opmode_oneshot_wait);

      [~, self.objAreas(1)] = self.vrep.simxGetObjectHandle(self.clientID, 'area_blue', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objAreas(2)] = self.vrep.simxGetObjectHandle(self.clientID, 'area_green', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objAreas(3)] = self.vrep.simxGetObjectHandle(self.clientID, 'area_red', self.vrep.simx_opmode_oneshot_wait);

      [~, self.objWheel(1)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_fl', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objWheel(2)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_fr', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objWheel(3)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_rl', self.vrep.simx_opmode_oneshot_wait);
      [~, self.objWheel(4)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_rr', self.vrep.simx_opmode_oneshot_wait);

      %[~, self.objJoints(6)] = self.vrep.simxGetObjectHandle(self.clientID, 'youBotGripperJoint1', self.vrep.simx_opmode_oneshot_wait);

      for i = 1 : 5
        [~, self.objJoints(i)] = self.vrep.simxGetObjectHandle(self.clientID, ...
                                                          ['youBotArmJoint' num2str(i-1)], ...
                                                          self.vrep.simx_opmode_oneshot_wait);
      end

    end


    function position = getPositionObject(self, reference)

      [~, position] = self.vrep.simxGetObjectPosition(self.clientID, reference, -1, self.vrep.simx_opmode_oneshot_wait);

    end


    function setPositionJoint(self, reference, positionAngular)

      self.vrep.simxSetJointTargetPosition(self.clientID, reference, deg2rad(positionAngular), self.vrep.simx_opmode_oneshot);

    end


    function positionAngular = getPositionJoint(self, reference)

      [~, positionAngular] = self.vrep.simxGetJointPosition(self.clientID, reference, self.vrep.simx_opmode_streaming);
      positionAngular = rad2deg(positionAngular);

    end


  end

end
