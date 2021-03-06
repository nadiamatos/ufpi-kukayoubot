classdef Vrep < handle

  properties

    vrep = []; clientID = -1;
    idCamera = 0;
    idJoints = zeros(1, 6); % 6 é do efetuador.
    idCubes = zeros(1, 3);
    idAreas = zeros(1, 3);
    idWheel = zeros(1, 4);
    idPlatform = 0;
    idDisc = 0;
    idOrigemPlatform = 0;

  end

  methods

    function self = Vrep()
      self.startCommunicationSimulator();
      self.getObjectsSimulation();
      self.playSimulator();
    end

    function startCommunicationSimulator(self)
      % Function to start communication between MATLAB and V-rep.
      CHANNEL = 19997;
      self.vrep = remApi('remoteApi');
      self.vrep.simxFinish(-1);
      self.stopCommunicationSimulator();
      self.clientID = self.vrep.simxStart('127.0.0.1', CHANNEL, true, true, 5000, 5);
      % self.vrep.simxSynchronous(self.clientID, true);
      if (~self.clientID) disp('Connected to remote API server'); end
    end

    function stopCommunicationSimulator(self)
    	% Function to stop communication between MATLAB and V-rep.
      % self.vrep.simxSynchronousTrigger(self.clientID);
      self.vrep.simxFinish(-1);
      % self.vrep.simxSynchronous(self.clientID, false);
      self.vrep.delete();
    end

    function pauseCommunication(self, isForPauseCommunication)
      self.vrep.simxPauseCommunication(self.clientID, isForPauseCommunication);
    end

    function playSimulator(self)
      if (~self.clientID)
        % self.vrep.simxSynchronous(self.clientID, true);
        self.vrep.simxStartSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait);
        disp('Simulacao iniciada');
      end
    end

    function stopSimulator(self)
      self.vrep.simxStopSimulation(self.clientID, self.vrep.simx_opmode_oneshot_wait);
      self.stopCommunicationSimulator();
      disp('Simulacao encerrada');
    end

    function getObjectsSimulation(self)

      % [~, self.idCamera] = self.vrep.simxGetObjectHandle(self.clientID, 'camera', self.vrep.simx_opmode_oneshot_wait);
      % [~, self.idCubes(1)] = self.vrep.simxGetObjectHandle(self.clientID, 'cube_blue', self.vrep.simx_opmode_oneshot_wait);
      % [~, self.idCubes(2)] = self.vrep.simxGetObjectHandle(self.clientID, 'cube_green', self.vrep.simx_opmode_oneshot_wait);
      % [~, self.idCubes(3)] = self.vrep.simxGetObjectHandle(self.clientID, 'cube_red', self.vrep.simx_opmode_oneshot_wait);
      % [~, self.idAreas(1)] = self.vrep.simxGetObjectHandle(self.clientID, 'area_blue', self.vrep.simx_opmode_oneshot_wait);
      % [~, self.idAreas(2)] = self.vrep.simxGetObjectHandle(self.clientID, 'area_green', self.vrep.simx_opmode_oneshot_wait);
      % [~, self.idAreas(3)] = self.vrep.simxGetObjectHandle(self.clientID, 'area_red', self.vrep.simx_opmode_oneshot_wait);

      [~, self.idDisc] = self.vrep.simxGetObjectHandle(self.clientID, 'Disc', self.vrep.simx_opmode_oneshot_wait);
      [~, self.idOrigemPlatform] = self.vrep.simxGetObjectHandle(self.clientID, 'youBot_center', self.vrep.simx_opmode_oneshot_wait);
      [~, self.idPlatform] = self.vrep.simxGetObjectHandle(self.clientID, 'youBot', self.vrep.simx_opmode_oneshot_wait);
      [~, self.idWheel(1)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_fl', self.vrep.simx_opmode_oneshot_wait);
      [~, self.idWheel(2)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_fr', self.vrep.simx_opmode_oneshot_wait);
      [~, self.idWheel(3)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_rl', self.vrep.simx_opmode_oneshot_wait);
      [~, self.idWheel(4)] = self.vrep.simxGetObjectHandle(self.clientID, 'rollingJoint_rr', self.vrep.simx_opmode_oneshot_wait);
      %[~, self.objJoints(6)] = self.vrep.simxGetObjectHandle(self.clientID, 'youBotGripperJoint1', self.vrep.simx_opmode_oneshot_wait);
      for i = 1 : 5
        [~, self.idJoints(i)] = self.vrep.simxGetObjectHandle(self.clientID, ['youBotArmJoint' num2str(i-1)], self.vrep.simx_opmode_oneshot_wait);
      end
    end

    function position = getPositionObject(self, id)
      [~, position] = self.vrep.simxGetObjectPosition(self.clientID, id, -1, self.vrep.simx_opmode_oneshot_wait);
    end

    function setPositionJoint(self, id, positionAngular)
      self.vrep.simxSetJointTargetPosition(self.clientID, id, deg2rad(positionAngular), self.vrep.simx_opmode_oneshot);
    end

    function positionAngular = getPositionJoint(self, id)
      [~, positionAngular] = self.vrep.simxGetJointPosition(self.clientID, id, self.vrep.simx_opmode_streaming);
      positionAngular = rad2deg(positionAngular);
    end

    function setVelocityJoint(self, id, velocity)
      self.vrep.simxSetJointTargetVelocity(self.clientID, id, velocity, self.vrep.simx_opmode_oneshot);
    end

    function angles = getOrientationObject(self, id)
      [~, angles] = self.vrep.simxGetObjectOrientation(self.clientID, id, -1, self.vrep.simx_opmode_streaming);
      angles = rad2deg(angles); % 1 - alpha, 2 - beta, 3 - gamma
      if (angles(1) == 0)
        if (angles(2) > 0)
          angles(2) = 0;
        elseif (angles(2) < 0)
          angles(2) = 180;
        end
      elseif(angles(1) < 0)
        angles(2) = 90 - angles(2);
      elseif(angles(1) > 0)
        angles(2) = 270 + angles(2);
      end
      angles = angles(2); % beta angle of the simulation, in degree
    end

  end

end
