classdef Robot < handle

  properties

    % Basead in: http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications

    % mensuraments in meters.
    distanceBetweenWheelCenterRobot = 0.15023;
    distanceBetweenWheelSomeSide = 0.2355;
    radiusWheel = 0.05;
    radiusPlatform = 0.571;
    velocityPlatform = zeros(3, 1); % [vx; vy; vz]
    velocityWheels = zeros(4, 1);   % [omega1; omega2; omega3; omega4]
    joint = zeros(1, 5);
    wheel = zeros(1, 4);
    positionCurrentPlatform = zeros(1, 3);
    positionTargetPlatform = zeros(1, 3);
    vrep = [];

  end

  methods

    function self = Robot()

      self.vrep = Vrep();

    end

    function forwardKinematicArm(self)

    end

    function inverseKinematicArm(self)

    end

    function forwardKinematicPlatform(self, velocityWheels)

    end

    function calculateVelocityPlatform(self)

      dt = 1;
      self.velocityPlatform = (self.positionTargetPlatform - self.positionCurrentPlatform)/dt;
      self.velocityPlatform(3) = 0;
      % a velocidade de rotacao da plataforma tem que considerar o angulo atual da mesma
      % e o angulo a ser obtido.
      self.controlLimitsVelocityPlatform();

    end

    function controlLimitsVelocityPlatform(self)

      LIMIT_VELOCITY_SUPERIOR = 0.8;  % m/s

      for i = 1 : length(self.velocityPlatform)
        if (self.velocityPlatform(i) > LIMIT_VELOCITY_SUPERIOR)
          self.velocityPlatform(i) = LIMIT_VELOCITY_SUPERIOR;
        end
      end

    end

    function controlLimitsVelocityWheel(self)

      LIMIT_VELOCITY_SUPERIOR = 1.57; % rad/sec

      for i = 1 : length(self.wheel)
        if (self.wheel(i) > LIMIT_VELOCITY_SUPERIOR)
          self.wheel(i) = LIMIT_VELOCITY_SUPERIOR;
        end
      end

    end

    function inverseKinematicPlatform(self)

      % distanceBetweenWheelCenterRobot is l1.
      % distanceBetweenWheelSomeSide is l2.

      aux = [1  1 -(self.distanceBetweenWheelCenterRobot + self.distanceBetweenWheelSomeSide); ...
             1 -1  (self.distanceBetweenWheelCenterRobot + self.distanceBetweenWheelSomeSide); ...
             1 -1 -(self.distanceBetweenWheelCenterRobot + self.distanceBetweenWheelSomeSide); ...
             1  1  (self.distanceBetweenWheelCenterRobot + self.distanceBetweenWheelSomeSide)];

      self.velocityWheels = (1/self.radiusWheel)*aux*self.velocityPlatform';

    end

    function moveRobotToPosition(self, positionTarget)

      distance = inf;

      while (true)

        if (distance < 0.2) break; end

        self.vrep.vrep.simxSynchronousTrigger(self.vrep.clientID);
        self.positionTargetPlatform = self.vrep.getPositionObject(self.vrep.idDisc);
        self.positionCurrentPlatform = self.vrep.getPositionObject(self.vrep.idPlatform);
        distance = pdist([self.positionTargetPlatform; self.positionCurrentPlatform], 'euclidean');
        self.calculateVelocityPlatform();
        self.inverseKinematicPlatform();

        for i = 1 : length(self.wheel)
          self.vrep.setVelocityJoint(self.vrep.idWheel(i), self.velocityWheels(i));
        end

      end

    end

  end

end
