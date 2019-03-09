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
    joint = zeros(1, 6);
    wheel = zeros(1, 4);
    positionCurrentPlatform = zeros(1, 3);
    positionTargetPlatform = zeros(1, 3);
    anglePlatform = 0;
    vrep = [];

  end

  methods

    function self = Robot()

      self.vrep = Vrep();
      self.anglePlatform = self.vrep.getOrientationObject(self.vrep.idPlatform);

    end

    function forwardKinematicArm(self)

      t1 = self.juntas(1); t2 = self.juntas(2); t3 = self.juntas(3);
      t3 = self.juntas(3); t4 = self.juntas(4); t5 = self.juntas(5);

      T01 = [cosd(t1)  -sind(t1) 0  0; ...
             sind(t1)   cosd(t1) 0  0; ...
               0          0      1  l0; ...
               0          0      0  1];

      T12 = [-sind(t2) -cosd(t2) 0  a; ...
               0          0     -1  0; ...
              cosd(t2) -sind(t2) 0  0; ...
               0          0      0  1];

      T23 = [cosd(t3)  -sind(t3) 0  l1; ...
             sind(t3)   cosd(t3) 0  0; ...
               0          0      1  0; ...
               0          0      0  1];

      T34 = [-sind(t4)  cosd(t4) 0  l2; ...
              cosd(t4) -sind(t4) 0  0; ...
               0          0      1  0; ...
               0          0      0  1];

      T45 = [cosd(t5) -sind(t5)  0  0; ...
               0          0     -1  0; ...
             sind(t5)  cosd(t5)  0  0; ...
               0          0      0  1];

      T56 = [1  0  0  0; ...
             0  1  0  0; ...
             0  0  1  l3; ...
             0  0  0  1];

      T = T01*T12*T23*T34*T45*T56;

      fprintf('x = %f\n', T(1, 4));
      fprintf('y = %f\n', T(2, 4));
      fprintf('z = %f\n', T(3, 4));

    end

    function inverseKinematicArm(self)

    end

    function forwardKinematicPlatform(self, velocityWheels)

    end

    function calculateVelocityPlatform(self)

      dt = 10;
      self.velocityPlatform = (self.positionTargetPlatform - self.positionCurrentPlatform)/dt;

      angleTarget = self.calculateAngleBetweenPoints(self.positionCurrentPlatform, self.positionTargetPlatform)
      %angleTarget = self.calculateAngleBetweenPoints([0 0 0], self.positionTargetPlatform)
      self.anglePlatform = self.vrep.getOrientationObject(self.vrep.idPlatform)
      self.velocityPlatform(3) = deg2rad(angleTarget - self.anglePlatform)/dt;

      %self.controlLimitsVelocityPlatform();

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
      self.velocityWheels = self.velocityWheels';

    end

    function sendVelocityJointsWheel(self)

      for i = 1 : length(self.wheel)
        self.vrep.setVelocityJoint(self.vrep.idWheel(i), self.velocityWheels(i));
      end

    end

    function distanceBetweenPoints = calculateDistanceBetweenPoints(self, point1, point2)

      % in meters
      distanceBetweenPoints = pdist([point1; point2], 'euclidean');

    end

    function angleBetweenPoints = calculateAngleBetweenPoints(self, point1, point2)

      % point1 - current point and point2 - target point
      angleBetweenPoints = atan2d( point2(2)-point1(2), point2(1)-point1(1) );
      %angleBetweenPoints = atan2d( point2(1)-point1(1), point2(2)-point1(2) );

    end

    function moveRobotToPosition(self, positionTarget)

      distance = inf;

      while (true)

        if (distance < 0.2) break; end

        self.vrep.vrep.simxSynchronousTrigger(self.vrep.clientID);
        self.positionTargetPlatform = self.vrep.getPositionObject(self.vrep.idDisc);
        self.positionCurrentPlatform = self.vrep.getPositionObject(self.vrep.idPlatform);
        distance = self.calculateDistanceBetweenPoints(self.positionTargetPlatform, self.positionCurrentPlatform);
        self.calculateVelocityPlatform();
        self.inverseKinematicPlatform();
        self.sendVelocityJointsWheel();

      end

    end

  end

end
