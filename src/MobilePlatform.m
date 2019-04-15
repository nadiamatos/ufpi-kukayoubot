classdef MobilePlatform < handle

  properties
    % Basead in: http://www.youbot-store.com/wiki/index.php/YouBot_Detailed_Specifications
    % mensuraments in meters.
    DISTANCE_BETWEEN_WHEEL_CENTER_ROBOT = 0.15023; % distanceBetweenWheelCenterRobot is l1.
    DISTANCE_BETWEEN_WHEEL_SOME_SIDE = 0.2355;  % distanceBetweenWheelSomeSide is l2.
    RADIUS_WHEEL = 0.05;
    RADIUS_PLATFORM = 0.2793;

    velocityModulePlatform = 1;
    velocityPlatform = zeros(3, 1); % [vx; vy; omega]
    velocityWheels = zeros(4, 1);   % [omega1; omega2; omega3; omega4]
    % wheel = zeros(1, 4);
    positionCurrentPlatform = zeros(1, 3);
    positionTargetPlatform = zeros(1, 3);
    angleBetweenPlatformPointTarget = 0.0;
    anglePlatform = 0.0;
    angleDesired = 0.0;
    vrep = [];
  end

  methods

    function self = MobilePlatform(vrep)
      self.vrep = vrep;
      self.anglePlatform = self.vrep.getOrientationObject(self.vrep.idPlatform);
    end

    function forwardKinematicPlatform(self)
      a = self.DISTANCE_BETWEEN_WHEEL_CENTER_ROBOT + self.DISTANCE_BETWEEN_WHEEL_SOME_SIDE;
      matrixAux = 0.25*[1    1    1    1;
                        1   -1    1   -1;
                        1/a -1/a -1/a  1/a];
      self.velocityPlatform = matrixAux*self.velocityWheels;
      self.controlLimitsVelocityPlatform();
    end

    function inverseKinematicPlatform(self, velocityInX, velocityInY, velocityRot)
      b = [velocityInX; velocityInY; velocityRot];
      a = self.DISTANCE_BETWEEN_WHEEL_CENTER_ROBOT + self.DISTANCE_BETWEEN_WHEEL_SOME_SIDE;
      aux = [1 -1  a; ...
             1  1 -a; ...
             1  1  a; ...
             1 -1 -a];
      self.velocityWheels = (1/self.RADIUS_WHEEL)*aux*b;
      self.controlLimitsVelocityWheel();
    end

    function calculateAngleDesired(self)
      self.angleDesired = self.anglePlatform - self.angleBetweenPlatformPointTarget;
      % if (self.angleDesired < 0)
        % self.angleDesired = 360 + self.angleDesired;
      % end
    end

    function calculateVelocityAngularPlatform(self)
      self.velocityPlatform(3) = deg2rad(self.angleDesired)*0.134646;
      % time = velocityAngular/self.velocityModulePlatform;
    end

    function updateDataOfTheRobot(self)
      self.positionCurrentPlatform = self.vrep.getPositionObject(self.vrep.idOrigemPlatform);
      self.positionTargetPlatform = self.vrep.getPositionObject(self.vrep.idDisc);
      self.anglePlatform = self.vrep.getOrientationObject(self.vrep.idPlatform);
      self.angleBetweenPlatformPointTarget = self.calculateAngleBetweenPoints(self.positionCurrentPlatform, ...
                                                                              self.positionTargetPlatform);
      self.calculateAngleDesired();
      self.calculateVelocityAngularPlatform();
      self.calculateVelocityLinearPlatform();
    end

    function controlLimitsVelocityPlatform(self)
      LIMIT_VELOCITY_SUPERIOR = 0.005;  % m/s
      for i = 1 : length(self.velocityPlatform-1)
        if (self.velocityPlatform(i) > LIMIT_VELOCITY_SUPERIOR)
          self.velocityPlatform(i) = LIMIT_VELOCITY_SUPERIOR;
        elseif (self.velocityPlatform(i) < -LIMIT_VELOCITY_SUPERIOR)
          self.velocityPlatform(i) = -LIMIT_VELOCITY_SUPERIOR;
        end
      end
    end

    function controlLimitsVelocityWheel(self)
      LIMIT_VELOCITY_SUPERIOR = 1.57; % rad/sec
      for i = 1 : length(self.velocityWheels)
        if (self.velocityWheels(i) > LIMIT_VELOCITY_SUPERIOR)
          self.velocityWheels(i) = LIMIT_VELOCITY_SUPERIOR;
        elseif (self.velocityWheels(i) < -LIMIT_VELOCITY_SUPERIOR)
          self.velocityWheels(i) = -LIMIT_VELOCITY_SUPERIOR;
        end
      end
    end

    function calculateVelocityLinearPlatform(self)
      dist = self.calculateDistanceBetweenPoints(self.positionCurrentPlatform(1, 1:2), ...
                                                 self.positionTargetPlatform(1, 1:2));
      self.velocityModulePlatform = dist;
      % self.velocityModulePlatform = dist/1000;
      self.velocityPlatform(1:2, 1) = self.velocityModulePlatform*[-cosd(self.angleDesired) ...
                                                                    sind(self.angleDesired)];
      self.controlLimitsVelocityPlatform();
    end

    function sendVelocityJointsWheel(self)
      for i = 1 : length(self.velocityWheels)
        % self.vrep.setVelocityJoint(self.vrep.idWheel(i), (self.velocityWheels(i)));
        self.vrep.setVelocityJoint(self.vrep.idWheel(i), rad2deg(self.velocityWheels(i)));
      end
    end

    function distanceBetweenPoints = calculateDistanceBetweenPoints(self, point1, point2)
      % in meters
      distanceBetweenPoints = (pdist([point1; point2], 'euclidean'));
    end

    function angleBetweenPoints = calculateAngleBetweenPoints(self, point1, point2)
      % point1 - current point and point2 - target point
      angleBetweenPoints = floor(atan2d(point2(2) - point1(2), point2(1) - point1(1)));
      if (angleBetweenPoints < 0)
        angleBetweenPoints = 360 + angleBetweenPoints;
      end
      %angleBetweenPoints = atan2d( point2(1)-point1(1), point2(2)-point1(2) );
    end

    function controlPlatform(self)
      self.inverseKinematicPlatform(0, 0, self.velocityPlatform(3));
      self.sendVelocityJointsWheel();
      self.inverseKinematicPlatform(self.velocityPlatform(1), ...
                                    self.velocityPlatform(2), ...
                                    0);
      self.sendVelocityJointsWheel();
    end

    function moveRobotToPosition(self)

      dist = inf;
      self.updateDataOfTheRobot();

      while dist > 0
        dist = self.calculateDistanceBetweenPoints(self.positionCurrentPlatform(1, 1:2), ...
                                                   self.positionTargetPlatform(1, 1:2));
        self.updateDataOfTheRobot();
        % self.vrep.pauseCommunication(true);
        self.controlPlatform();
        % self.vrep.pauseCommunication(false);
        if (dist < 0.04)
          break; disp('aqui');
        end
      end
    end

  end

end
