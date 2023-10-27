classdef RobotBaseball < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        Environment;
        KR;
        KRJointAngles;
        UR;
        URJointAngles;
        
        EStopFlag = false;
        eStopApp;
        steps = 30;
        RmrcTraj;
        baseTransformUR3;
        HardStop;
        buttonPin = 'D2';


    end

    methods
        %% cthrow the ball
        function self = RobotBaseball()
            close all force;
            clc;
            clf;
            
            hold on;
            % self.clearArduino();
            % self.HardStop = arduino('COM5', 'Uno');
            % configurePin(self.HardStop, self.buttonPin, 'DigitalInput');
            self.eStopApp = eStop();
            self.RmrcTraj = nan(self.steps,6); 
            self.Environment = RobotBaseballEnvironment();
            self.BuildRobots();
            % self.TestLightCurtain();
            input('press enter to play baseball');

            self.PitchBall(1);
            self.PitchBall(2);
            self.PitchBall(3);
            self.PitchBall(4);
        end

       
        %%
        function BuildRobots(self)
            hold on;
            
            self.baseTransformUR3 = transl(10.5,0,0.1) *trotz(pi/2); % Rotates UR3 by pi/2 back
            self.UR = UR3Batter(self.baseTransformUR3); %store UR3 robot in the UR3 
            self.URJointAngles = self.UR.model.getpos();
            self.UR.model.animate(self.KRJointAngles);

            baseTransformKR6R700C = transl(-1,0,0.2);
            self.KR = KR6R700CR(baseTransformKR6R700C); %store Kuka Robot in the 'KR' robot property.
            self.KRJointAngles = self.KR.model.getpos(); %store joint angles for use
            self.KR.model.animate(self.KRJointAngles); %Draw the robot
            

            % self.KR.model.links
           
            % self.KR.model.teach();
            % self.UR.model.teach();
        end

      
        %%
        % Batter in position
        function RMRC(self, robot, goal, jointGuess)  
            q = robot.model.getpos;
            T1 = robot.model.fkine(q).T;       % First pose

            M = [1 1 1 zeros(1,3)];                         % Masking Matrix

            x1 = [robot.model.fkine(q).t(1);robot.model.fkine(q).t(2);robot.model.fkine(q).t(3)];
            x2 = goal;
            deltaT = 0.05;     

            x = zeros(3,self.steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
            s = lspb(0,1,self.steps);                                 % Create interpolation scalar
            for i = 1:self.steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2; % x position at each step                 % Create trajectory in x-y plane
            end

            self.RmrcTraj(1,:) = robot.model.ikine(T1, 'q0', jointGuess, 'mask', M);   % sets the inital joint angle              % Solve for joint angles

            for i = 1:self.steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;   % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
                xdot = [xdot' 0 0 0];
                J = robot.model.jacob0(self.RmrcTraj(i,:));            % Get the Jacobian at the current state
                J = J(1:6,:);                           
                qdot = pinv(J)*xdot'; % change in joint angles   (velocity of joint angles)                         % Solve velocitities via RMRC
                self.RmrcTraj(i+1,:) =  self.RmrcTraj(i,:) + deltaT*qdot';                   % Update next joint state
            end

        end   

        %% 
         function WaveBat(self, robot)        
            t = 10;             % Total time (s)
            deltaT = t/self.steps;      % Control frequency
            epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector
            
            % 1.2) Allocate array data
            m = zeros(self.steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(self.steps,6);       % Array for joint anglesR
            qdot = zeros(self.steps,6);          % Array for joint velocities
            theta = zeros(3,self.steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,self.steps);             % Array for x-y-z trajectory
            positionError = zeros(3,self.steps); % For plotting trajectory error
            angleError = zeros(3,self.steps);    % For plotting trajectory error
            radius = 0.5;
            
            % 1.3) Set up trajectory, initial pose
            % s = lspb(0,1,self.steps);                % Trapezoidal trajectory scalar
            thetaSemiCircle = linspace(0, pi, self.steps); % Model a semi circle for the UR3 to wave its bat
            for i = 1:self.steps
                x(1, i) = cos(thetaSemiCircle(i)) * radius + self.baseTransformUR3(1,4); % x-coordinate
                x(2, i) = sin(thetaSemiCircle(i)) * radius + self.baseTransformUR3(2,4); % y-coordinate
                x(3, i) = 0.8 + self.baseTransformUR3(3,4); % Fixed z-coordinate
                theta(1, i) = pi/2; % Roll angle
                theta(2, i) = 0; % Pitch angle
                theta(3, i) = 0; % Yaw angle
            end
             
            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
            q0 = zeros(1,6);                                                            % Initial guess for joint angles
            qMatrix(1,:) = robot.model.ikcon(T,q0);                                              % Solve joint angles to achieve first waypoint
            
            % 1.4) Track the trajectory with RMRC
            for i = 1:self.steps-1
                T = robot.model.fkine(qMatrix(i,:)).T;                                           % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                Rerror = Rd * Ra';
                deltaTheta = tr2rpy(Rerror);                                            % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = robot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = pinv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < robot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
                positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
                angleError(:,i) = deltaTheta;                                           % For plotting
            end
            
            % 1.5) Plot the results for debugging
            % figure(1)
            % plotRMRC = plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',0.2); % Track the line of the RMRC movement

            % UR = UR3Batter(baseTransformUR3);
            MoveRobot(self,robot,qMatrix);
            % delete(plotRMRC)

        end  
        %%
        function ReturnToStart (self, robot)
            % Function for UR3 to return to start position
            urq1 = robot.model.getpos();
            s = lspb(0,1,self.steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(self.steps,6);
            urq2 = robot.model.ikcon(self.baseTransformUR3, zeros(1,6));
            for i = 1:self.steps
                    qMatrix(i,:) = (1-s(i))*urq1 + s(i)*urq2;
            end
            MoveRobot(self,robot,qMatrix);      
        end



        %% Pitch Ball 1
        % PitchBall animates the pitch and hit
        function PitchBall(self, situation)
            % Switch situation determines how the UR3 hits the ball
            switch situation 
                case 1 % Bat turns 90 deg. Hit successful.
                    UR3Base = self.UR.model.base.T;
                    ballThrow = transl(0.2,0,-0); % Ball throw translation
                    ballHit = transl(-0.2,-0.2,0.17); % Ball hit translation
                    hitPos1 = transl(UR3Base(1,4),0,0.7466)*trotx(pi/2)*trotz(-pi/2); % End eff position of UR3 for the hit
                    urq2 = self.UR.model.ikcon(hitPos1);
                case 2 % Bat turns 180 deg. Hit successful.
                    UR3Base = self.UR.model.base.T;
                    UR3Base(1,4) = UR3Base(1,4) - 0.2; % Needed to look like ball hits bat at right moment
                    ballThrow = transl(0.2,0,-0);
                    ballHit = transl(-0.2,0.2,0.17);
                    urq2 = deg2rad([-180 0 0 0 0 0]);
                case 3 % Home run - high hit
                    UR3Base = self.UR.model.base.T;
                    UR3Base(1,4) = UR3Base(1,4) - 0.2; % Needed to look like ball hits bat at right moment
                    ballThrow = transl(0.2,0,-0);
                    ballHit = transl(-0.2,0.2,0.25);
                    urq2 = deg2rad([-180 0 0 0 0 0]);
                case 4 % foul ball
                    UR3Base = self.UR.model.base.T;
                    UR3Base(1,4) = UR3Base(1,4); 
                    ballThrow = transl(0.2,0,-0);
                    ballHit = transl(0.2,-0.2,0.17);
                    urq2 = deg2rad([-90 0 0 0 0 0]);
                otherwise
                    error('situation = 1,2, 3 or 4')
            end

            self.ReturnToStart(self.UR) % UR3 returns to start
            self.WaveBat(self.UR) % UR3 waves the bat
            self.ReturnToStart(self.UR) % UR3 returns to start

            UR3reaction = 2;
            qpasser1 = [0 2.5598 0.4145 0 0 0]; % Initial guess for first KR6 ikcon
            hit1Flag = 0; % Hitflag to change between throw and hit
            balls = RobotBalls; 
            krq1 = self.KR.model.getpos();
            krq2 = self.KR.model.ikcon(balls.ballModel{1}.base, qpasser1);
            s = lspb(0,1,self.steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(self.steps,6);
                 for i = 1:self.steps
                    qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
                 end
            self.MoveRobot(self.KR, qMatrix);
            % 1.1 Prepare to throw the ball
            krq1 = krq2;
            krq2 = [0    0.0818   -2.0944    0.0000   -0.0122    0.0000]; % preparing to throw pos of KR
            s = lspb(0,1,self.steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(self.steps,6);
                 for i = 1:self.steps
                    qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
                 end

            for i = 1:self.steps
                % self.checkButtonState();
                CheckCollision(self, self.KR);
                if strcmp(self.eStopApp.systemState, 'running')
                    self.KR.model.animate(qMatrix(i,:));
                    balls.ballModel{1}.base = self.KR.model.fkine(qMatrix(i,:));
                    balls.ballModel{1}.animate(0);
                    drawnow();
                else
                    input("please disengage emergency stop before continuing")
                end
            end
            
            % 1.2 Action to throw the ball
            krq1 = krq2;
            krq2 = [ 0    2.8162   -0.7898         0   -0.0122   -1.5272]; % throw pos of KR
            s = lspb(0,1,self.steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(self.steps,6);
                 for i = 1:self.steps
                    qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
                 end
    
            for i = 1:self.steps
                    % self.checkButtonState();
                    CheckCollision(self, self.KR);
                    if strcmp(self.eStopApp.systemState, 'running')
                        self.KR.model.animate(qMatrix(i,:));
                        if self.steps/1.36 >= i % ball is thrown when steps = 30/1.36. Height is 0.7466 to align with end eff of UR3. 
                          balls.ballModel{1}.base = self.KR.model.fkine(qMatrix(i,:));
                          balls.ballModel{1}.animate(0);
                          drawnow();
                        end
                        if i > self.steps/1.36
                                  % Reset orientation of ball so we can control it
                                  ballPos = balls.ballModel{1}.base.T;
                                  newBallPos = eye(4); % set orientation matrix to eye(3)
                                  newBallPos(1:3,4) = ballPos(1:3,4); % set position matrix to current pos
                                  balls.ballModel{1}.base = newBallPos;
                                  % Animate it
                                  balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballThrow;
                                  balls.ballModel{1}.animate(0);
                        end
                    drawnow();
                    else
                        input("please disengage emergency stop before continuing")
                    end           
            end

            urq1 = self.UR.model.getpos(); % in case robot stops halfway in steps - will restart from pos
            
            s = lspb(0,1,self.steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(self.steps,6);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*urq1 + s(i)*urq2;
            end

            for i = 1:self.steps % 1st part of throw/hit (as 30 steps doesnt get to the end)
                ballXYZ = balls.ballModel{1}.base.T;
                % self.checkButtonState();
                CheckCollision(self, self.UR);
                if strcmp(self.eStopApp.systemState, 'running')
                    if ballXYZ(1,4) >= UR3Base(1,4) - UR3reaction % if ball comes within 2m (UR3reaction) of UR3 it animates
                        self.UR.model.animate(qMatrix(i,:));
                    end
                    if ballXYZ(1,4) <= UR3Base(1,4) && hit1Flag == 0 % if the ball does not pass the UR3 base it follows the throw traj ballThrow
                        balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballThrow;
                        balls.ballModel{1}.animate(0);
                    else                                             % if the ball passes the UR3 base it follows the hit traj ballHit
                        hit1Flag = 1;
                        balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballHit;
                        balls.ballModel{1}.animate(0);
                    end
                        drawnow();
                else
                    input("please disengage emergency stop before continuing")
                end              
            end
            
            urq1 = self.UR.model.getpos(); % in case robot stops halfway in steps - will restart from pos
            
            s = lspb(0,1,self.steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(self.steps,6);
            for i = 1:self.steps
                qMatrix(i,:) = (1-s(i))*urq1 + s(i)*urq2;
            end

            for i = 1:self.steps % 2nd part of throw/hit (as 30 steps doesnt get to the end)
                ballXYZ = balls.ballModel{1}.base.T;
                % self.checkButtonState();
                CheckCollision(self, self.UR);
                if strcmp(self.eStopApp.systemState, 'running')
                    if ballXYZ(1,4) >= UR3Base(1,4) - UR3reaction % if ball comes within 2m (UR3reaction) of UR3 it animates
                        self.UR.model.animate(qMatrix(i,:));
                    end
                    if ballXYZ(1,4) <= UR3Base(1,4) && hit1Flag == 0 % if the ball does not pass the UR3 base it follows the throw traj ballThrow
                        balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballThrow;
                        balls.ballModel{1}.animate(0);
                    else                                             % if the ball passes the UR3 base it follows the hit traj ballHit
                        hit1Flag = 1;
                        balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballHit;
                        balls.ballModel{1}.animate(0);
                    end
                        drawnow();
                    
                else
                    input("please disengage emergency stop before continuing")
                end
            end
            for i = 1:self.steps % 3rd part of throw (as 30 steps doesnt get ball to the end)
                balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballHit;
                balls.ballModel{1}.animate(0);
                drawnow();
            end
        end
      

        %% animate the robot
        function MoveRobot(self, robot, qMatrix)
            for i = 1:self.steps
                % self.checkButtonState();
                CheckCollision(self, robot);
                if strcmp(self.eStopApp.systemState, 'running')
                        robot.model.animate(qMatrix(i,:));
                        drawnow();
                        pause(0.05); 
                else
                    input("please disengage emergency stop before continuing")
                end
            end
        end

   %% GetLinkPoses
        function [ transforms ] = GetLinkPoses(~, robot)
            q = robot.model.getpos();
            links = robot.model.links;
            transforms = zeros(4, 4, length(links) + 1);
            transforms(:,:,1) = robot.model.base;
            
            %from LAB 5, get transform of each link
            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = transforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                transforms(:,:,i + 1) = current_transform;

                % position = current_transform(1:3, 4);  % get translation part of the transform
                % fprintf('Link %d position: x = %.2f, y = %.2f, z = %.2f\n', i, position(1), position(2), position(3));
            end
        end

        %% Check collisions
        function collisionDetected = CheckCollision(self, robot)
            % get the transforms of all robot links
            transforms = self.GetLinkPoses(robot);
            
            collisionDetected = false;
            
            
            for i = 1:size(transforms, 3)
                linkPos = transforms(1:3, 4, i); %loop through each robot link getting position
                
                % check through each environment object
                for j = 1:length(self.Environment.EnvironmentObjects)
                    objPos = self.Environment.EnvironmentObjects(j).Position';
                    objRadius = self.Environment.EnvironmentObjects(j).Radius;
                    
                    % Calculate the distance between the link and the object
                    distance = norm(linkPos - objPos);
                    
                    %check if the distance is less than the object's radius
                    if distance < objRadius
                        % distance;
                        collisionDetected = true;
                        fprintf('Collision detected between Link %d and Object %d\n', i, j);
                        
                        % small dot where the collision occurred
                        plot3(linkPos(1), linkPos(2), linkPos(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
                        hold on; 
                        self.eStopApp.systemState = 'eStopped';
                    end
                end
            end
        end

        %%
        function checkButtonState(self)
            % Check for button press from Arduino
            % buttonState = readDigitalPin(self.HardStop, self.buttonPin);
            % If button is pressed, change systemState to 'eStopped'
            if buttonState == 0
                self.eStopApp.systemState = 'eStopped';
            end
        end
        %%
        function clearArduino(self)
            if ~isempty(self.HardStop)
                clear self.HardStop;
                self.HardStop = [];
            end
        end

%%
           function TestLightCurtain(self)
            personSteps = 200; 
        
            B = [12, 3.8, 0];
            A = [12, -8, 0];
            stepSize = (B - A) / personSteps; %step length for each coordinate
        
            person_light_Pos = A;
            person_light = PlaceObject('personMaleCasual.ply', person_light_Pos);
            original_verts = [get(person_light, 'Vertices'), ones(size(get(person_light, 'Vertices'), 1), 1)]; % Store the original vertices
            verts = original_verts;
            verts(:, 1:3) = verts(:, 1:3) * 1; %scale 
            set(person_light, 'Vertices', verts(:, 1:3));
            self.Environment.AddEnvironmentObject('Person', person_light_Pos, 1);
        
            input('Start Light Curtain Test, press enter:')
            for i = 1:personSteps
                  if strcmp(self.eStopApp.systemState, 'running')
                        self.checkLightCurtainCollision();
    
                        % Update position of the person
                        person_light_Pos = person_light_Pos + stepSize;
                    
                        % Calculate the new vertices based on the original position
                        new_verts = original_verts;
                        new_verts(:, 1:3) = new_verts(:, 1:3) + repmat((person_light_Pos - A), size(new_verts, 1), 1);
                    
                        % Update verticies of person
                        set(person_light, 'Vertices', new_verts(:, 1:3));
                    
                        % Update the position of the person in EnvironmentObjects
                        % array
                        self.Environment.EnvironmentObjects(end).Position = person_light_Pos;
                        pause(0.1);
                else
                    input("please disengage emergency stop before continuing")
                end
                
                
            end
        end

        %%
        function checkLightCurtainCollision(self)
            pt1 = [10.7, -2.2, 0];
            pt2 = [13.9, -2.2, 0];

            % Ensure pt1 and pt2 are row vectors
            if size(pt1, 1) > 1
                pt1 = pt1';
            end
            if size(pt2, 1) > 1
                pt2 = pt2';
            end
            
            % Calculate the direction vector from pt1 to pt2
            direction = pt2 - pt1;
            
            % calculate the normal vector to the plane
            normal = cross(direction, [0, 0, 1]);
            
            for j = 1:length(self.Environment.EnvironmentObjects)
              
                objPos = self.Environment.EnvironmentObjects(j).Position';
                objRadius = self.Environment.EnvironmentObjects(j).Radius;
                
                % Ensure objPos is a row vector
                if size(objPos, 1) > 1
                    objPos = objPos';
                end
        
                % Calculate the distance from the object center to the plane
                distanceToPlane = dot(normal, objPos - pt1) / norm(normal);
                
                 % Check if the object's radius intersects the plane
                if abs(distanceToPlane) <= objRadius-0.4
                    fprintf('Collision detected with Light Curtain for Object %d\n', j);
                    
                    self.eStopApp.systemState = 'eStopped';
                    
                    % plot collision point
                    plot3(objPos(1), objPos(2), objPos(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
                    hold on;
                end
            end
        end      
    end
end
