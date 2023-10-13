classdef RobotBaseball < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
       
        KR
        KRJointAngles
        UR
        URJointAngles
        
        EnvironmentObjects

        EStopFlag = false

    end

    methods
        %% cthrow the ball
        function self = RobotBaseball()
            %close all

            clc;
            clf;
            
            hold on;
            self.BuildField();
            self.BuildRobots();
            self.BuildPeople();
            input('press enter to play baseball')
            
            self.PitchBall()

        end

        %%
        function BuildField(self)
            surf([-1,-1;1,1]*15 ...
                ,[-1,1;-1,1]*15 ...
                ,[0,0;0,0] ...
                ,'CData',imread('baseball_field_1.jpg') ...
                ,'FaceColor','texturemap');
        end
        %%
        function BuildRobots(self)
            hold on;
            baseTransformKR6R700C = transl(-1,0,0.2);
            self.KR = KR6R700CR(baseTransformKR6R700C); %store Kuka Robot in the 'KR' robot property.
            self.KRJointAngles = self.KR.model.getpos(); %store joint angles for use
            self.KR.model.animate(self.KRJointAngles); %Draw the robot
            
            baseTransformUR3 = transl(10.5,0,0);
            self.UR = UR3(baseTransformUR3); %store UR3 robot in the UR3 
            self.URJointAngles = self.UR.model.getpos();
            self.UR.model.animate(self.KRJointAngles);

            % self.KR.model.links
           
            % self.KR.model.teach();
            % self.UR.model.teach();
        end

        %%
        function BuildPeople(self)

            hold on;

            % person_1_Pos = [-2, 0, 0]; 
            % person_1 = PlaceObject('personMaleCasual.ply', person_1_Pos);
            % verts = [get(person_1, 'Vertices'), ones(size(get(person_1, 'Vertices'), 1), 1)];
            % verts(:, 1:3) = verts(:, 1:3) * 1; %scale 
            % set(person_1, 'Vertices', verts(:, 1:3));
            % 
            % self.AddEnvironmentObject('Person', person_1_Pos, 1);
            % 
            % person_2_Pos = [0, 0, 0];
            % person_2 = PlaceObject('personMaleCasual.ply', person_2_Pos); 
            % verts = [get(person_2, 'Vertices'), ones(size(get(person_2, 'Vertices'), 1), 1)];
            % verts(:, 1:3) = verts(:, 1:3) * 1; %scale 
            % set(person_2, 'Vertices', verts(:, 1:3));
            % 
            % self.AddEnvironmentObject('Person', person_2_Pos, 1);
            % 
        end

        %%
        function AddEnvironmentObject(self, type, position, radius)
            objInfo = struct();
            objInfo.Type = type;
            objInfo.Position = position;
            objInfo.Radius = radius;
            
            if isempty(self.EnvironmentObjects)
                self.EnvironmentObjects = objInfo;
            else
                self.EnvironmentObjects(end+1) = objInfo;
            end
            fprintf('Current objects in the environment:\n');
            for i = 1:length(self.EnvironmentObjects)
                fprintf('Object %d:\n', i);
                fprintf('\tType: %s\n', self.EnvironmentObjects(i).Type);
                fprintf('\tPosition: [%f, %f, %f]\n', self.EnvironmentObjects(i).Position(1), ...
                                                      self.EnvironmentObjects(i).Position(2), ...
                                                      self.EnvironmentObjects(i).Position(3));
                fprintf('\tRadius: %f\n', self.EnvironmentObjects(i).Radius);
            end
        end


        %%
        % Batter in position
        function RMRC(~, robot, goal, jointGuess, steps)      
            q = robot.model.getpos;
            T1 = robot.model.fkine(q).T;       % First pose

            M = [1 1 1 zeros(1,3)];                         % Masking Matrix
            
            x1 = [robot.model.fkine(q).t(1);robot.model.fkine(q).t(2);robot.model.fkine(q).t(3)];
            x2 = goal;
            deltaT = 0.05;     
            
            x = zeros(3,steps); % zeros for columns 1 to steps. Stores x and y and z positions for each step of the trajectory
            s = lspb(0,1,steps);                                 % Create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2; % x position at each step                 % Create trajectory in x-y plane
            end
 
            qMatrix = nan(steps,6); % stores joint angles at each step  
            qMatrix(1,:) = robot.model.ikine(T1, 'q0', jointGuess, 'mask', M);   % sets the inital joint angle              % Solve for joint angles
      
            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;   % calculates velocity at each position by getting change between next and current position and dividing by time step                          % Calculate velocity at discrete time step
                xdot = [xdot' 0 0 0];
                J = robot.model.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
                J = J(1:6,:);                           
                qdot = inv(J)*xdot'; % change in joint angles   (velocity of joint angles)                         % Solve velocitities via RMRC
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
            end
            
            for i = 1:steps
                robot.model.animate(qMatrix(i,:)); % Animating the robot to move to the set joint configuratio
                drawnow()
                pause(0.05);
            end
        end   



        %% 
        %1.0 Pick up ball
        function PitchBall(self)
            qpasser1 = [ 0    2.5598    0.4145         0         0         0];
            goal = [10; 0; 0.5];
            self.RMRC(self.UR, goal, qpasser1, 50);
            ballTransl = transl(-0.2,0,-0);
            ballHit = transl(0.2,-0.1,0.15);
            hit1Flag = 0;
            steps = 30;
            balls = RobotBalls;
            krq1 = zeros(1,6);
            krq2 = self.KR.model.ikcon(balls.ballModel{1}.base, qpasser1);
            s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(steps,6);
                 for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
                 end
            MoveRobot(self, self.KR, steps, qMatrix);
               % 1.1 Prepare to throw the ball
            krq1 = krq2;
            krq2 = [pi    0.0818   -2.0944    0.0000   -0.0122    0.0000];
            s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(steps,6);
                 for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
                 end

            for i = 1:steps
                    self.KR.model.animate(qMatrix(i,:));
                    CheckCollision(self, self.KR);
                    balls.ballModel{1}.base = self.KR.model.fkine(qMatrix(i,:));
                    balls.ballModel{1}.animate(0);
                    drawnow();
            end
            % 1.2 Action to throw the ball
            krq1 = krq2;
            krq2 = [ pi    2.8162   -0.7898         0   -0.0122   -1.5272];
            s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(steps,6);
                 for i = 1:steps
                    qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
                 end
    
            for i = 1:steps
                    self.KR.model.animate(qMatrix(i,:));
                    CheckCollision(self, self.KR);
            
                    if steps/1.36 >= i
                      balls.ballModel{1}.base = self.KR.model.fkine(qMatrix(i,:));
                      balls.ballModel{1}.animate(0);
                      drawnow();
                      if i == steps/1.36
                          ballStart = self.KR.model.fkine(qMatrix(i,:))
                      end
                    end
                    if i > steps/1.36
                       % for j = 0:0.01:0.1
                              % reset orientation of ball so we can control it. Can do
                              % in a function later
                              ballPos = balls.ballModel{1}.base.T;
                              newBallPos = eye(4);
                              newBallPos(1:3,4) = ballPos(1:3,4);
                              balls.ballModel{1}.base = newBallPos;
            
                              % Animate it
                              balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballTransl;
                              balls.ballModel{1}.animate(0);
                              drawnow();
                       % end
                    end
            
                    drawnow();
            
            end
            
            urq1 = self.UR.model.getpos();
            urq2 = self.UR.model.ikcon(transl(-5,0,0.7466)*trotx(pi/2)*trotz(pi/2));
            
            s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
            qMatrix = nan(steps,6);
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*urq1 + s(i)*urq2;
            end

            for i = 1:steps
                ballXYZ = balls.ballModel{1}.base.T;
                self.UR.model.animate(qMatrix(i,:));
                CheckCollision(self, self.UR);
                drawnow();
                if ballXYZ(1,4) >= -5 && hit1Flag == 0
                    balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballTransl;
                    balls.ballModel{1}.animate(0);
                    drawnow();
                else
                    hit1Flag = 1;
                    balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballHit;
                    balls.ballModel{1}.animate(0);
                    drawnow();
                end
            end
        end

        %% animate the robot
        function MoveRobot(self, robot, steps, qMatrix)
           for i = 1:steps
                    CheckCollision(self, robot);
                    robot.model.animate(qMatrix(i,:));
                    drawnow();
            end 
        end

   %% GetLinkPoses
        function [ transforms ] = GetLinkPoses(self, robot)
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
                for j = 1:length(self.EnvironmentObjects)
                    objPos = self.EnvironmentObjects(j).Position';
                    objRadius = self.EnvironmentObjects(j).Radius;
                    
                    % Calculate the distance between the link and the object
                    distance = norm(linkPos - objPos);
                    
                    %check if the distance is less than the object's radius
                    if distance < objRadius
                        distance;
                        collisionDetected = true;
                        fprintf('Collision detected between Link %d and Object %d\n', i, j);
                        
                        % small dot where the collision occurred
                        plot3(linkPos(1), linkPos(2), linkPos(3), 'ro', 'MarkerSize', 5, 'MarkerFaceColor', 'r');
                        hold on; 

                        EmergencyStop(self, true);
                        %return;
                    end
                end
            end
        end
    %% emergency stop if collission or e-stop button is pressed
        function EmergencyStop(self, flag)
                % Check if the emergency stop flag is set
                self.EStopFlag = flag;
                if self.EStopFlag
                    input('Emergency stop triggered. Press any key to undo (we should reset here....or do something dynamic in future') % wait until user is happy to continue
                    self.EStopFlag = false;
                end
            end
    end
end