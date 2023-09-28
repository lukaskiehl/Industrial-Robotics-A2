classdef RobotBaseball < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        % KR = KR6R700CR
        KR
        KRJointAngles
        UR
        URJointAngles
        
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
            
            
             input('press enter to play baseball')
            self.PitchBall()

        end

        %%
        function BuildField(self)
            surf([-1.5,-1.5;0.5,0.5]*5 ...
                ,[-1,1;-1,1]*5 ...
                ,[0,0;0,0] ...
                ,'CData',imread('baseball_field_1.jpg') ...
                ,'FaceColor','texturemap');
            
        end
        %%
        function BuildRobots(self)
            hold on;
            baseTransformKR6R700C = transl(-3,0,0);
            self.KR = KR6R700CR(baseTransformKR6R700C); %store Kuka Robot in the 'KR' robot property.
            self.KRJointAngles = self.KR.model.getpos(); %store joint angles for use
            self.KR.model.animate(self.KRJointAngles); %Draw the robot
            
            baseTransformUR3 = transl(1.5,0,0);
            self.UR = UR3(baseTransformUR3); %store UR3 robot in the UR3 
            self.URJointAngles = self.UR.model.getpos();
            self.UR.model.animate(self.KRJointAngles);
           
            % self.KR.model.teach();
            % self.UR.model.teach();
        end
        %%
       function PitchBall(self)
            
        balls = RobotBalls;
        steps = 30;
        
        input('press enter to pickup ball')
  
        %1.0 Pick up ball
        qpasser1 = [ 0    2.5598    0.4145         0         0         0];
        q1 = zeros(1,6);
        q2 = self.KR.model.ikcon(balls.ballModel{1}.base, qpasser1);
        s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
        qMatrix = nan(steps,6);
             for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
             end
        
       
        checkGroundCollision(self,self.KR,qMatrix);
        for i = 1:steps
                self.KR.model.animate(qMatrix(i,:));
                %pause(0.2);
                drawnow();
        end
        

        % 1.1 Prepare to throw the ball
        q1 = q2;
        q2 = [1.2363    0.0818   -2.0944    0.0000   -0.0122    0.0000];
        s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
        qMatrix = nan(steps,6);
             for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
             end
        checkGroundCollision(self,self.KR,qMatrix);
        for i = 1:steps
                self.KR.model.animate(qMatrix(i,:));
                
                balls.ballModel{1}.base = self.KR.model.fkine(qMatrix(i,:));
                balls.ballModel{1}.animate(0);
                %pause(0.2);
                drawnow();
        end


        % 1.2 Action to throw the ball
        q1 = q2;
        q2 = [ 1.2363    2.8162   -0.7898         0   -0.0122   -1.5272];
        s = lspb(0,1,steps/2); % use trapezoidal velocity method from Lab 4.1
        qMatrix = nan(steps/2,6);
             for i = 1:steps/2
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
             end
        checkGroundCollision(self,self.KR,qMatrix);
        for i = 1:steps/2
                self.KR.model.animate(qMatrix(i,:));
                %pause(0.2); 
        
                if steps/4 >= i
                  balls.ballModel{1}.base = self.KR.model.fkine(qMatrix(i,:));
                  balls.ballModel{1}.animate(0);
                  drawnow();
                  % if i == steps/4
                  %     ballStart = self.KR.model.fkine(qMatrix(i,:))
                  % end
                end
                if i > steps/4
                   % for j = 0:0.01:0.2
                          balls.ballModel{1}.base = balls.ballModel{1}.base.T*transl(.05,0.1,0);
                          balls.ballModel{1}.animate(0);
                          drawnow();
                   % end
                end
        
                drawnow();
        end
       end

       %% Stops program if any joints/end effector get closer than given tolerance to the ground
       function checkGroundCollision(self, robot, qMatrix)
            
           % ill use this later to pass information to check collisions
           %Assumption of method will be:
                %Get array of current stored obstacles.
                %Check the joint trajectory is clear.
                %If not, generate a new joint trajectory around obstacle to
                %some level of tolerance.
            
                tolerance = 0.01; % 

            %i will use getpos after to check each joint position
            for i = 1:size(qMatrix, 1)
                T = robot.model.fkine(qMatrix(i,:)).T;
                
                zPosEndEffector = T(3, 4, end); %also cehck end effector pos
                
                fprintf('At joint config %d, the end effector is %f units from the ground.\n', i, zPosEndEffector);
                % if zPosEndEffector <= tolerance
                %     error('Potential collision at joint config %d for the end effector', i);
                % end
            end
        end
    end
end