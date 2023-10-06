function RunnerLab2
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Run Code
%Load Environment
close all hidden
clear all
clear classes
close all
clc
balls = RobotBalls;
KR = KR6R700CR;
steps = 30;
UR3StartPos = transl(-5,0.5,0)*trotz(-pi/2);
UR3 = UR3Batter(UR3StartPos);
ballTransl = transl(-0.2,0,-0);

%1.0 Pick up ball
qpasser1 = [ 0    2.5598    0.4145         0         0         0];
krq1 = zeros(1,6);
krq2 = KR.model.ikcon(balls.ballModel{1}.base, qpasser1);
s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
qMatrix = nan(steps,6);
     for i = 1:steps
        qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
     end
for i = 1:steps
        KR.model.animate(qMatrix(i,:));
        drawnow();
end
% 1.1 Prepare to throw the ball
krq1 = krq2;
krq2 = [pi    0.0818   -2.0944    0.0000   -0.0122    0.0000];
s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
qMatrix = nan(steps,6);
     for i = 1:steps
        qMatrix(i,:) = (1-s(i))*krq1 + s(i)*krq2;
     end
for i = 1:steps
        KR.model.animate(qMatrix(i,:));
        balls.ballModel{1}.base = KR.model.fkine(qMatrix(i,:));
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
        KR.model.animate(qMatrix(i,:));

        if steps/1.36 >= i
          balls.ballModel{1}.base = KR.model.fkine(qMatrix(i,:));
          balls.ballModel{1}.animate(0);
          drawnow();
          if i == steps/1.36
              ballStart = KR.model.fkine(qMatrix(i,:))
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
for i = 1:steps
    ballXYZ = balls.ballModel{1}.base.T;
    
    ballXYZ2 = balls.ballModel{1}.base.T;
    ballXYZ2(1,4) = -5;

    urq1 = UR3.model.getpos();
    urq2 = UR3.model.ikcon(ballXYZ2*trotx(pi/2)*trotz(pi/2));

    s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
    qMatrix = nan(steps,6);
    for i = 1:steps
            qMatrix(i,:) = (1-s(i))*urq1 + s(i)*urq2;
    end
    
    for i = 1:steps
            UR3.model.animate(qMatrix(i,:));
            drawnow();
            balls.ballModel{1}.base = balls.ballModel{1}.base.T*ballTransl;
            balls.ballModel{1}.animate(0);
            drawnow();
    end
end

    
    


