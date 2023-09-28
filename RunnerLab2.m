function RunnerLab2
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Run Code
%Load Environment
clf;
close all;
balls = RobotBalls;
KR = KR6R700CR;
steps = 30;
KR.model.teach();


input('press enter to pickup ball')

%1.0 Pick up ball
qpasser1 = [ 0    2.5598    0.4145         0         0         0];
q1 = zeros(1,6);
q2 = KR.model.ikcon(balls.ballModel{1}.base, qpasser1);
s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
qMatrix = nan(steps,6);
     for i = 1:steps
        qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
     end
for i = 1:steps
        KR.model.animate(qMatrix(i,:));
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
for i = 1:steps
        KR.model.animate(qMatrix(i,:));
        balls.ballModel{1}.base = KR.model.fkine(qMatrix(i,:));
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
for i = 1:steps/2
        KR.model.animate(qMatrix(i,:));
        %pause(0.2); 

        if steps/4 >= i
          balls.ballModel{1}.base = KR.model.fkine(qMatrix(i,:));
          balls.ballModel{1}.animate(0);
          drawnow();
          if i == steps/4
              ballStart = KR.model.fkine(qMatrix(i,:))
          end
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

%% Collision check 






