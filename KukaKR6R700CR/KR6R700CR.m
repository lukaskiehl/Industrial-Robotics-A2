classdef KR6R700CR < RobotBaseClass
    %% Linear UR3 on Rail
    % Created from LinearUR5 and UR3 robots in UTS toolbox

    properties(Access = public)              
        plyFileNameStem = 'KukaKR6R700CR'; % Load in ply files. Ply files from Kim (2023)
    end
    
    methods
%% Define robot Function 
function self = KR6R700CR(baseTr) % Constructor to create instance (model)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4)*transl(0,0,0.2);				
            end
            self.model.base = self.model.base.T * baseTr; %* trotx(pi/2) %* troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the UR5 model mounted on a linear rail
            link(1) = Link('d',0.183,'a',0.025,'alpha',-pi/2,'qlim',deg2rad([-170 170]), 'offset',0); % DH parameters from (Lloyd et al., 2022)
            link(2) = Link('d',0,'a',-0.315,'alpha',0,'qlim', deg2rad([-10 225]), 'offset',0);        % Joint limits from Kuka (2022).
            link(3) = Link('d',0,'a',-0.035,'alpha',pi/2,'qlim', deg2rad([-120 156]), 'offset', 0);
            link(4) = Link('d',0.365,'a',0,'alpha',-pi/2,'qlim',deg2rad([-185 185]),'offset', 0);
            link(5) = Link('d',0,'a',0,'alpha',pi/2,'qlim',deg2rad([-120,120]), 'offset',0);
            link(6) = Link('d',0.08,'a',0,'alpha',0,'qlim',deg2rad([-350,350]), 'offset', 0);
            
            % Incorporate joint limits and offsets

            % Stand model starting upright
            % link(1).offset = -pi/2;
            % link(5).offset = -pi/2; 

            self.model = SerialLink(link,'name',self.name); % Create robot
            % Call robot using robot.model
        end
     end
  end

% % Run Code
% %Load Environment
% balls = RobotBalls
% KR = KR6R700CR
% steps = 30;
% 
% %1.0 Pick up ball
% qpasser1 = [ 0    2.5598    0.4145         0         0         0];
% q1 = zeros(1,6);
% q2 = KR.model.ikcon(balls.ballModel{1}.base, qpasser1);
% s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
% qMatrix = nan(steps,6);
%      for i = 1:steps
%         qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
%      end
% for i = 1:steps
%         KR.model.animate(qMatrix(i,:));
%         drawnow();
% end
% % 1.1 Prepare to throw the ball
% q1 = q2;
% q2 = [1.2363    0.0818   -2.0944    0.0000   -0.0122    0.0000];
% s = lspb(0,1,steps); % use trapezoidal velocity method from Lab 4.1
% qMatrix = nan(steps,6);
%      for i = 1:steps
%         qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
%      end
% for i = 1:steps
%         KR.model.animate(qMatrix(i,:));
%         balls.ballModel{1}.base = KR.model.fkine(qMatrix(i,:));
%         balls.ballModel{1}.animate(0);
%         drawnow();
% end
% % 1.2 Action to throw the ball
% q1 = q2;
% q2 = [ 1.2363    2.8162   -0.7898         0   -0.0122   -1.5272];
% s = lspb(0,1,steps/2); % use trapezoidal velocity method from Lab 4.1
% qMatrix = nan(steps/2,6);
%      for i = 1:steps/2
%         qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
%      end
% for i = 1:steps/2
%         KR.model.animate(qMatrix(i,:));
% 
%         if steps/4 >= i
%           balls.ballModel{1}.base = KR.model.fkine(qMatrix(i,:));
%           balls.ballModel{1}.animate(0);
%           drawnow();
%           %if i == steps/4
%               ballStart = KR.model.fkine(qMatrix(steps/4,:))
%           %end
%         end
%         drawnow();
% 
% end

% 
% 
