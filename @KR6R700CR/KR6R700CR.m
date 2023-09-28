classdef KR6R700CR < RobotBaseClass
    %% Kuka Robot KR6R700CR

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