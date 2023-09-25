classdef RobotBalls < handle
%% Create Robot Balls using Gavin Paul's Robot Cows file
% Code from modified robotics toolbox UTS
    %#ok<*TRYNC>    

    properties (Constant)
        maxHeight = 2;
    end
    
    properties
        %> Number of balls
        ballCount = 1;
        
        %> A cell structure of \c ballCount cow models
        ballModel;
        
        %> siteSize in meters
        siteSize = [2,2];        
        
        %> Dimensions of the workspace in regard to the site size
        workspaceDimensions;

    end
    
    methods
        %% ...structors
        function self = RobotBalls(ballCount)
            spawnChoice = 2; % can modify spawn mechanism here if needed.  
            % Can add to function as an input variable with self.spawnChoice = spawnChoice 
            % Left in function for ease, so I can call  RobotBalls with no inputs

            if 0 < nargin
                self.ballCount = ballCount;
            end
            
            self.workspaceDimensions = [-self.siteSize(1)/2, self.siteSize(1)/2 ...
                                       ,-self.siteSize(2)/2, self.siteSize(2)/2 ...
                                       ,0,self.maxHeight];
          
            switch spawnChoice % Decide how to spawn balls
                
                case 1
                % Case 1. spawn balls using the i value as a multiplier for x
                % location. Useful for over 9 balls. 
                    for i = 1:self.ballCount
                        self.ballModel{i} = self.GetBallModel(['ball',num2str(i)]);
                        % base tr
                        basePose = transl(-0.1*i,-0.4,0.05)*trotx(pi);
                        self.ballModel{i}.base = basePose;

                        plot3d(self.ballModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                        % Hold on after the first plot (if already on there's no difference)
                        if i == 1 
                            hold on;
                        end
                    end

                case 2
                % Case 2. spawn balls using specific values in nested if
                % statements. Useful for 9 balls. 
                    for i = 1:self.ballCount
                        self.ballModel{i} = self.GetBallModel(['ball',num2str(i)]);
                        % Choose base tr
                        if i== 1
                            basePose = transl(0.4,0,0.1)*trotx(pi); 
                            % rotate each about x to have right axis. 
                        end
                        if i == 2
                            basePose = transl(-0.2,-0.42,0.05)*trotx(pi);
                        end
                        if i == 3
                            basePose = transl(-0.3,-0.4,0.05)*trotx(pi);
                        end
                        if i == 4
                            basePose = transl(-0.4,-0.4,0.05)*trotx(pi);
                        end
                        if i == 5
                            basePose = transl(-0.5,-0.4,0.05)*trotx(pi);
                        end
                        if i == 6
                            basePose = transl(-0.6,-0.4,0.05)*trotx(pi);
                        end
                        if i == 7
                            basePose = transl(-0.7,-0.4,0.05)*trotx(pi);
                        end
                        if i == 8
                            basePose = transl(-0.8,-0.4,0.05)*trotx(pi);
                        end
                        if i == 9
                            basePose = transl(-0.9,-0.4,0.05)*trotx(pi);
                        end
                        if i > 9
                            basePose = transl(0.5*i,-0.3,0.05)*trotx(pi);
                        end
                        self.ballModel{i}.base = basePose;
                        plot3d(self.ballModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                        % Hold on after the first plot (if already on there's no difference)
                        if i == 1 
                            hold on;
                        end

                    end
            end

            axis equal
            % Run below code to delete camlight in figure
            delete(findall(gcf,'Type','light'));
        end
        
        function delete(self)
            for index = 1:self.ballCount
                handles = findobj('Tag', self.ballModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end  
    end     
        
    methods (Static)
        %% GetCowModel
        function model = GetBallModel(name)
            if nargin < 1
                name = 'Ball';
            end
            [faceData,vertexData] = plyread('baseball2.ply','tri');
            link1 = Link('alpha',pi/2,'a',0,'d',.05,'offset',0); % change d offset to change where ball sits on robot end eff
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end    
end

%% Run Code
% balls = RobotBalls
% balls.ballModel{1}.base