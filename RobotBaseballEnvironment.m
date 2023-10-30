classdef RobotBaseballEnvironment < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        EnvironmentObjects;
    end

    methods

        function self = RobotBaseballEnvironment()
            self.EnvironmentObjects = [];
            self.BuildField();
            self.BuildFence();
            self.BuildPeople();
            self.PlaceExtinguisher();
            self.PlaceLightCurtain();
            self.PlaceEStop();

        end

        %% Place people in environment
        function BuildPeople(self)

            hold on;

%             person_1_Pos = [-2, 0, 0]; 
%             person_1 = PlaceObject('personMaleCasual.ply', person_1_Pos);
%             verts = [get(person_1, 'Vertices'), ones(size(get(person_1, 'Vertices'), 1), 1)];
%             verts(:, 1:3) = verts(:, 1:3) * 1; %scale 
%             set(person_1, 'Vertices', verts(:, 1:3));
% 
%             self.AddEnvironmentObject('Person', person_1_Pos, 1);
            % 
            % person_2_Pos = [0, 0, 0];
            % person_2 = PlaceObject('personMaleCasual.ply', person_2_Pos); 
            % verts = [get(person_2, 'Vertices'), ones(size(get(person_2, 'Vertices'), 1), 1)];
            % verts(:, 1:3) = verts(:, 1:3) * 1; %scale 
            % set(person_2, 'Vertices', verts(:, 1:3));
            % 
            % self.AddEnvironmentObject('Person', person_2_Pos, 1);

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
        function BuildField(~)
            surf([-1,-1;1,1]*15 ...
                ,[-1,1;-1,1]*15 ...
                ,[0,0;0,0] ...
                ,'CData',imread('baseball_field_1.jpg') ...
                ,'FaceColor','texturemap');
        end

        %%
        function BuildFence(~)
            hold on;
        
            scalingFactor = 1; 
        
            %fence 1 behind batter
            h_1 = PlaceObject('fence.ply', [0, -3, 14]);
            verts = [get(h_1, 'Vertices'), ones(size(get(h_1, 'Vertices'), 1), 1)] * trotx(pi/2) * trotz(pi/2);
            verts = verts(:, 1:3) * scalingFactor; 
            verts(:, 2) = verts(:, 2) * 0.6; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_1, 'Vertices', verts, 'FaceColor', [0, 0, 0]);

            % 
            h_2 = PlaceObject('fence.ply', [-2.4/1.2, -3, 7.5/1.2]);
            verts = [get(h_2, 'Vertices'), ones(size(get(h_2, 'Vertices'), 1), 1)] * trotx(pi/2) * trotz(pi/4.2 ...
                );
            verts = verts(:, 1:3) * scalingFactor;
            verts(:, 1) = verts(:, 1) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 2) = verts(:, 2) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_2, 'Vertices', verts, 'FaceColor', [0, 0, 0]);

            % 
            h_3 = PlaceObject('fence.ply', [12/0.3, -3, 3.5]);
            verts = [get(h_3, 'Vertices'), ones(size(get(h_3, 'Vertices'), 1), 1)] * trotx(pi/2);
            verts = verts(:, 1:3) * scalingFactor;
            verts(:, 1) = verts(:, 1) * 0.3; %shorterning fence by scaling one axis vert
            verts(:, 2) = verts(:, 2) * 1; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_3, 'Vertices', verts, 'FaceColor', [0, 0, 0]);

            
            h_4 = PlaceObject('fence.ply', [-0/1.2, -3, 10/1.2]);
            verts = [get(h_4, 'Vertices'), ones(size(get(h_4, 'Vertices'), 1), 1)] * -trotx(pi/2) * trotz(pi/4.2) * trotx(pi);
            verts = verts(:, 1:3) * scalingFactor;
            verts(:, 1) = verts(:, 1) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 2) = verts(:, 2) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_4, 'Vertices', verts, 'FaceColor', [0, 0, 0]);

            h_5 = PlaceObject('fence.ply', [0, -3, -14.5]);
            verts = [get(h_5, 'Vertices'), ones(size(get(h_5, 'Vertices'), 1), 1)] * trotx(pi/2) * trotz(pi/2);
            verts = verts(:, 1:3) * scalingFactor; 
            verts(:, 2) = verts(:, 2) * 0.6; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_5, 'Vertices', verts, 'FaceColor', [0, 0, 0]);

            h_6 = PlaceObject('fence.ply', [-0/1.2, -3, 10/1.2]);
            verts = [get(h_6, 'Vertices'), ones(size(get(h_6, 'Vertices'), 1), 1)] * trotx(pi/2) * -trotz(pi/4.2) * trotx(pi);
            verts = verts(:, 1:3) * scalingFactor;
            verts(:, 1) = verts(:, 1) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 2) = verts(:, 2) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_6, 'Vertices', verts, 'FaceColor', [0, 0, 0]);
            
            h_7 = PlaceObject('fence.ply', [0/1.2, -3, -10/1.2]);
            verts = [get(h_7, 'Vertices'), ones(size(get(h_7, 'Vertices'), 1), 1)] * trotx(pi/2) * trotz(pi/4.2 ...
                );
            verts = verts(:, 1:3) * scalingFactor;
            verts(:, 1) = verts(:, 1) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 2) = verts(:, 2) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_7, 'Vertices', verts, 'FaceColor', [0, 0, 0]);

             % 
            h_8 = PlaceObject('fence.ply', [-2.4/1.2, -3, 7.5/1.2]);
            verts = [get(h_8, 'Vertices'), ones(size(get(h_8, 'Vertices'), 1), 1)] * trotx(pi/2) * -trotz(pi/4.2) * troty(pi);
            verts = verts(:, 1:3) * scalingFactor;
            verts(:, 1) = verts(:, 1) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 2) = verts(:, 2) * 1.5; %shorterning fence by scaling one axis vert
            verts(:, 3) = verts(:, 3) * 0.1; %shorterning fence by scaling one axis vert
            set(h_8, 'Vertices', verts, 'FaceColor', [0, 0, 0]);
          
          
           
        end
            
        %% Place fire extinguisher on fence
        function PlaceExtinguisher(self)
            hold on;
            extinguisher_1 = PlaceObject('fireExtinguisherElevated.ply', [2, 13.9, 0.5]);
            verts = [get(extinguisher_1, 'Vertices'), ones(size(get(extinguisher_1, 'Vertices'), 1), 1)] * trotz(pi/2);
            verts(:, 1:3) = verts(:, 1:3) * 1; %scale 
            set(extinguisher_1, 'Vertices', verts(:, 1:3));
        end
             
        
        %% Place fire extinguisher on fence
        function PlaceLightCurtain(self)
            sf = 0.001;
            hold on;
            l_1 = PlaceObject('light_curtain.ply', [10.7/sf, -3.8/sf, 1/sf]);
            verts = [get(l_1, 'Vertices'), ones(size(get(l_1, 'Vertices'), 1), 1)];
            verts(:, 1:3) = verts(:, 1:3) * sf; %scale 
            set(l_1, 'Vertices', verts(:, 1:3));

            l_2 = PlaceObject('light_curtain.ply', [13.9/sf, -3.8/sf, 1/sf]);
            verts = [get(l_2, 'Vertices'), ones(size(get(l_2, 'Vertices'), 1), 1)];
            verts(:, 1:3) = verts(:, 1:3) * sf; %scale 
            set(l_2, 'Vertices', verts(:, 1:3));
        end
        
        %% Place emergency stop wall mounted
        function PlaceEStop(self)
            sf = 0.33;
            hold on;
            eStop_1 = PlaceObject('emergencyStopButton.ply', [10.5, 0.4, 2.85]/sf);
            verts = [get(eStop_1, 'Vertices'), ones(size(get(eStop_1, 'Vertices'), 1), 1)] * trotx(-pi/2);
            verts(:, 1:3) = verts(:, 1:3) * sf; %scale 
            set(eStop_1, 'Vertices', verts(:, 1:3));
            hold off;
            hold on;
            eStop_2 = PlaceObject('emergencyStopButton.ply', [10.5, -0.4, -2.75]/sf);
            verts = [get(eStop_2, 'Vertices'), ones(size(get(eStop_1, 'Vertices'), 1), 1)] * trotx(pi/2);
            verts(:, 1:3) = verts(:, 1:3) * sf; %scale 
            set(eStop_2, 'Vertices', verts(:, 1:3));
            hold off;
        end

    end
end
