classdef Dobot < handle
    properties
        %> Robot model
        model;
    
        %>
        workspace = [-2 2 -2 2 -0.3 2];
    end
    
    methods
        function self = Dobot(base, pose)    
            %> Define the boundaries of the workspace

            % robot = 
            self.CreateDobot(base);
            self.PlotAndColourDobot(pose);

        end

        %% GetDobot
        % Given a name (optional), create and return a Dobot model
        function CreateDobot(self, base)    
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.001);
            name = ['Dobot',datestr(now,'yyyymmddTHHMMSSFFF')];

            % Create the UR3 model mounted on a linear rail
            L(1) = Link('d',0.138,   'a',0,     'alpha',-pi/2, 'offset',0,     'qlim',[deg2rad(-135),deg2rad(135)]);
            L(2) = Link('d',0,       'a',0.135, 'alpha',0,     'offset',-pi/2, 'qlim',[deg2rad(5)   ,deg2rad(80)]);
            L(3) = Link('d',0,       'a',0.147, 'alpha',0,     'offset',0,     'qlim',[deg2rad(15)  ,deg2rad(170)]);
            L(4) = Link('d',0,       'a',0.061, 'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-90) ,deg2rad(90)]);
            L(5) = Link('d',-0.077, 'a',0,     'alpha',0,      'offset',0,     'qlim',[deg2rad(-85) ,deg2rad(85)]);
 
            self.model = SerialLink(L,'name',name);

            % Rotate robot to the correct orientation
            self.model.base = base;
        end
        
        %% PlotAndColourRobot
        function PlotAndColourDobot(self, pose)%robot,workspace
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Dobot',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
            
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            % Display robot
            self.model.plot3d(pose,'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
        %% Line Plane Intersection
        function [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,point1OnLine,point2OnLine)

            intersectionPoint = [0 0 0];
            u = point2OnLine - point1OnLine;
            w = point1OnLine - pointOnPlane;
            D = dot(planeNormal,u);
            N = -dot(planeNormal,w);
            check = 0; %#ok<NASGU>
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check = 2;
                    return
                else
                    check = 0;       %no intersection
                    return
                end
            end

%compute the intersection parameter
            sI = N / D;
            intersectionPoint = point1OnLine + sI.*u;

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end
        end
        
        %% Set Initialise Dobot
        function SetInitialiseDobot(self)
            [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
            safetyStateMsg.Data = 2;
            send(safetyStatePublisher,safetyStateMsg);
        end
        
        %% Move Real Dobot
        function MoveRealDobot(self, qReal, usingRealRobot)
            if usingRealRobot
                jointTarget = qReal; % Remember that the Dobot has 4 joints by default.

                [self.targetJointTrajPub,self.targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
                self.trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
                self.trajectoryPoint.Positions = jointTarget;
                self.targetJointTrajMsg.Points = self.trajectoryPoint;

                send(self.targetJointTrajPub,self.targetJointTrajMsg);
            end
        end
        
        %% Set Gripper
        function SetGripper(self, state, usingRealRobot)
            if usingRealRobot
                [self.toolStatePub, self.toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
                self.toolStateMsg.Data = [state]; % Send 1 for on and 0 for off 
                send(self.toolStatePub,self.toolStateMsg);
            end
        end
    end
end