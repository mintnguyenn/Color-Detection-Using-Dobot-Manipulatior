% Create base on the work of Toan + Minh + LabEx - Test Collision Avoidance +
% TokenSorterDobot
% x = 0.2; y = 0.25; z = 0.1
% function [] = CollisionAvoidance(x,y,z)

clc
clf

xCen = 0.2;
yCen = 0.25;
zCen = 1;
%For TokenSorterDobot
% qTeach = [0 0.7862 1.5690 0.7864 0];
% qHomeReal = [0 0.7862 0.7844 0];

%% Set up for Dobot
qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 1.05);

dobot = Dobot2(base, qHome);
hold on;
% axis ([-0.5, 0.5, -0.5, 0.5, -0.6, 0.8]);

%% Set up for Token
locationRed     = [0.181  0   0.91];
locationGreen   = [0.181  0.1 0.91];
destinationRed   = [0.05 0.25 0.91];
destinationGreen = [0.10 0.25 0.91];
locationRedTray = [0.05 0.25 0.91];
locationTable = [0,0,0.75];
locationFire = [1.5,0.5,0.45];
locationLightCurtain1 = [0.8,0.8,1.5];
locationLightCurtain2 = [0.8,-0.8,1.5];
locationLightCurtain3 = [-0.8,-0.8,1.5];
locationLightCurtain4 = [-0.8,0.8,1.5];
locationE_Stop = [1.5,0,0.92];
% for GUI
% self.tokenRed1   = PlotObject('tokenred.ply'  , self.locationRed1);
% self.tokenGreen1 = PlotObject('tokengreen.ply', self.locationGreen1);

% for Testing
[frontMatrix,backMatrix,leftMatrix,rightMatrix] = drawLightCurtain();
tokenRed = PlotObject('tokenred.ply', locationRed);
tokenGreen = PlotObject('tokengreen.ply', locationGreen);
PlotObject('red_tray.ply' , locationRedTray);
PlotObject('table.ply' , locationTable);
PlotObject('fire.ply' , locationFire);
PlotObject('lightcurtain.ply' , locationLightCurtain1);
PlotObject('lightcurtain.ply' , locationLightCurtain2);
PlotObject('lightcurtain.ply' , locationLightCurtain3);
PlotObject('lightcurtain.ply' , locationLightCurtain4);
PlotObject('estop1.ply' , locationE_Stop);
surf([-2,-2;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('s_concrete.jpg'),'FaceColor','texturemap');

%% Set up the Obstacle
centerpnt = [xCen,  yCen, zCen];
side = 0.2;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2,centerpnt+side/2,plotOptions);
%% Trajectory
rotation = rpy2r(0, 0, 0);
qPrepare1 = dobot.model.ikcon(rt2tr(rotation, locationRed'+[0; 0; 0.05]), qHome);
qMatrix{1} = jtraj(qHome, qPrepare1, 50);

qPick = dobot.model.ikcon(rt2tr(rotation, locationRed'), qPrepare1);
qMatrix{2} = jtraj(qPrepare1, qPick, 50);

qMatrix{3} = jtraj(qPick, qPrepare1, 50);

qPrepare2 = dobot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.05]), qPrepare1);
qMatrix{4} = jtraj(qPrepare1, qPrepare2, 50);

qPlace = dobot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.0023]), qPrepare2);
qMatrix{5} = jtraj(qPrepare2, qPlace, 50);

qMatrix{6} = jtraj(qPlace, qPrepare2, 50);

qMatrix{7} = jtraj(qPrepare2, qHome, 50);

%% Collision prediction
for i = 1:1:7
    for j = 1:1:50
        result(j) = IsCollision(dobot.model, qMatrix{i}(j,:), faces, vertex, faceNormals);
        if result(j) == 0
            Collide = 0;
        else
            Collide = 1;
            disp(['The Dobot collide with an object at joint pose:']);
            disp([num2str(qMatrix{i}(j,:))]);
            stopTraj = i;
            stopStep = j;
            break %% To break the for loop
        end
    end
    
    if Collide == 1
        break
    end
end

%% Go to
if Collide == 0
    for i = 1:1:7
        for j = 1:1:50
            qMatrix{i}(j,4) = pi - qMatrix{i}(j,2) - qMatrix{i}(j,3);
            qMatrix{i}(j,5) = 0;
            
            qTraj = qMatrix{i}(j,:);
            dobot.model.animate(qTraj);
            drawnow();
            
            if i == 4 && i ==5
                tokenRed.Move(dobot.model.fkine(qTraj))
            end
        end
    end
elseif Collide == 1
    for i = 1:1:stopTraj
        for j = 1:1:(stopStep - 5)
            qMatrix{i}(j,4) = pi - qMatrix{i}(j,2) - qMatrix{i}(j,3);
            qMatrix{i}(j,5) = 0;
            
            qTraj = qMatrix{i}(j,:);
            dobot.model.animate(qTraj);
            drawnow();
            
            if i == 4 || i ==5
                tokenRed.Move(dobot.model.fkine(qTraj))
            end
        end
    end
end
%% Another Trajectory
if Collide == 1
    disp(['Go to the goal through another trajectory']);
    currentPos = dobot.model.getpos();
    nextPos = dobot.model.ikcon(dobot.model.fkine(currentPos) * transl(0, 0, 0.23));
    qPrepare3 = dobot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.23]), nextPos);
    qMatrix{8} = jtraj(currentPos, nextPos ,50);
    qMatrix{9} = jtraj(nextPos, qPrepare3, 50);
    qMatrix{10} = jtraj(qPrepare3, qPlace, 50);
    for i = 8:1:10
        for j = 1:1:50
            qMatrix{i}(j,4) = pi - qMatrix{i}(j,2) - qMatrix{i}(j,3);
            qMatrix{i}(j,5) = 0;
            
            qTraj = qMatrix{i}(j,:);
            dobot.model.animate(qTraj);
            drawnow();
            tokenRed.Move(dobot.model.fkine(qTraj))
            pause(0.02)
        end
    end
end

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is
% inside (result == 1) or
% outside a triangle (result ==0 )
    function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)
        
        u = triangleVerts(2,:) - triangleVerts(1,:);
        v = triangleVerts(3,:) - triangleVerts(1,:);
        
        uu = dot(u,u);
        uv = dot(u,v);
        vv = dot(v,v);
        
        w = intersectP - triangleVerts(1,:);
        wu = dot(w,u);
        wv = dot(w,v);
        
        D = uv * uv - uu * vv;
        
        % Get and test parametric coords (s and t)
        s = (uv * wv - vv * wu) / D;
        if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
            result = 0;
            return;
        end
        
        t = (uv * wu - uu * wv) / D;
        if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
            result = 0;
            return;
        end
        
        result = 1;                      % intersectP is in Triangle
    end

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
    function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
        if nargin < 6
            returnOnceFound = true;
        end
        result = false;
        
        for qIndex = 1:size(qMatrix,1)
            % Get the transform of every joint (i.e. start and end of every link)
            tr = GetLinkPoses(qMatrix(qIndex,:), robot);
            
            % Go through each link and also each triangle face
            for i = 1 : size(tr,3)-1
                for faceIndex = 1:size(faces,1)
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                        display('Intersection');
                        result = true;
                        if returnOnceFound
                            return
                        end
                    end
                end
            end
        end
    end
%% GetLinkPoses
% q - robot joint angles
% robot -  seriallink robot model
% transforms - list of transforms
    function [ transforms ] = GetLinkPoses( q, robot)
        
        links = robot.links;
        transforms = zeros(4, 4, length(links) + 1);
        transforms(:,:,1) = robot.base;
        
        for i = 1:length(links)
            L = links(1,i);
            
            current_transform = transforms(:,:, i);
            
            current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
            transforms(:,:,i + 1) = current_transform;
        end
    end
%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
    function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
        if nargin < 3
            maxStepRadians = deg2rad(1);
        end
        
        steps = 2;
        while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
            steps = steps + 1;
        end
        qMatrix = jtraj(q1,q2,steps);
    end
%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
    function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
        if nargin < 2
            maxStepRadians = deg2rad(1);
        end
        
        qMatrix = [];
        for i = 1: size(waypointRadians,1)-1
            qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
        end
    end
    %% Draw light curtain 
    function [front_,back_,left_,right_] = drawLightCurtain()
% Plot front 
    height = 0.95;
    side = 0.8;
    for i =1:10
        front_{i}=[-side ,side ,height ;side ,side ,height ];
        height = height+0.1;  
    end
    for i =1:10
        plot3(front_{i}(:,1),front_{i}(:,2),front_{i}(:,3),'r');
    end
% Plot back 
    height = 0.95;
    side = 0.8;
    for i =1:10
        back_{i}=[-side ,-side ,height ;side ,-side ,height ];
        height = height +0.1; 
    end
    for i =1:10
        plot3(back_{i}(:,1),back_{i}(:,2),back_{i}(:,3),'r');
    end
% Plot left
    height = 0.95;
    side = 0.8;
    for i =1:10
        left_{i}=[-side ,-side ,height ;-side ,side ,height ];
        height = height+0.1;  
    end
    for i =1:10
        plot3(left_{i}(:,1),left_{i}(:,2),left_{i}(:,3),'r');
    end
% Plot right
    height = 0.95;
    side = 0.8;
    for i =1:10
        right_{i}=[side ,-side ,height ;side ,side ,height ];
        height = height+0.1;  
    end
    for i =1:10
        plot3(right_{i}(:,1),right_{i}(:,2),right_{i}(:,3),'r');
    end
end
% end