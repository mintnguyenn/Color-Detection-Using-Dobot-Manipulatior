%% Conclusion and update 
%% Update 1
% Check for collision , able to pint point the joint positions that collide 
% with object
% Will work on hard code obstacle avoidance 
%  Will work on the RMRC
%% Update 2 
% Check for collision , able to pint point the joint positions that collide 
% with object
% Able hard code obstacle avoidance 
%  Will work on the RMRC
%% Collision test 
close all
clear all 
clc 

% brick = plotObject_byTon(transl(0,0,0.15),troty(pi/2),'Brick.ply');
base = transl(0.25,0.25,0);
q_pickup = [0,0,pi/2,0,0];
q = [0,0,pi/3,0,0]; 
q1 = [pi/2,0,pi/3,0,0];
q_putdown = [pi/2,0,pi/2,0,0];
steps = 50; 

Dobot = Dobot(base,q);

% centerpnt = [0,0,0.15];
centerpnt = [0.5,0.5,0.15];
side = 0.3;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2,centerpnt+side/2,plotOptions);

Trajectory = jtraj(q_pickup,q_putdown,steps);
trajectoryDefault_No1 = jtraj(q_pickup,q,steps);
trajectoryDefault_No2 = jtraj(q,q1,steps);
trajectoryDefault_No3 = jtraj(q1,q_putdown,steps);

% Collision prediction 
for i = 1:1:steps
%     resultBrick(i) = IsCollision(Dobot.model,Trajectory(i,:),brick.fData,brick.vData,brick.faceNormals);
    result(i) = IsCollision(Dobot.model,Trajectory(i,:),faces,vertex,faceNormals);
    if result(i) == 0
        Collide = 0;
    else
        Collide = 1;
        disp(['The Dobot collide with an object at joint pose:']);
        disp([num2str(Trajectory(i,:))]);
        break
    end
end
 if Collide == 1
    disp(['The Dobot switchs to alternative movements']);
    for i = 1:1:steps
        Dobot.model.animate(trajectoryDefault_No1(i,:));  
        pause(0.1);
    end
    for i = 1:1:steps
        Dobot.model.animate(trajectoryDefault_No2(i,:));  
        pause(0.1);
    end
    for k = 1:1:steps
        Dobot.model.animate(trajectoryDefault_No3(k,:));  
        pause(0.1);
    end
 end
 
  if Collide == 0
    for i = 1:1:steps
        Dobot.model.animate(Trajectory(i,:));  
        pause(0.5);
    end
 end

% % Primitive obstacle avoidance
%         disp(['The Dobot collide with an object at joint pose:']);
%         disp([num2str(Trajectory(i+1,:))]);
%         qDefault = [0,0,0,0,0];
%         trajectoryDefault_No1 = jtraj(q,qDefault,steps);
%         trajectoryDefault_No2 = jtraj(qDefault,q1,steps);
%         disp(['The Dobot switchs to alternative movements']);
%         for k = 1:1:steps
%             Dobot.model.animate(trajectoryDefault_No1(k,:));  
%             pause(0.1);
%         end
%         for k = 1:1:steps
%             Dobot.model.animate(trajectoryDefault_No2(k,:));  
%             pause(0.1);
%         end
%         for k = 1:1:steps
%             Dobot.model.animate(trajectoryDefault_No3(k,:));  
%             pause(0.1);
%         end

 
%% RMRC
% function qMatrix = GenerateRMRC(robot,pose,steps,deltaT,epsilon)
%     m = zeros(steps,1); 
%     qMatrix = zeros(steps,6); 
%     qdot = zeros(steps,6);
%     x = zeros(6,steps);
%     s = lspb(0,1,steps);
%     q1 = robot.model.getpos; 
%     q2 = robot.ikine(pose);
%     T1 = robot.fkine(q1);
%     T2 = pose; 
%     x1 = [T1(1:3,4);tr2rpy(T1)'];
%     x2 = [T2(1:3,4);tr2rpy(T2)'];
%     for i=1:steps
%         x(:,i) = (1-s(i))*x1 + s(i)*x2;
%     end
%     for i = 1: steps - 1 
%         deltaX = x(1:3,i+1) - T(1:3,4); 
%         m(i) = sqrt(det(J*J'));
%         if m(i) < epsilon  % If manipulability is less than given threshold
%             lambda = sqrt((1 - (m(i)/epsilon)^2)*(5E-2)^2);
%         else
%             lambda = 0;
%         end
%     end
%     
% end

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