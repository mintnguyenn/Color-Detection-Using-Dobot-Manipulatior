close all 
clc 
%% Set up for Dobot
qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 1.05);

dobot = Dobot2(base, qHome);
hold on;
%% Set up for environment
locationRedTray = [0.05 0.25 0.91];
locationTable = [0,0,0.75];
locationFire = [1.5,0.5,0.45];
locationLightCurtain1 = [0.8,0.8,1.5];
locationLightCurtain2 = [0.8,-0.8,1.5];
locationLightCurtain3 = [-0.8,-0.8,1.5];
locationLightCurtain4 = [-0.8,0.8,1.5];
locationE_Stop = [1.5,0,0.92];

[frontMatrix,backMatrix,leftMatrix,rightMatrix] = drawLightCurtain();
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
k = -1.5;
steps = 50;
q = [pi/2    0.7862    1.5690    0.7864         0];
Trajectory = jtraj(qHome,q,steps);

for i = 1:1:steps
    dobot.model.animate(Trajectory(i,:)); 
    stepDone = i; 
    centerpnt = [k,0,1.3];
    side = 0.3;
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals,facePatchTest] = RectangularPrism(centerpnt-side/2,centerpnt+side/2,plotOptions);
    [resultFront,resultBack,resultLeft,resultRight] = lightCurtainCheck(frontMatrix,backMatrix,leftMatrix,leftMatrix,faces,vertex,faceNormals);
    k = k + 0.02;
    if resultFront == 1
        break
    end
    if resultBack == 1
        break
    end   
    if resultLeft == 1
        break
    end
    if resultRight == 1
        break
    end
    pause(0.1)
    delete(facePatchTest);
end 

pause(5); 
delete(facePatchTest);
b = stepDone;

%move the cube away . if the cube isnt collided with the light curtain then
%the robot continue to moves in it trajectory 
for a = k:-0.02:-1.5
    centerpnt = [a,0,1.3];
    side = 0.3;
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals,facePatchTest] = RectangularPrism(centerpnt-side/2,centerpnt+side/2,plotOptions);
    [resultFront,resultBack,resultLeft,resultRight] = lightCurtainCheck(frontMatrix,backMatrix,leftMatrix,leftMatrix,faces,vertex,faceNormals);
    if (resultFront == 0) && (resultBack == 0) && (resultLeft == 0) && (resultRight ==0)
       b = b+1;
       if b == 50
           b = 49;
       end  
       dobot.model.animate(Trajectory(b,:));
    end
    pause(0.1)
    delete(facePatchTest);
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
%% Check LightCurtain  collision 
function [resultFront,resultBack,resultLeft,resultRight] = lightCurtainCheck(frontMatrix,backMatrix,leftMatrix,rightMatrix,faces,vertex,faceNormals)
    resultFront = IsCollisionCurtain(frontMatrix,faces,vertex,faceNormals);
    resultBack = IsCollisionCurtain(backMatrix,faces,vertex,faceNormals);
    resultLeft = IsCollisionCurtain(leftMatrix,faces,vertex,faceNormals);
    resultRight = IsCollisionCurtain(rightMatrix,faces,vertex,faceNormals);
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
%% IsCollisionCurtain
function [result,i] = IsCollisionCurtain(curtainDir,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

    % Go through each link and also each triangle face
    for i = 1 : size(curtainDir,2)-1   
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,curtainDir{i}(1,:),curtainDir{i}(2,:)); 
            if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                display('Warning');
                display('Perimeter has been reached');
                display('Stop!!!');
                result = true;
                if returnOnceFound
                    return
                end
            end
        end    
    end

end
%