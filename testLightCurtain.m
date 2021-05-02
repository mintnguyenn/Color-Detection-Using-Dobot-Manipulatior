%% Update 1 
%successfully make prototype light curtain 
% need to make the cube dispear after it moves one at a time
%% Test light curtain 
close all 
clear all
clc
hold on 

[frontMatrix,backMatrix,leftMatrix,rightMatrix]=drawLightCurtain(0,1);
base = transl(0,0,0);
qDefault = [0,0,0,0,0];
q = [0,0,pi/2,0,0]; 
Dobot = Dobot(base,qDefault);

steps = 50;
Trajectory = jtraj(qDefault,q,steps);
k = -2; 

for i = 1:1:steps
    Dobot.model.animate(Trajectory(i,:)); 
    centerpnt = [k,0,0.15];
    side = 0.3;
    plotOptions.plotFaces = true;
    [vertex,faces,faceNormals] = RectangularPrism(centerpnt-side/2,centerpnt+side/2,plotOptions);
    resultFront = IsCollisionCurtain(frontMatrix,faces,vertex,faceNormals);
    resultBack = IsCollisionCurtain(backMatrix,faces,vertex,faceNormals);
    resultLeft = IsCollisionCurtain(leftMatrix,faces,vertex,faceNormals);
    resultRight = IsCollisionCurtain(rightMatrix,faces,vertex,faceNormals);
    k = k + 0.03;
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
end 


% resultFront = IsCollisionCurtain(frontMatrix,faces,vertex,faceNormals);
% resultBack = IsCollisionCurtain(backMatrix,faces,vertex,faceNormals);
% resultLeft = IsCollisionCurtain(leftMatrix,faces,vertex,faceNormals);
% resultRight = IsCollisionCurtain(rightMatrix,faces,vertex,faceNormals);
%% Draw light curtain 
function [front_,back_,left_,right_] = drawLightCurtain(defaultV,side)
% Plot front 
    height = defaultV;
    for i =1:10
        front_{i}=[-side ,side ,height ;side ,side ,height ];
        height = height+0.1;  
    end
    for i =1:10
        plot3(front_{i}(:,1),front_{i}(:,2),front_{i}(:,3),'r');
    end
% Plot back 
    height = defaultV;
    for i =1:10
        back_{i}=[-side ,-side ,height ;side ,-side ,height ];
        height = height +0.1; 
    end
    for i =1:10
        plot3(back_{i}(:,1),back_{i}(:,2),back_{i}(:,3),'r');
    end
% Plot left
    height = defaultV;
    for i =1:10
        left_{i}=[-side ,-side ,height ;-side ,side ,height ];
        height = height+0.1;  
    end
    for i =1:10
        plot3(left_{i}(:,1),left_{i}(:,2),left_{i}(:,3),'r');
    end
% Plot right
    height = defaultV;
    for i =1:10
        right_{i}=[side ,-side ,height ;side ,side ,height ];
        height = height+0.1;  
    end
    for i =1:10
        plot3(right_{i}(:,1),right_{i}(:,2),right_{i}(:,3),'r');
    end
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