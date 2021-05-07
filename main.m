%% Setup
clear; clc; clf;
rosshutdown();

rosinit();

q = [0 1.13 1.84 0];
qHome = [0 q(2) q(3) (pi)-q(2)-q(3) q(4)];
base = eye(4);

robot = Dobot(base, qHome);
hold on

R1 = [0.25  0   0.001];
G1 = [0.25  0.1 0.001];
B1 = [0.25 -0.1 0.001];

R2 = [0.2  0   0.001];
G2 = [0.2  0.1 0.001];
B2 = [0.2 -0.1 0.001];

tokenRed1   = PlotObject('tokenred.ply'  , R1);
tokenGreen1 = PlotObject('tokengreen.ply', G1);
tokenBlue1  = PlotObject('tokenblue.ply' , B1);

tokenRed2   = PlotObject('tokenred.ply'  , R2);
tokenGreen2 = PlotObject('tokengreen.ply', G2);
tokenBlue2  = PlotObject('tokenblue.ply' , B2);

%%
destinationRed   = [0.05 0.2 0.001];
destinationGreen = [0.10 0.2 0.001];
destinationBlue  = [0.15 0.2 0.001];

rotation = rpy2r(0, 0, 0);

%% Pick Red
input("");
qPrepare1 = robot.model.ikcon(rt2tr(rotation, R1'+[0; 0; 0.05]), qHome);
qMatrix1 = jtraj(qHome, qPrepare1, 50);

for i = 1:50
    qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
    qMatrix1(i,5) = 0;
    
    qTraj = qMatrix1(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qPick = robot.model.ikcon(rt2tr(rotation, R1'), qPrepare1);
qMatrix2 = jtraj(qPrepare1, qPick, 50);
for i = 1:50
    qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
    qMatrix2(i,5) = 0;
    
    qTraj = qMatrix2(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qPlace = robot.model.ikcon(rt2tr(rotation, destinationRed'), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qMatrix6 = jtraj(qPlace, qPrepare2, 50);
for i = 1:50
    qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
    qMatrix6(i,5) = 0;
    
    qTraj = qMatrix6(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qMatrix7 = jtraj(qPrepare2, qHome, 50);
for i = 1:50
    qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
    qMatrix7(i,5) = 0;
    
    qTraj = qMatrix7(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

%% Pick Green
qPrepare1 = robot.model.ikcon(rt2tr(rotation, G1'+[0; 0; 0.05]), qHome);
qMatrix1 = jtraj(qHome, qPrepare1, 50);
for i = 1:50
    qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
    qMatrix1(i,5) = 0;
    
    qTraj = qMatrix1(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qPick = robot.model.ikcon(rt2tr(rotation, G1'), qPrepare1);
qMatrix2 = jtraj(qPrepare1, qPick, 50);
for i = 1:50
    qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
    qMatrix2(i,5) = 0;
    
    qTraj = qMatrix2(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationGreen'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qPlace = robot.model.ikcon(rt2tr(rotation, destinationGreen'), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qMatrix6 = jtraj(qPlace, qPrepare2, 50);
for i = 1:50
    qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
    qMatrix6(i,5) = 0;
    
    qTraj = qMatrix6(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qMatrix7 = jtraj(qPrepare2, qHome, 50);
for i = 1:50
    qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
    qMatrix7(i,5) = 0;
    
    qTraj = qMatrix7(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

%% Pick Blue
qPrepare1 = robot.model.ikcon(rt2tr(rotation, B1'+[0; 0; 0.05]), qHome);
qMatrix1 = jtraj(qHome, qPrepare1, 50);
for i = 1:50
    qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
    qMatrix1(i,5) = 0;
    
    qTraj = qMatrix1(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qPick = robot.model.ikcon(rt2tr(rotation, B1'), qPrepare1);
qMatrix2 = jtraj(qPrepare1, qPick, 50);
for i = 1:50
    qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
    qMatrix2(i,5) = 0;
    
    qTraj = qMatrix2(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationBlue'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qPlace = robot.model.ikcon(rt2tr(rotation, destinationBlue'), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue1.Move(robot.model.fkine(qTraj));
%     pause(0.01);
end

qMatrix6 = jtraj(qPlace, qPrepare2, 50);
for i = 1:50
    qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
    qMatrix6(i,5) = 0;
    
    qTraj = qMatrix6(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qMatrix7 = jtraj(qPrepare2, qHome, 50);
for i = 1:50
    qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
    qMatrix7(i,5) = 0;
    
    qTraj = qMatrix7(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end