%% Setup
clear; clc; clf;
% rosshutdown();
% 
% rosinit();

qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 0.138);

robot = Dobot2(base, qHome);
hold on

z = -0.03;
R1 = [0.27  0   z];
G1 = [0.27  0.1 z];
B1 = [0.27 -0.1 z];

R2 = [0.3  0   z];
G2 = [0.3  0.1 z];
B2 = [0.3 -0.1 z];

destinationRed   = [0.05 0.25 z];
destinationGreen = [0.10 0.25 z];
destinationBlue  = [0.15 0.25 z];

rotation = rpy2r(0, 0, 0);

tokenRed1   = PlotObject('tokenred.ply'  , R1);
tokenGreen1 = PlotObject('tokengreen.ply', G1);
tokenBlue1  = PlotObject('tokenblue.ply' , B1);

tokenRed2   = PlotObject('tokenred.ply'  , R2);
tokenGreen2 = PlotObject('tokengreen.ply', G2);
tokenBlue2  = PlotObject('tokenblue.ply' , B2);

robot.MoveRealDobot(qHomeReal);

%% Pick Red
input("Press Enter to Start");

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
qReal = qMatrix1(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);

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
qReal = qMatrix2(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);
robot.SetGripper(1);

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed1.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix3(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed1.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix4(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);

qPlace = robot.model.ikcon(rt2tr(rotation, destinationRed'), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed1.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix5(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);
pause(0.5);
robot.SetGripper(0);

qMatrix6 = jtraj(qPlace, qPrepare2, 50);
for i = 1:50
    qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
    qMatrix6(i,5) = 0;
    
    qTraj = qMatrix6(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end
qReal = qMatrix6(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);

qMatrix7 = jtraj(qPrepare2, qHome, 50);
for i = 1:50
    qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
    qMatrix7(i,5) = 0;
    
    qTraj = qMatrix7(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end
qReal = qMatrix7(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal);

%% Pick Red 2
qPrepare1 = robot.model.ikcon(rt2tr(rotation, R2'+[0; 0; 0.05]), qHome);
qMatrix1 = jtraj(qHome, qPrepare1, 50);

for i = 1:50
    qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
    qMatrix1(i,5) = 0;
    
    qTraj = qMatrix1(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qPick = robot.model.ikcon(rt2tr(rotation, R2'), qPrepare1);
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
    
    tokenRed2.Move(robot.model.fkine(qTraj));
end

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed2.Move(robot.model.fkine(qTraj));
end

qPlace = robot.model.ikcon(rt2tr(rotation, destinationRed'+[0;0;0.0023]), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed2.Move(robot.model.fkine(qTraj));
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

%% Pick Green 2
qPrepare1 = robot.model.ikcon(rt2tr(rotation, G2'+[0; 0; 0.05]), qHome);
qMatrix1 = jtraj(qHome, qPrepare1, 50);

for i = 1:50
    qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
    qMatrix1(i,5) = 0;
    
    qTraj = qMatrix1(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qPick = robot.model.ikcon(rt2tr(rotation, G2'), qPrepare1);
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
    
    tokenGreen2.Move(robot.model.fkine(qTraj));

end

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationGreen'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen2.Move(robot.model.fkine(qTraj));
end

qPlace = robot.model.ikcon(rt2tr(rotation, destinationGreen'+[0;0;0.0023]), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen2.Move(robot.model.fkine(qTraj));
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

%% Pick Blue 2
qPrepare1 = robot.model.ikcon(rt2tr(rotation, B2'+[0; 0; 0.05]), qHome);
qMatrix1 = jtraj(qHome, qPrepare1, 50);

for i = 1:50
    qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
    qMatrix1(i,5) = 0;
    
    qTraj = qMatrix1(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    pause(0.01);
end

qPick = robot.model.ikcon(rt2tr(rotation, B2'), qPrepare1);
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
    
    tokenBlue2.Move(robot.model.fkine(qTraj));

end

qPrepare2 = robot.model.ikcon(rt2tr(rotation, destinationBlue'+[0;0;0.05]), qPrepare1);
qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
for i = 1:50
    qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
    qMatrix4(i,5) = 0;
    
    qTraj = qMatrix4(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue2.Move(robot.model.fkine(qTraj));
end

qPlace = robot.model.ikcon(rt2tr(rotation, destinationBlue'+[0;0;0.0023]), qPrepare2);
qMatrix5 = jtraj(qPrepare2, qPlace, 50);
for i = 1:50
    qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
    qMatrix5(i,5) = 0;
    
    qTraj = qMatrix5(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue2.Move(robot.model.fkine(qTraj));
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
