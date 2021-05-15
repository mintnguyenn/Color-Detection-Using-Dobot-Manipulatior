%% Setup
clear; clc; clf;
usingRealRobot = false;

if usingRealRobot
rosshutdown();
rosinit();
end

qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 0.138);

robot = Dobot2(base, qHome);
hold on

z = 0.001;
locationRed1   = [0.181  0   z];
locationGreen1 = [0.181  0.1 z];
locationBlue1  = [0.181 -0.1 z];

locationRed2   = [0.291  0   z];
locationGreen2 = [0.291  0.1 z];
locationBlue2  = [0.291 -0.1 z];

destinationRed   = [0.05 0.25 z];
destinationGreen = [0.10 0.25 z];
destinationBlue  = [0.15 0.25 z];

rotation = rpy2r(0, 0, 0);

tokenRed1   = PlotObject('tokenred.ply'  , locationRed1);
tokenGreen1 = PlotObject('tokengreen.ply', locationGreen1);
tokenBlue1  = PlotObject('tokenblue.ply' , locationBlue1);

tokenRed2   = PlotObject('tokenred.ply'  , locationRed2);
tokenGreen2 = PlotObject('tokengreen.ply', locationGreen2);
tokenBlue2  = PlotObject('tokenblue.ply' , locationBlue2);

robot.MoveRealDobot(qHomeReal, usingRealRobot);
%%
% GUI.Joint1SliderValueChanging(qHome(1));

%% Pick Red
input("Press Enter to Start");

qPrepare1 = robot.model.ikcon(rt2tr(rotation, locationRed1'+[0; 0; 0.05]), qHome);
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
robot.MoveRealDobot(qReal, usingRealRobot);

qPick = robot.model.ikcon(rt2tr(rotation, locationRed1'), qPrepare1);
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
robot.MoveRealDobot(qReal, usingRealRobot);
robot.SetGripper(1, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);
pause(0.6);
robot.SetGripper(0, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

%% Pick Red 2
qPrepare1 = robot.model.ikcon(rt2tr(rotation, locationRed2'+[0; 0; 0.05]), qHome);
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
robot.MoveRealDobot(qReal, usingRealRobot);

qPick = robot.model.ikcon(rt2tr(rotation, locationRed2'), qPrepare1);
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
robot.MoveRealDobot(qReal, usingRealRobot);
robot.SetGripper(1, usingRealRobot);

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenRed2.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix3(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix4(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix5(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);
pause(0.6);
robot.SetGripper(0, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);


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
robot.MoveRealDobot(qReal, usingRealRobot);


%% Pick Green
qPrepare1 = robot.model.ikcon(rt2tr(rotation, locationGreen1'+[0; 0; 0.05]), qHome);
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
robot.MoveRealDobot(qReal, usingRealRobot);

qPick = robot.model.ikcon(rt2tr(rotation, locationGreen1'), qPrepare1);
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
robot.MoveRealDobot(qReal, usingRealRobot);
robot.SetGripper(1, usingRealRobot);

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen1.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix3(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix4(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);


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
qReal = qMatrix5(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);
pause(0.6);
robot.SetGripper(0, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

%% Pick Green 2
qPrepare1 = robot.model.ikcon(rt2tr(rotation, locationGreen2'+[0; 0; 0.05]), qHome);
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
robot.MoveRealDobot(qReal, usingRealRobot);

qPick = robot.model.ikcon(rt2tr(rotation, locationGreen2'), qPrepare1);
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
robot.MoveRealDobot(qReal, usingRealRobot);
robot.SetGripper(1, usingRealRobot);

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenGreen2.Move(robot.model.fkine(qTraj));

end
qReal = qMatrix3(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix4(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);


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
qReal = qMatrix5(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);
pause(0.6);
robot.SetGripper(0, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

%% Pick Blue
qPrepare1 = robot.model.ikcon(rt2tr(rotation, locationBlue1'+[0; 0; 0.05]), qHome);
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
robot.MoveRealDobot(qReal, usingRealRobot);

qPick = robot.model.ikcon(rt2tr(rotation, locationBlue1'), qPrepare1);
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
robot.MoveRealDobot(qReal, usingRealRobot);
robot.SetGripper(1, usingRealRobot);

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue1.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix3(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix4(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix5(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);
pause(0.6);
robot.SetGripper(0, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

%% Pick Blue 2
qPrepare1 = robot.model.ikcon(rt2tr(rotation, locationBlue2'+[0; 0; 0.05]), qHome);
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
robot.MoveRealDobot(qReal, usingRealRobot);

qPick = robot.model.ikcon(rt2tr(rotation, locationBlue2'), qPrepare1);
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
robot.MoveRealDobot(qReal, usingRealRobot);
robot.SetGripper(1, usingRealRobot);

qMatrix3 = jtraj(qPick, qPrepare1, 50);
for i = 1:50
    qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
    qMatrix3(i,5) = 0;
    
    qTraj = qMatrix3(i,:);
    robot.model.animate(qTraj);
    drawnow();
    
    tokenBlue2.Move(robot.model.fkine(qTraj));
end
qReal = qMatrix3(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix4(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);

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
qReal = qMatrix5(end,:);
qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
robot.MoveRealDobot(qReal, usingRealRobot);
pause(0.6);
robot.SetGripper(0, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);

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
robot.MoveRealDobot(qReal, usingRealRobot);
