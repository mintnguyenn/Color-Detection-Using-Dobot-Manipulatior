classdef TokenSorterDobot < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        usingRealRobot;
        
        base;
        qHome;
        qHomeReal = [0 0.7862 0.7844 0];
        robot;
        
        locationRed1     = [0.181  0   0.001];
        locationGreen1   = [0.181  0.1 0.001];
        locationBlue1    = [0.181 -0.1 0.001];
        locationRed2     = [0.291  0   0.001];
        locationGreen2   = [0.291  0.1 0.001];
        locationBlue2    = [0.291 -0.1 0.001];
        
        destinationRed   = [0.05 0.25 0.001];
        destinationGreen = [0.10 0.25 0.001];
        destinationBlue  = [0.15 0.25 0.001];
        
        tokenRed1;
        tokenGreen1;
        tokenBlue1;
        tokenRed2;
        tokenGreen2;
        tokenBlue2;
    end
    
    methods
%% Constructor
        function self = TokenSorterDobot()
            self.PlotWorkspace();
        end

%% Plot workspace
        function PlotWorkspace(self)
            self.base = transl(0, 0, 0.138);
            self.qHome = [0 0.7862 1.5690 0.7864 0];
            
            self.robot = Dobot2(self.base, self.qHome);
            hold on
            
            self.tokenRed1   = PlotObject('tokenred.ply'  , self.locationRed1);
            self.tokenGreen1 = PlotObject('tokengreen.ply', self.locationGreen1);
            self.tokenBlue1  = PlotObject('tokenblue.ply' , self.locationBlue1);
            self.tokenRed2   = PlotObject('tokenred.ply'  , self.locationRed2);
            self.tokenGreen2 = PlotObject('tokengreen.ply', self.locationGreen2);
            self.tokenBlue2  = PlotObject('tokenblue.ply' , self.locationBlue2);
        end
        
%%
        function PickAndPlace(self, usingRealRobot)
            self.usingRealRobot = usingRealRobot;
            if self.usingRealRobot
                rosshutdown();
                rosinit();
            end
            
            self.robot.MoveRealDobot(self.qHomeReal, self.usingRealRobot);
            
            input("Press Enter to Start");
            
            rotation = rpy2r(0, 0, 0);
            
 % Pick Red 1         
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationRed1'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationRed1'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            
            for i = 1:50
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Red 2
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationRed2'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);

            for i = 1:50
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationRed2'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'+[0;0;0.0023]), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Green 1
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen1'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow(); 
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen1'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();  
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);


            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();  
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Green 2
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen2'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);

            for i = 1:50
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen2'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'+[0;0;0.0023]), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow(); 
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Blue 1
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue1'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue1'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow(); 
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue1.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();  
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Blue 2
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue2'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);

            for i = 1:50
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow(); 
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue2'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'+[0;0;0.0023]), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue2.Move(self.robot.model.fkine(qTraj));
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();  
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
        end

%%
        function CollisionAvoidance(self)
            
        end

%%
        function LightCurtain(self)
    
        end

%% Color detection
        function [location] = ColorDetection(self)
            location = zeros(6,3);
            A = rossubscriber('/camera/color/image_raw');
            RGB = readImage(A.LatestMessage);
            I = rgb2hsv(RGB);
            
            % Red color threshoder
            redChannel1Min = 0.909;
            redChannel1Max = 0.126;
            redChannel2Min = 0.432;
            redChannel2Max = 0.844;
            redChannel3Min = 0.000;
            redChannel3Max = 1.000;
            
            BWRed = ( (I(:,:,1) >= redChannel1Min) | (I(:,:,1) <= redChannel1Max) ) & ...
            (I(:,:,2) >= redChannel2Min ) & (I(:,:,2) <= redChannel2Max) & ...
            (I(:,:,3) >= redChannel3Min ) & (I(:,:,3) <= redChannel3Max);
        
            BRed = bwboundaries(BWRed,'noholes');
            
            % Green color threshoder
            greenChannel1Min = 0.323;
            greenChannel1Max = 0.546;
            greenChannel2Min = 0.186;
            greenChannel2Max = 1.000;
            greenChannel3Min = 0.000;
            greenChannel3Max = 1.000;
            
            BWGreen = (I(:,:,1) >= greenChannel1Min ) & (I(:,:,1) <= greenChannel1Max) & ...
            (I(:,:,2) >= greenChannel2Min ) & (I(:,:,2) <= greenChannel2Max) & ...
            (I(:,:,3) >= greenChannel3Min ) & (I(:,:,3) <= greenChannel3Max);
        
            BGreen = bwboundaries(BWGreen,'noholes');
            
            % Blue color thresholder
            
            % 
            BRed   = NoiseFilter(BRed);
            BGreen = NoiseFilter(BGreen);
            
            xRed1   = BRed{1,1}(:,2);
            yRed1   = BRed{1,1}(:,1);
            xRed2   = BRed{2,1}(:,2);
            yRed2   = BRed{2,1}(:,1);
            
            xGreen1 = BGreen{1,1}(:,2);
            yGreen1 = BGreen{1,1}(:,1);
            xGreen2 = BGreen{2,1}(:,2);
            yGreen2 = BGreen{2,1}(:,1);
            
            [xRed1Center, yRed1Center] = Circlefit(xRed1, yRed1);
            [xRed2Center, yRed2Center] = Circlefit(xRed2, yRed2);
            [xGreen1Center, yGreen1Center] = Circlefit(xGreen1, yGreen1);
            [xGreen2Center, yGreen2Center] = Circlefit(xGreen2, yGreen2);
            
        end

    end
end

