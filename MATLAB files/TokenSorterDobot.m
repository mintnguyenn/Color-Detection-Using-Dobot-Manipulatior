classdef TokenSorterDobot < handle
    properties
        usingRealRobot;
        eStopState = false;
        
        base;
        qHome;
        qTeach = [0 0.7862 1.5690 0.7864 0];
        qHomeReal = [0 0.7862 0.7844 0];
        robot;
        
        frontMatrix;
        backMatrix;
        leftMatrix;
        rightMatrix;
        vertex;
        faces;
        faceNormals;
        facePatchTest;
        resultFront;
        resultBack;
        resultLeft;
        resultRight;
       
        locationRed1     = [0.181  0   0.91];
        locationGreen1   = [0.181  0.1 0.91];
        locationBlue1    = [0.181 -0.1 0.91];
        locationRed2     = [0.291  0   0.91];
        locationGreen2   = [0.291  0.1 0.91];
        locationBlue2    = [0.291 -0.1 0.91];
        
        locationRedTray = [0.05 0.25 0.91];
        locationBlueTray = [0.15 0.25 0.91];
        locationGreenTray = [0.10 0.25 0.91];
        locationTable = [0,0,0.75];
        locationFire = [1.5,0.5,0.45];
        locationFence = [0,0,0]; 
        locationLightCurtain1 = [0.8,0.8,1.5];
        locationLightCurtain2 = [0.8,-0.8,1.5];
        locationLightCurtain3 = [-0.8,-0.8,1.5];
        locationLightCurtain4 = [-0.8,0.8,1.5];
        LocationE_Stop = [1.5,0,0.92];
        
        destinationRed   = [0.05 0.25 0.91];
        destinationGreen = [0.10 0.25 0.91];
        destinationBlue  = [0.15 0.25 0.91];
        
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
            self.base = transl(0, 0, 0.91);
            self.qHome = [0 0.7862 1.5690 0.7864 0];
            
            self.robot = Dobot(self.base, self.qHome);
            hold on
            
            side = 0.3;
            k = -2; 
            centerpnt = [k,0,1.3];
            plotOptions.plotFaces = true;
            [self.vertex,self.faces,self.faceNormals,self.facePatchTest] = RectangularPrism(centerpnt-side/2,centerpnt+side/2,plotOptions);
            
            [self.frontMatrix,self.backMatrix,self.leftMatrix,self.rightMatrix] = self.drawLightCurtain();
  
            self.tokenRed1   = PlotObject('tokenred.ply'  , self.locationRed1);
            self.tokenGreen1 = PlotObject('tokengreen.ply', self.locationGreen1);
            self.tokenBlue1  = PlotObject('tokenblue.ply' , self.locationBlue1);
            self.tokenRed2   = PlotObject('tokenred.ply'  , self.locationRed2);
            self.tokenGreen2 = PlotObject('tokengreen.ply', self.locationGreen2);
            self.tokenBlue2  = PlotObject('tokenblue.ply' , self.locationBlue2);
            
            PlotObject('red_tray.ply' , self.locationRedTray);
            PlotObject('blue_tray.ply' , self.locationBlueTray);
            PlotObject('green_tray.ply' , self.locationGreenTray);
            PlotObject('table.ply' , self.locationTable);
            PlotObject('fire.ply' , self.locationFire);
            PlotObject('lightcurtain.ply' , self.locationLightCurtain1);
            PlotObject('lightcurtain.ply' , self.locationLightCurtain2);
            PlotObject('lightcurtain.ply' , self.locationLightCurtain3);
            PlotObject('lightcurtain.ply' , self.locationLightCurtain4);
            PlotObject('estop.ply' , self.LocationE_Stop);
            surf([-2,-2;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('s_concrete.jpg'),'FaceColor','texturemap');
            
        end
        
%%
        function PickAndPlace(self, usingRealRobot)
            self.usingRealRobot = usingRealRobot;
            if self.usingRealRobot  
                rosshutdown();
                rosinit();
            end
            
            self.robot.MoveRealDobot(self.qHomeReal, self.usingRealRobot);
            self.robot.model.animate(self.qHome);
            
%             input("Press Enter to Start");
            
            rotation = rpy2r(0, 0, 0);
            
 % Pick Red 1
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationRed1'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationRed1'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Red 2
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationRed2'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);

            for i = 1:50
                if ~self.eStopState
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationRed2'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationRed'+[0;0;0.0023]), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenRed2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0,   self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Green 1
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen1'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen1'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);


            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Green 2
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen2'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);

            for i = 1:50
                if ~self.eStopState
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationGreen2'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationGreen'+[0;0;0.0023]), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenGreen2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Blue 1
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue1'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue1'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue1.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

% Pick Blue 2
            qPrepare1 = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue2'+[0; 0; 0.05]), self.qHome);
            qMatrix1 = jtraj(self.qHome, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix1(i,4) = pi - qMatrix1(i,2) - qMatrix1(i,3);
                qMatrix1(i,5) = 0;
    
                qTraj = qMatrix1(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix1(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPick = self.robot.model.ikcon(rt2tr(rotation, self.locationBlue2'), qPrepare1);
            qMatrix2 = jtraj(qPrepare1, qPick, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix2(i,4) = pi - qMatrix2(i,2) - qMatrix2(i,3);
                qMatrix2(i,5) = 0;
    
                qTraj = qMatrix2(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix2(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            self.robot.SetGripper(1, self.usingRealRobot);

            qMatrix3 = jtraj(qPick, qPrepare1, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix3(i,4) = pi - qMatrix3(i,2) - qMatrix3(i,3);
                qMatrix3(i,5) = 0;
    
                qTraj = qMatrix3(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix3(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPrepare2 = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'+[0;0;0.05]), qPrepare1);
            qMatrix4 = jtraj(qPrepare1, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix4(i,4) = pi - qMatrix4(i,2) - qMatrix4(i,3);
                qMatrix4(i,5) = 0;
    
                qTraj = qMatrix4(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix4(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qPlace = self.robot.model.ikcon(rt2tr(rotation, self.destinationBlue'+[0;0;0.0023]), qPrepare2);
            qMatrix5 = jtraj(qPrepare2, qPlace, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix5(i,4) = pi - qMatrix5(i,2) - qMatrix5(i,3);
                qMatrix5(i,5) = 0;
    
                qTraj = qMatrix5(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
    
                self.tokenBlue2.Move(self.robot.model.fkine(qTraj));
                end
            end
            qReal = qMatrix5(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
            pause(0.6);
            self.robot.SetGripper(0, self.usingRealRobot);

            qMatrix6 = jtraj(qPlace, qPrepare2, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix6(i,4) = pi - qMatrix6(i,2) - qMatrix6(i,3);
                qMatrix6(i,5) = 0;
    
                qTraj = qMatrix6(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix6(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);

            qMatrix7 = jtraj(qPrepare2, self.qHome, 50);
            for i = 1:50
                if ~self.eStopState
                qMatrix7(i,4) = pi - qMatrix7(i,2) - qMatrix7(i,3);
                qMatrix7(i,5) = 0;
    
                qTraj = qMatrix7(i,:);
                self.robot.model.animate(qTraj);
                drawnow();
                end
            end
            qReal = qMatrix7(end,:);
            qReal = [qReal(1) qReal(2) qReal(3) - pi/2 + qReal(2) qReal(5)];
            self.robot.MoveRealDobot(qReal, self.usingRealRobot);
        end

%%
        function CollisionAvoidance(self)
            
        end
              
%% Draw light curtain
function [front_,back_,left_,right_] = drawLightCurtain(self)
% function [front_,back_,left_,right_] = drawLightCurtain(defaultV,side)
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

%% IsCollisionCurtain
function result = IsCollisionCurtain(curtainDir,faces,vertex,faceNormals,returnOnceFound)
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
                disp('Warning');
                disp('Perimeter has been reached');
                disp('Stop!!!');
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

%% IsCollision
% This is based upon the output of questions 2.5 and 2.6
% Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
% and triangle obstacles in the environment (faces,vertex,faceNormals)
function result = IsCollision(self,robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
if nargin < 6
    returnOnceFound = true;
end
result = false;

for qIndex = 1:size(qMatrix,1)
    % Get the transform of every joint (i.e. start and end of every link)
    tr = self.GetLinkPoses(qMatrix(qIndex,:), robot);

    % Go through each link and also each triangle face
    for i = 1 : size(tr,3)-1    
        for faceIndex = 1:size(faces,1)
            vertOnPlane = vertex(faces(faceIndex,1)',:);
            [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
            if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
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
function [ transforms ] = GetLinkPoses(self, q, robot)

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
        %% Check LightCurtain  collision 
function [resultFront,resultBack,resultLeft,resultRight] = lightCurtainCheck(self,frontMatrix,backMatrix,leftMatrix,rightMatrix,faces,vertex,faceNormals)
    resultFront = self.IsCollisionCurtain(frontMatrix,faces,vertex,faceNormals);
    resultBack = self.IsCollisionCurtain(backMatrix,faces,vertex,faceNormals);
    resultLeft = self.IsCollisionCurtain(leftMatrix,faces,vertex,faceNormals);
    resultRight = self.IsCollisionCurtain(rightMatrix,faces,vertex,faceNormals);
end  

    end
end

