%% setup joystick
close all
clear all 
clc 

id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information
%% Set up robot
qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 0.138);
Dobot = Dobot2(base,qHome);
Dobot.model.tool = transl(0,0,0); % Define tool frame on end-effector
hold on;
%% Start "real-time" simulation
HF = figure(1);         % Initialise figure to display robot
Dobot.model.delay = 0.001;    % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);
dt = 0.016;      % Set time step for simulation (seconds)
disp('PS4 control mode: ON')
locationRed     = [0.181  0   0.01];
tokenRed = PlotObject('tokenred.ply', locationRed);
doPick = 0;
doDrop = 0;
while 1 
    % read joystick
    [axes, buttons, povs] = read(joy);
    
    if buttons(4) == 1
        disp('PS4 control mode: OFF')
        break 
    end
       
    Kv = 0.5; 
    Kw = 0.5;
    
    vx = Kv*axes(2);
    vy = Kv*axes(1);
    vz = Kv*axes(5);
    
    wx = 0;
    wy = 0;
    wz = 0;
    
    
    
    if buttons(1) == 1
        doPick = 1;
    end
    if buttons(2) == 1
        doDrop = 1;
    end
    x_vel = [vx vy vz wx wy wz]';
    
    x_vel((x_vel.^2)<0.01) = 0;
    
    lambda =0.1; 
    J = Dobot.model.jacob0(qHome);
    J_dls_inv = inv((J'*J) + lambda*eye(5))*J';
    
    q_vel = J_dls_inv*x_vel; 
    
    qHome = qHome + (dt*q_vel)';
    
    
    % Update plot
    Dobot.model.animate(qHome);  
    
    if doPick == 1 && doDrop == 0
        tokenRed.Move(Dobot.model.fkine(qHome));
    end
    
    if doDrop == 1
        doPick = 0;
        doDrop = 0;
    end
end



