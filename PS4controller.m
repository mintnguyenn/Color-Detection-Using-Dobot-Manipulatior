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

%% Start "real-time" simulation

HF = figure(1);         % Initialise figure to display robot
Dobot.model.delay = 0.001;    % Set smaller delay when animating
set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 180;  % Set duration of the simulation (seconds)
dt = 0.016;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time

while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    Kv = 0.5; 
    Kw = 0.5;
    
    vx = Kv*axes(2);
    vy = Kv*axes(1);
    vz = Kv*axes(5);
    
    wx = Kw*(buttons(3)-buttons(2));
    wy = Kw*(buttons(5)-buttons(6));
    wz = Kw*(buttons(7)-buttons(8));
    
    x_vel = [vx vy vz wx wy wz]';
    
    x_vel((x_vel.^2)<0.01) = 0;
    
    lambda =0.1; 
    J = Dobot.model.jacob0(qHome);
    J_dls_inv = inv((J'*J) + lambda*eye(5))*J';
    
    q_vel = J_dls_inv*x_vel; 
    
    qHome = qHome + (dt*q_vel)';
    
    
    % Update plot
    Dobot.model.animate(qHome);  
    
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end



