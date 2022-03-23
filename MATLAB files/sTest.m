%% Begin
%Task need to be done:
% - Make the robot retreat from a "simulated safety symbol" using 
% VISUAL SERVOVING and RMRC
% - Helping team with "GUI"
% - Make sure understand LAB 8 + LAB 9 

% Can check out the RMRC section below 

%% Setup
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc
clf

%% Set up environment
% this section follow the test file by Minh
q = [0 1.13 1.84 0];
qq = [0 q(2) q(3) (pi/2)-q(2)-q(3) q(4)];
base = eye(4);
dobot = Dobot(base, qq);

% try to use my environment 
% enableTheEnvironment();
% hold on; %uncmt this if he enableTheEnvironment not on
surf([-1.7,-1.7;-1.7,-1.7],[-3,3;-3,3],[1.7,1.7;0,0],'CData',imread('s_Robotics.jpg'),'FaceColor','texturemap');
surf([-2,-2;3,3],[-3,3;-3,3],[0.01,0.01;0.01,0.01],'CData',imread('s_concrete.jpg'),'FaceColor','texturemap');
axis equal;

%% RMRC
% just run this section is enough for testing RMRC
%Section 1.1 Lab 9
clear all; clc; clf;

qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 0.138);

dobot = Dobot(base, qHome);
hold on
axis ([-0.5, 0.5, -0.5, 0.5, -0.6, 0.8])

% z = -0.03;
% R1 = [0.27  0   z];
% qPrepare1 = robot.model.ikcon(rt2tr(rotation, R1'+[0; 0; 0.05]), qHome);

t = 10;             % Total time
deltaT = 1;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;   % Manipulability
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

% 1.2 Lab 9
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,5);       % Array for joint angles
qdot = zeros(steps,5);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% 1.3 Lab 9
s = lspb(0.2,0.4,steps);                % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = 0.26; % Points in x
    x(2,i) = 0; % Points in y / no change if wanna go in straight line
    x(3,i) = s(i); % Points in z
    theta(1,i) = 0;                 % Roll angle 
    theta(2,i) = 0;            % Pitch angle
    theta(3,i) = 1;                 % Yaw angle
end
n = zeros(4,4,steps);
for i = 1:steps
T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,i);zeros(1,3) 1];          % Create transformation of first point and angle
q0 = zeros(1,5);                                                            % Initial guess for joint angles
qMatrix(i,:) = dobot.model.ikcon(T, q0);  % Solve joint angles to achieve first waypoint
n(:,:,i) = T;
k = qMatrix;
end
% trajectory = dobot.model.fkin

% 1.4
for i = 1:steps-1
    T = dobot.model.fkine(qMatrix(i,:));                                           % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);         % error                                	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                                % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = dobot.model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    m(i) = sqrt(abs(det(J(1:3,1:3)*J(1:3,1:3)'))); %might wanna change here, I added abd for det J*J' in case in become negative which made qMatrix turn in to double imaginary type
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(5))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:5                                                           % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < dobot.model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > dobot.model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
dobot.model.animate(qMatrix)
for i = 1:steps
    dobot.model.animate(qMatrix(i,:));
    drawnow();
    pause(0.2)
end
dobot.model.plot(qMatrix,'trail','r-')

figure(2)
subplot(2,1,1)
plot(positionError'*1000,'LineWidth',1)
refline(0,0)
xlabel('Step')
ylabel('Position Error (mm)')
legend('X-Axis','Y-Axis','Z-Axis')


%% test lab 8
clc
clear all
clf
q = deg2rad([0 45 110 0 0]);
qq = [0 q(2) q(3) pi-q(2)-q(3) 0];
base = eye(4);
dobot = Dobot(base, qq);
dobot.model.animate([0 0 0 0 0]);

% stop symbol
surf([-0.05,-0.05;0.05,0.05],[-0.05,0.05;-0.05,0.05],[0,0;0,0],'CData',imread('stop.jpg'),'FaceColor','texturemap');

% display = surf([0.05,0.05;-0.05,-0.05],[-0.05,0.05;-0.05,0.05],[0,0;0,0],'CData',imread('stop.jpg'),'FaceColor','texturemap');
% hold on

axis ([-0.5, 0.5, -0.5, 0.5, -0.6, 0.8])
% 3D point
%test z
z = -0.1
P1 = [0.18,  0.05, z];
P2 = [0.18, -0.05, z];
P3 = [0.23, -0.05, z];
P4 = [0.23,  0.05, z];
Points = [P1; P2; P3; P4]';

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'DOBOTcamera');

% Frame rate

% Gain of the controller
lamda = 0.6;

% Depth of the IBVS
depth = mean(Points(3,:)); %mean Z axis

% Display DObot
Tc = dobot.model.fkine(q) * trotx(pi);
dobot.model.animate(qq);
drawnow

% plot camera
cam.T = Tc;

% Display points and the camera (3D)
cam.plot_camera('Tcam', Tc, 'label', 'scale', 0.02);
plot_sphere(Points, 0.02, 'b') 

p = cam.plot(Points, 'Tcam', Tc);
hold on;
x = plot3(Tc(1,4), Tc(2,4), Tc(3,4), 'r.', 'MarkerSize', 18)


% Points_h = plot3(Points(1,1), Points(2,1), Points(3,1), 'r.', 'MarkerSize', 10);
% Points_h = plot3(Points(1,2), Points(2,2), Points(3,2), 'g.', 'MarkerSize', 10);
% Points_h = plot3(Points(1,3), Points(2,3), Points(3,3), 'b.', 'MarkerSize', 10);
% Points_h = plot3(Points(1,4), Points(2,4), Points(3,4), 'm.', 'MarkerSize', 10);
%%
P1 = [0.4530, 0.1495, 0.5976] - [0.2,0,0]; 
P2 = [0.6422, 0.1495, 0.5976] - [0.2,0,0];
P3 = [0.6422, 0.0254, 0.5976] - [0.2,0,0];
P4 = [0.4530, 0.0254, 0.5976] - [0.2,0,0];

points = [P1; P2; P3; P4]';

% points_h = plot3(points(1,1), points(2,1), points(3,1), 'r.', 'MarkerSize', 18);
% points_h = plot3(points(1,2), points(2,2), points(3,2), 'g.', 'MarkerSize', 18);
% points_h = plot3(points(1,3), points(2,3), points(3,3), 'b.', 'MarkerSize', 18);
% points_h = plot3(points(1,4), points(2,4), points(3,4), 'm.', 'MarkerSize', 18);

%%

