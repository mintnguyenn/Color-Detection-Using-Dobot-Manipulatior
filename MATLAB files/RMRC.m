clear; clc; clf

xInput1 = 0.21;
xInput2 = 0.21;
yInput1 = 0;
yInput2 = 0;
zInput1 = 0;
zInput2 = 0.1;
%%
qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 0.138);
% 
% % Display the robot
dobot = Dobot2(base, qHome);
hold on;

t = 10;                             % Total time
deltaT = 1;                         % Control frequency
steps = t/deltaT;                   % No. of steps for simulation
epsilon = 0.1;                      % Manipulability
W = diag([1 1 1 0.1 0.1 0.1]);      % Weighting matrix for the velocity vector
    
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,5);       % Array for joint angles
qdot = zeros(steps,5);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory

positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

%% Trajectory
    s = lspb(zInput1, zInput2, steps);        % Trapezoidal trajectory scalar
    for i=1:steps
        x(1,i) = xInput1;            % Points in x
        x(2,i) = yInput1;            % Points in y / no change if wanna go in straight line
        x(3,i) = s(i);              % Points in z
        theta(1,i) = 0;             % Roll angle
        theta(2,i) = 0;             % Pitch angle
        theta(3,i) = 1;             % Yaw angle
    end

%%
    T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
    q0 = zeros(1,5);                                                            % Initial guess for joint angles
    qMatrix(1,:) = dobot.model.ikcon(T, qHome);  % Solve joint angles to achieve first waypoint

% Loop
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
    J = dobot.model.jacob0(qMatrix(i,:));                                   % Get Jacobian at current joint state
    m(i) = sqrt(abs(det(J(1:3,1:3)*J(1:3,1:3)')));                          %might wanna change here, I added abs for det J*J' in case in become negative which made qMatrix turn in to double imaginary type
    if m(i) < epsilon                                                       % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
%     invJ = inv(J'*J + lambda *eye(5))*J';                                   % DLS equation
    invJ = pinv(J);
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the         vector)
    for j = 1:5                                                           % Loop through joints 1 to 5
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

%% Plot the result
pause()
figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
hold on;

for i = 1:steps
    dobot.model.animate(qMatrix(i,:));
    drawnow();
    pause(0.1)
end
