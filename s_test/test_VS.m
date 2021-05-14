clear all
clc
clf

qHomeReal  = [0 0.7862 0.7844 0];
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
qHome = qHomeModel;
base = transl(0, 0, 0.138);
q0 = [qHomeReal(1); qHomeReal(2);(pi/2)-qHomeReal(2)+qHomeReal(3); (pi/2)-qHomeReal(3); qHomeReal(4)];
% q0 = [0.0038; 0.7736; 1.5700; 0.7979; -0.0382];


pStar = [662 362 362 662; 362 362 662 662];

% 3D point
%test z
z = -0.05;
P1 = [0.25,  0.05, z];
P2 = [0.30, 0.05, z];
P3 = [0.30, 0, z];
P4 = [0.25,  0, z];
Points = [P1; P2; P3; P4]';
% Points = [0.18, 0.18, 0.23, 0.23;
%          0.025, -0.025, -0.025, 0.025;
%           z, z, z, z];

dobot = Dobot2(base, qHome);
hold on
axis ([-0.5, 0.5, -0.5, 0.5, -0.6, 0.8])

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'DOBOTcamera');

% Frame rate
fps = 25;

% Gain of the controller
lamda = 0.6;

% Depth of the IBVS
depth = mean(Points(3,:)); %mean Z axis

% Display DObot
Tc0 = dobot.model.fkine(q0) * trotx(pi);
dobot.model.animate(q0');
drawnow()

% plot camera
cam.T = Tc0;
hold on;
% Display points and the camera (3D)
cam.plot_camera('Tcam', Tc0, 'label', 'scale', 0.02);
plot_sphere(Points, 0.01, 'b') 
lighting gouraud
light

p = cam.plot(Points, 'Tcam', Tc0);

cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(Points, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(Points);    % show initial view]


vel_p = [];
uv_p = [];
history = [];
% Loop
%%
ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(Points)
        
        % compute image plane error as a column
        e = pStar-uv   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), Points);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lamda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = dobot.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;
        dobot.model.animate(q');

        %Get camera location
        Tc = dobot.model.fkine(q) * trotx(pi);
        cam.T = Tc;

        drawnow()
        
        % update the history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist];

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes