%%
clear; clc; clf;


q = [0 1.13 1.84 0];
q1 = [0 q(2) q(3) (pi)-q(2)-q(3) q(4)];
base = eye(4);

robot = Dobot(base, q1);
hold on

C = [0 0 0 -0.1; 0 0 0 0.25; 0 0 0 0.15; 0 0 0 0];

A = robot.model.fkine(q1);
B = A + C;

qX = robot.model.ikcon(A);
qY = robot.model.ikcon(B);

qM = jtraj(qX, qY, 50);

input("");
for i = 1:50
    qqq = qM(i,:);
    robot.model.animate(qqq);
    drawnow();
%     pause(0.1);
end

% robot.model.teach();

% TR = PlotObject('tokenred.ply', transl(0.3,0,0));
% TG = PlotObject('tokengreen.ply', transl(0.3, 0.1, 0));
% TB = PlotObject('tokenblue.ply', transl(0.3, -0.1, 0));

%% 3 link
clear; clc;

L(1) = Link('d',0.138,   'a',0,     'alpha',-pi/2, 'offset',0,     'qlim',[deg2rad(-135),deg2rad(135)]);
L(2) = Link('d',0,       'a',0.135, 'alpha',0,     'offset',-pi/2, 'qlim',[deg2rad(5)   ,deg2rad(80)]);
L(3) = Link('d',0,       'a',0.147, 'alpha',0,     'offset',0,     'qlim',[deg2rad(15)  ,deg2rad(170)]);

model = SerialLink(L,'name','a');

q = [0 1.13 1.84];
qq = [0 q(2) q(3) (pi)-q(2)-q(3)];
model.plot(q)
model.teach()

%% 5 link
clear; clc;

L(1) = Link('d',0.138,   'a',0,     'alpha',-pi/2, 'offset',0,     'qlim',[deg2rad(-135),deg2rad(135)]);
L(2) = Link('d',0,       'a',0.135, 'alpha',0,     'offset',-pi/2, 'qlim',[deg2rad(5)   ,deg2rad(80)]);
L(3) = Link('d',0,       'a',0.147, 'alpha',0,     'offset',0,     'qlim',[deg2rad(15)  ,deg2rad(170)]);
L(4) = Link('d',0,       'a',0.061, 'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-90) ,deg2rad(90)]);
L(5) = Link('d',-0.0385, 'a',0,     'alpha',0,     'offset',0,     'qlim',[deg2rad(-85) ,deg2rad(85)]);

model = SerialLink(L,'name','a');

q = [0 1.13 1.84 0];
qq = [0 q(2) q(3) (pi)-q(2)-q(3) q(4)];
model.plot(qq)
model.teach()