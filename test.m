%% 5 link - stick
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

%% stop button GUI
clear; clc;

A = testgui;
i = true;
a = 0;

while i
    pause(2);
    a = a + 1
    if a == 10 
        break; 
    end
    A.STOPButtonPushed(i);
end
%%
clear; clc;

qHomeReal  = [0 0.7862 0.7844 0];
qReal = qHomeReal;
qHomeModel = [qHomeReal(1) qHomeReal(2) (pi/2)-qHomeReal(2)+qHomeReal(3) (pi/2)-qHomeReal(3) qHomeReal(4)];
base = eye(4);

robot = Dobot2(base, qHomeModel);

% robot.MoveRealDobot(qHomeReal);















