 clc;

% A = rossubscriber('/camera/color/image_raw');
% pause(1);
% RGB = readImage(A.LatestMessage);
% imshow(I)
RGB = imread('test.png');
I = rgb2hsv(RGB);
% Define thresholds for channel 1 based on histogram settings
greenChannel1Min = 0.323;
greenChannel1Max = 0.546;

% Define thresholds for channel 2 based on histogram settings
greenChannel2Min = 0.186;
greenChannel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
greenChannel3Min = 0.000;
greenChannel3Max = 1.000;

greenSliderBW = (I(:,:,1) >= greenChannel1Min ) & (I(:,:,1) <= greenChannel1Max) & ...
    (I(:,:,2) >= greenChannel2Min ) & (I(:,:,2) <= greenChannel2Max) & ...
    (I(:,:,3) >= greenChannel3Min ) & (I(:,:,3) <= greenChannel3Max);
BWGreen = greenSliderBW;

[B1, L1, n1, A1] = bwboundaries(BWGreen,'noholes');

%% Red
% Define thresholds for channel 1 based on histogram settings
redChannel1Min = 0.909;
redChannel1Max = 0.126;

% Define thresholds for channel 2 based on histogram settings
redChannel2Min = 0.432;
redChannel2Max = 0.844;

% Define thresholds for channel 3 based on histogram settings
redChannel3Min = 0.000;
redChannel3Max = 1.000;

% Create mask based on chosen histogram thresholds
redSliderBW = ( (I(:,:,1) >= redChannel1Min) | (I(:,:,1) <= redChannel1Max) ) & ...
    (I(:,:,2) >= redChannel2Min ) & (I(:,:,2) <= redChannel2Max) & ...
    (I(:,:,3) >= redChannel3Min ) & (I(:,:,3) <= redChannel3Max);
BWRed = redSliderBW;

[B2, L2, n2, A2] = bwboundaries(BWRed,'noholes');
%%

BGreen = cell2mat(B1);
x = BGreen(:,2)';
% x = linspace(0,1080);
y = BGreen(:,1)';

plot(x,y,'g');
axis([0 1920 0 1080]);

hold on;
% BRed1 = cell2mat(B2{1,1});
% u = BRed1(:,2)';
% v = BRed1(:,1)';

u1 = B2{1,1}(:,2);
v1 = B2{1,1}(:,1);

plot(u1,v1,'r'); hold on;

u2 = B2{4,1}(:,2);
v2 = B2{4,1}(:,1);

plot(u2,v2,'r'); hold on;

u3 = B2{5,1}(:,2);
v3 = B2{5,1}(:,1);

plot(u3,v3,'r');
% u = [u1; u2; u3];
% v = [v1; v2; v3];
% plot(u,v);