clc;

if false
A = rossubscriber('/camera/color/image_raw');
pause(1);
RGB = readImage(A.LatestMessage);
end

RGB = imread('test.png');
I = rgb2hsv(RGB);

% Green
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

BGreen = bwboundaries(BWGreen,'noholes');

% Red
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

BRed = bwboundaries(BWRed,'noholes');
%%
            BRed   = NoiseFilter(BRed);
            BGreen = NoiseFilter(BGreen);
            
            xRed1   = BRed{1,1}(:,2);
            yRed1   = BRed{1,1}(:,1);
            plot(xRed1, yRed1,'r');
            hold on;
            axis([0 1920 0 1080]);
            
            xRed2   = BRed{2,1}(:,2);
            yRed2   = BRed{2,1}(:,1);
            plot(xRed2, yRed2,'r');
            
            xGreen1 = BGreen{1,1}(:,2);
            yGreen1 = BGreen{1,1}(:,1);
            plot(xGreen1, yGreen1,'g');
%             xGreen2 = BGreen{2,1}(:,2);
%             yGreen2 = BGreen{2,1}(:,1);
            
            [xRed1Center, yRed1Center] = Circlefit(xRed1, yRed1); plot(xRed1Center, yRed1Center,'*');
            [xRed2Center, yRed2Center] = Circlefit(xRed2, yRed2); plot(xRed2Center, yRed2Center,'*');
            [xGreen1Center, yGreen1Center] = Circlefit(xGreen1, yGreen1); plot(xGreen1Center, yGreen1Center,'*');
%             [xGreen2Center, yGreen2Center] = Circlefit(xGreen2, yGreen2);